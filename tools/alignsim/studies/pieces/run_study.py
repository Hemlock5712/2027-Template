"""Game-piece pickup study: every strategy x every stress, fixed seeds, single process.

    cd tools/alignsim/studies/pieces
    python3 check_signs.py            # tx/ty sign convention, end to end
    python3 run_study.py 150          # main matrix + ablations -> results/*.json, tables on stdout
"""

import functools
import json
import math
import os
import sys
import time
from dataclasses import replace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from configs import BASE, SCENARIOS, STRATEGIES  # noqa: E402
from piece_world import run_set  # noqa: E402
from strategies import Hybrid, KalmanPiece  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
WIDTHS = (0.4, 0.3, 0.2)  # intake width W; narrower ones are scored from the same runs (|miss| <= W/2)


def summarize(rows):
    n = len(rows)
    caps = [r for r in rows if r["success"]]
    contacts = [r for r in rows if r["outcome"] != "timeout"]
    miss = np.array([r["miss"] for r in contacts]) if contacts else np.zeros(0)
    tt = np.array([r["t"] for r in caps]) if caps else np.zeros(0)
    out = {"n": n}
    for w in WIDTHS:
        out[f"succ_W{int(w * 100)}"] = 100.0 * sum(1 for r in caps if r["miss"] <= w / 2) / n
    for k in ("too_fast", "bump", "timeout"):
        out[k] = 100.0 * sum(1 for r in rows if r["outcome"] == k) / n
    out["no_lock"] = 100.0 * sum(1 for r in rows if r["t_lock"] is None) / n
    out["t_med"] = float(np.median(tt)) if len(tt) else float("nan")
    out["t_p90"] = float(np.percentile(tt, 90)) if len(tt) else float("nan")
    out["miss_med"] = float(np.median(miss) * 100) if len(miss) else float("nan")
    out["miss_p95"] = float(np.percentile(miss, 95) * 100) if len(miss) else float("nan")
    vr = [r["v_rel"] for r in contacts]
    out["v_rel_med"] = float(np.median(vr)) if vr else float("nan")
    errs = [r["est_err"] for r in contacts if r["est_err"] is not None]
    out["est_x_p95"] = float(np.percentile([abs(e[0]) for e in errs], 95) * 100) if errs else float("nan")
    out["est_y_p95"] = float(np.percentile([abs(e[1]) for e in errs], 95) * 100) if errs else float("nan")
    return out


def run_block(name, scenarios, strategies, n, seed0=0):
    path = os.path.join(OUT, f"{name}.json")
    res = {}
    if os.path.exists(path):
        with open(path) as f:
            res = json.load(f)
    for sk, sc in scenarios.items():
        for gk, factory in strategies.items():
            key = f"{sk} || {gk}"
            if key in res and res[key]["n"] == n:
                continue
            t0 = time.time()
            rows = run_set(sc, factory, n, seed0=seed0)
            res[key] = summarize(rows)
            print(f"{name}: {key}: W40 {res[key]['succ_W40']:.1f}%  ({time.time() - t0:.0f}s)", flush=True)
            with open(path, "w") as f:
                json.dump(res, f, indent=1)
    return res


def fmt(v, f="{:.0f}"):
    return "-" if v is None or (isinstance(v, float) and math.isnan(v)) else f.format(v)


def matrix(res, scenarios, strategies, metric, f="{:.0f}"):
    lines = ["| stress | " + " | ".join(strategies) + " |", "|---|" + "---:|" * len(strategies)]
    for sk in scenarios:
        cells = [fmt(res.get(f"{sk} || {gk}", {}).get(metric), f) for gk in strategies]
        lines.append(f"| {sk} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def detail(res, sk, strategies):
    cols = [("succ_W40", "succ% W0.4"), ("succ_W20", "W0.2"), ("bump", "bump%"), ("timeout", "timeout%"),
            ("too_fast", "too fast%"), ("no_lock", "no lock%"), ("t_med", "t med s"), ("t_p90", "t p90"),
            ("miss_med", "miss med cm"), ("miss_p95", "miss p95"), ("v_rel_med", "v_rel m/s")]
    lines = [f"**{sk}**", "", "| strategy | " + " | ".join(c[1] for c in cols) + " |", "|---|" + "---:|" * len(cols)]
    for gk in strategies:
        s = res.get(f"{sk} || {gk}")
        if s is None:
            continue
        cells = [fmt(s[k], "{:.2f}" if k in ("t_med", "t_p90", "v_rel_med") else
                     "{:.1f}" if k.startswith("miss") else "{:.0f}") for k, _ in cols]
        lines.append(f"| {gk} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 150
    os.makedirs(OUT, exist_ok=True)
    t0 = time.time()
    strat = {k: v[1] for k, v in STRATEGIES.items()}
    main = run_block("main", SCENARIOS, strat, n)

    # ablation: how far past the bumper to aim (R with different reach), and KF process noise
    P = functools.partial
    abl_sc = {k: SCENARIOS[k] for k in ("base", "pitch 2 deg", "blind + pitch 2 deg", "late: 0.9-1.8 m, 2-3 m/s",
                                        "rolling 0.3-1 m/s", "ALL (static piece)", "ALL + rolling")}
    abl_st = {f"reach {r:.2f}": P(Hybrid, reach=r, tracker_kw={"edge_reject": False}) for r in (0.45, 0.40, 0.35, 0.30)}
    abl_st["R, KF q=1.0"] = P(Hybrid, reach=0.40, tracker_kw={"edge_reject": False, "q_acc": 1.0})
    abl_st["R, static KF"] = P(Hybrid, reach=0.40, tracker_kw={"edge_reject": False, "static": True})
    abl_st["R + edge reject"] = P(Hybrid, reach=0.40)
    abl_st["C, 4 m gate"] = KalmanPiece
    abl = run_block("ablation", abl_sc, abl_st, n)

    # camera mount: how much does seeing the piece into the intake matter (R and A)
    from piece_world import piece_camera
    cams = {
        "cam 0.5 m, 25 deg down (default)": BASE,
        "cam 0.6 m, 5 deg down (blind 0.5 m)": SCENARIOS["blind last 0.5 m"],
        "cam 0.5 m, 25 deg down, rolling": SCENARIOS["rolling 0.3-1 m/s"],
        "cam 0.6 m, 5 deg down, rolling": replace(SCENARIOS["rolling 0.3-1 m/s"], cameras=SCENARIOS["blind last 0.5 m"].cameras),
        "cam 0.6 m, 5 deg down, 30 fps -> 15 fps": replace(SCENARIOS["blind last 0.5 m"],
                                                            cameras=(piece_camera(x=0.20, z=0.60, pitch_down_deg=5.0, fps=15.0),)),
    }
    camres = run_block("camera", cams, {k: strat[k] for k in ("A", "B", "C", "R")}, n)

    keys = list(STRATEGIES)
    print("\n## success % (W = 0.4 m)\n")
    print(matrix(main, SCENARIOS, keys, "succ_W40"))
    print("\n## success % if the intake were W = 0.2 m\n")
    print(matrix(main, SCENARIOS, keys, "succ_W20"))
    print("\n## median time to intake (s)\n")
    print(matrix(main, SCENARIOS, keys, "t_med", "{:.2f}"))
    print("\n## lateral miss p95 (cm)\n")
    print(matrix(main, SCENARIOS, keys, "miss_p95", "{:.1f}"))
    for sk in ("base", "blind + pitch 2 deg", "late: 0.9-1.8 m, 2-3 m/s", "rolling 0.3-1 m/s", "two pieces 0.4-0.8 m apart",
               "blind + shove", "ALL (static piece)", "ALL + rolling"):
        print("\n" + detail(main, sk, keys))
    print("\n## ablation (success % W0.4 / W0.2 / median t)\n")
    for metric, f in (("succ_W40", "{:.0f}"), ("succ_W20", "{:.0f}"), ("t_med", "{:.2f}"), ("miss_p95", "{:.1f}")):
        print(f"\n{metric}\n")
        print(matrix(abl, abl_sc, list(abl_st), metric, f))
    print("\n## camera mount\n")
    for metric, f in (("succ_W40", "{:.0f}"), ("succ_W20", "{:.0f}"), ("t_med", "{:.2f}")):
        print(f"\n{metric}\n")
        print(matrix(camres, cams, ["A", "B", "C", "R"], metric, f))
    print(f"\ntotal {time.time() - t0:.0f}s")
