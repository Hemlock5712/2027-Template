"""Q2 collisions vs the odom-frame tracker + Q4 mitigations + Q5 (100 Hz odometry).

    python3 alignsim/studies/skid/q2_q4_alignment.py BLOCK [n]
BLOCK: pilot | phases | accel | radius | rate. Results are cached in $SKID_OUT (pickles).
"""

import functools
import math
import os
import pickle
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from skidsim import Config, Push, SkidGlobalVision, SkidTracker, SkidWorld  # noqa: E402

from alignsim.controllers import VectorBrake  # noqa: E402
from alignsim.experiment import summarize  # noqa: E402
from alignsim.geom import relative  # noqa: E402
from alignsim.params import LoopParams, RobotParams  # noqa: E402
from alignsim.trackers import FrameTracker  # noqa: E402
from alignsim.world import Scenario, Strategy, TrackerSpec  # noqa: E402

OUT = os.environ.get("SKID_OUT", os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))

PHASES = {
    "none": (),
    "prelock": (Push(phase="prelock", t=(0.05, 0.12)),),
    "inview": (Push(phase="inview", dist=0.6),),
    "coast": (Push(phase="coast", gap=0.06, dist=0.10),),
}

ODOM = TrackerSpec(FrameTracker, frame="odom")
INFLATE = TrackerSpec(SkidTracker, frame="odom", inflate=(0.05, math.radians(2)))
REACQ = TrackerSpec(SkidTracker, frame="odom", reacquire=True)
BOTH = TrackerSpec(SkidTracker, frame="odom", inflate=(0.05, math.radians(2)), reacquire=True)
FUSED = TrackerSpec(FrameTracker, frame="fused")

# label -> (strategy, config overrides)
VARIANTS = {
    "baseline (odom frame)": (Strategy(tracker=ODOM), {}),
    "(a) inflate filter on skid": (Strategy(tracker=INFLATE), {}),
    "(b) drop slipping modules": (Strategy(tracker=ODOM), {"odom": "drop"}),
    "(c) re-acquire (lock suspect after skid)": (Strategy(tracker=REACQ), {}),
    "(a)+(c)": (Strategy(tracker=BOTH), {}),
    "(e) fused frame + trust vision 5x after skid": (Strategy(tracker=FUSED, vision=SkidGlobalVision(factor=0.2)), {}),
    "(f) IMU bridging in odometry": (Strategy(tracker=ODOM), {"odom": "bridge"}),
    "(f)+(a)+(c)": (Strategy(tracker=BOTH), {"odom": "bridge"}),
}


def scenario(period=0.004, **kw):
    return Scenario(t_end=5.0, loop=LoopParams(odom_period=period), **kw)


def run_block(name, sc, st, cfg, n, seed0=0):
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"{name}.pkl")
    if os.path.exists(path):
        rows = pickle.load(open(path, "rb"))
        if len(rows) >= n:
            return rows[:n]
    t0 = time.time()
    rows = [SkidWorld(sc, seed0 + i, cfg).run(st) for i in range(n)]
    pickle.dump(rows, open(path, "wb"))
    print(f"  {name}: {time.time() - t0:.0f}s", flush=True)
    return rows


def push_stats(rows):
    """Extra columns: pushed %, flagged %, odometry error from push to done (cm, p95)."""
    pushed = [r for r in rows if r["pushes"] and r["pushes"][0]["fired"] is not None]
    out = {"pushed_%": 100 * len(pushed) / len(rows) if rows and rows[0]["pushes"] else float("nan")}
    flagged, drift = 0, []
    for r in pushed:
        p = r["pushes"][0]
        flagged += r["flag_count"] > 0
        if r["done"] and "odom_at" in p:
            dt = relative(p["true_at"], r["true_at_done"])
            do = relative(p["odom_at"], r["odom_at_done"])
            drift.append(math.hypot(do[0] - dt[0], do[1] - dt[1]) * 100)
    out["flagged_%"] = 100 * flagged / len(pushed) if pushed else float("nan")
    out["drift_p95_cm"] = float(np.percentile(drift, 95)) if drift else float("nan")
    out["drift_med_cm"] = float(np.median(drift)) if drift else float("nan")
    out["flag_any_%"] = 100 * np.mean([r["flag_count"] > 0 for r in rows])
    return out


COLS = [("success_%", "succ %", "{:.0f}"), ("false_done_%", "false done %", "{:.0f}"),
        ("finished_%", "finished %", "{:.0f}"), ("t_done_med", "t_done s", "{:.2f}"),
        ("pos_done_med_cm", "err@done med cm", "{:.2f}"), ("pos_done_p95_cm", "err@done p95 cm", "{:.1f}"),
        ("pushed_%", "pushed %", "{:.0f}"), ("flag_any_%", "skid flagged %", "{:.0f}"),
        ("drift_p95_cm", "odom drift push->done p95 cm", "{:.1f}")]


def table(results):
    lines = ["| case | " + " | ".join(c[1] for c in COLS) + " |", "|---|" + "---|" * len(COLS)]
    for label, s in results:
        cells = []
        for k, _, f in COLS:
            v = s.get(k, float("nan"))
            cells.append("-" if v is None or (isinstance(v, float) and math.isnan(v)) else f.format(v))
        lines.append(f"| {label} | " + " | ".join(cells) + " |")
    return "\n".join(lines)


def summary(rows):
    s = summarize(rows)
    s.update(push_stats(rows))
    return s


def by_force(rows):
    """Coast-phase breakdown by push force bin."""
    bins = [(300, 500), (500, 680), (680, 800.1)]
    out = []
    for lo, hi in bins:
        sub = [r for r in rows if r["pushes"][0]["fired"] is not None and lo <= r["pushes"][0]["force"] < hi]
        if sub:
            out.append((f"{lo}-{int(hi)} N (n={len(sub)})", summary(sub)))
    return out


def block_phases(n, period=0.004, variants=None, tag=""):
    results = {}
    for phase, pushes in PHASES.items():
        for label, (st, over) in VARIANTS.items():
            if variants is not None and label not in variants:
                continue
            if phase == "prelock" and label not in ("baseline (odom frame)", "(a)+(c)", "(f) IMU bridging in odometry"):
                continue
            if phase == "none" and label not in ("baseline (odom frame)", "(e) fused frame + trust vision 5x after skid",
                                                 "(f)+(a)+(c)"):
                continue  # no skid is ever flagged without a push, so the others equal the baseline
            cfg = Config(pushes=pushes, **over)
            key = f"{tag}{phase}__{label}"
            rows = run_block(_fname(key), scenario(period), st, cfg, n)
            results[(phase, label)] = rows
    return results


def _fname(key):
    return "".join(ch if ch.isalnum() else "_" for ch in key)[:120]


def block_accel(n):
    """(d) traction-aware limits vs aggressive acceleration (no pushes)."""
    acc = lambda a: functools.partial(VectorBrake, accel=a)  # noqa: E731
    cases = {
        "accel 6 (default)": (scenario(), Strategy(controller=acc(6), tracker=ODOM), {}),
        "accel 12": (scenario(), Strategy(controller=acc(12), tracker=ODOM), {}),
        "accel 12 + slip current 65 A": (scenario(robot=RobotParams(stator_limit=65.0)),
                                         Strategy(controller=acc(12), tracker=ODOM), {}),
        "accel 12 + (b) drop slipping modules": (scenario(), Strategy(controller=acc(12), tracker=ODOM), {"odom": "drop"}),
        "accel 12 + (f) IMU bridging": (scenario(), Strategy(controller=acc(12), tracker=ODOM), {"odom": "bridge"}),
        "accel 6 + slip current 65 A": (scenario(robot=RobotParams(stator_limit=65.0)),
                                        Strategy(controller=acc(6), tracker=ODOM), {}),
    }
    out = []
    for label, (sc, st, over) in cases.items():
        rows = run_block(_fname("accel__" + label), sc, st, Config(**over), n)
        out.append((label, summary(rows)))
    return out


if __name__ == "__main__":
    block = sys.argv[1] if len(sys.argv) > 1 else "pilot"
    n = int(sys.argv[2]) if len(sys.argv) > 2 else 80
    if block == "pilot":
        OUT = os.path.join(OUT, "pilot")
        res = block_phases(n)
        for phase in PHASES:
            print(f"\n**phase: {phase}**\n")
            print(table([(lab, summary(rows)) for (ph, lab), rows in res.items() if ph == phase]))
    elif block == "phases":
        res = block_phases(n)
        for phase in PHASES:
            print(f"\n**phase: {phase}, n={n}**\n")
            print(table([(lab, summary(rows)) for (ph, lab), rows in res.items() if ph == phase]))
        print("\n**coast phase by push force**\n")
        for lab in ("baseline (odom frame)", "(a)+(c)", "(f) IMU bridging in odometry", "(f)+(a)+(c)"):
            print(f"\n{lab}\n")
            print(table(by_force(res[("coast", lab)])))
    elif block == "accel":
        print(table(block_accel(n)))
    elif block == "radius":
        out = []
        for rad in (-0.02, 0.02):
            st, over = VARIANTS["baseline (odom frame)"]
            rows = run_block(_fname(f"radius_{rad}"), scenario(), st, Config(radius_error=rad), n)
            out.append((f"wheel radius error {rad * 100:+.0f}% (no push)", summary(rows)))
        print(table(out))
    elif block == "rate":
        keep = ["baseline (odom frame)", "(a)+(c)", "(f)+(a)+(c)"]
        for period, tag in ((0.004, ""), (0.010, "100hz_")):
            res = {}
            for phase in ("none", "coast"):
                for lab in keep:
                    st, over = VARIANTS[lab]
                    rows = run_block(_fname(f"{tag}{phase}__{lab}"), scenario(period), st,
                                     Config(pushes=PHASES[phase], **over), n)
                    res[(phase, lab)] = rows
            print(f"\n**odometry {1 / period:.0f} Hz, n={n}**\n")
            print(table([(f"{ph}: {lab}", summary(rows)) for (ph, lab), rows in res.items()]))
