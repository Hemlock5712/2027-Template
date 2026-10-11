"""Q3 waypoint chaining: stop-and-go vs pass-through on 2-3 waypoint routes, perfect localization.

Each leg gets a fresh controller seeded from the measured velocity (like chaining Java DriveToPose
commands). Pass-through legs set end_speed and hand over within switch_radius of the waypoint.

    cd tools && python3 alignsim/studies/controllers/q3_route.py [n]
"""

import math
import sys
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

import numpy as np  # noqa: E402

from alignsim.studies.controllers.bench import BenchCfg, Leg, Move, run_move  # noqa: E402
from alignsim.studies.controllers.common import Timer, md, save  # noqa: E402
from alignsim.studies.controllers.laws import MATCHED_PT  # noqa: E402
from alignsim.studies.controllers.q1b_fixes import VB_TUNED  # noqa: E402
from alignsim.studies.controllers.variants import Trailblazer581X  # noqa: E402

R = math.radians
# waypoints after the start (0, 0, 0); the last one is the stop
ROUTES = {
    "zigzag (3 legs, ~60° turns)": [(2.0, 1.0, R(45)), (4.0, -0.2, R(90)), (5.5, 0.8, R(180))],
    "dogleg (90° turn)": [(3.0, 0.0, 0.0), (3.0, 2.5, R(90))],
    "hairpin (135° turn)": [(3.0, 0.0, 0.0), (1.2, 1.8, R(180))],
}


def route_moves(wps, n, seed):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(n):
        pts = [(x + rng.uniform(-0.2, 0.2), y + rng.uniform(-0.2, 0.2), th + rng.uniform(-R(15), R(15)))
               for x, y, th in wps]
        out.append(pts)
    return out


def corner(pts, k, v_req, accel=4.0, r_max=0.4):
    """Corner rule: switch radius r = min(r_max, 40% of the shorter leg); end speed
    ve = sqrt(accel * r / tan(turn / 2)) so the tightest arc at ve fits inside the corner."""
    prev = (0.0, 0.0) if k == 0 else pts[k - 1]
    l1 = math.hypot(pts[k][0] - prev[0], pts[k][1] - prev[1])
    l2 = math.hypot(pts[k + 1][0] - pts[k][0], pts[k + 1][1] - pts[k][1])
    a1 = math.atan2(pts[k][1] - prev[1], pts[k][0] - prev[0])
    a2 = math.atan2(pts[k + 1][1] - pts[k][1], pts[k + 1][0] - pts[k][0])
    turn = abs(math.remainder(a2 - a1, 2 * math.pi))
    r = min(r_max, 0.4 * min(l1, l2))
    ve = v_req if turn < 1e-3 else min(v_req, math.sqrt(accel * r / math.tan(turn / 2)))
    return r, ve


def build(pts, end_speed, switch):
    legs = []
    for k, p in enumerate(pts):
        if k == len(pts) - 1:
            legs.append(Leg(p, 0.0, 0.15))
        elif switch == "corner":
            r, ve = corner(pts, k, end_speed)
            legs.append(Leg(p, ve, r, brake_at_switch=True))
        else:
            legs.append(Leg(p, end_speed, switch))
    return Move((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), legs)


def summarize_route(rows):
    def a(k):
        return np.array([r[k] for r in rows], dtype=float)

    wp_min = np.array([max(r["wp_min"]) for r in rows])
    wp_spd = np.array([np.mean(r["wp_speed"]) for r in rows])
    td = a("t_done")
    return {
        "t_done": float(np.nanmedian(td)) if np.any(~np.isnan(td)) else float("nan"),
        "done_%": 100 * float(np.mean(~np.isnan(td))),
        "dev_p95_cm": 100 * float(np.percentile(a("max_dev"), 95)),
        "wp_miss_p95_cm": 100 * float(np.percentile(wp_min, 95)),
        "wp_speed": float(np.median(wp_spd)),
        "err_p95_cm": 100 * float(np.percentile(a("err_end"), 95)),
        "slip_ms": 1000 * float(a("slip").mean()),
        "odo_p95_cm": 100 * float(np.percentile(a("odo_err"), 95)),
        "pk_acc": float(np.median(a("pk_acc"))),
        "acc_tv": float(np.median(a("acc_tv"))),
    }


COLS = [("t_done", "t done", "{:.2f}"), ("done_%", "done%", "{:.0f}"), ("wp_speed", "speed at wp", "{:.2f}"),
        ("wp_miss_p95_cm", "wp miss p95 cm", "{:.0f}"), ("dev_p95_cm", "path dev p95 cm", "{:.0f}"),
        ("err_p95_cm", "final err p95 cm", "{:.2f}"), ("slip_ms", "slip ms", "{:.0f}"),
        ("odo_p95_cm", "odo p95 cm", "{:.1f}"), ("pk_acc", "pk acc", "{:.1f}"), ("acc_tv", "accel TV", "{:.0f}")]

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    cfg = BenchCfg(t_end=9.0)
    configs = []
    for law, fac in MATCHED_PT.items():
        configs.append((f"{law} stop-and-go", fac, 0.0, 0.15))
        configs.append((f"{law} pass 1.5 m/s @15cm", fac, 1.5, 0.15))
    vb = VB_TUNED
    configs += [
        ("VB tuned stop-and-go", vb, 0.0, 0.15),
        ("VB tuned pass 1.5 @15cm", vb, 1.5, 0.15),
        ("VB tuned pass 1.5 @40cm", vb, 1.5, 0.40),
        ("VB tuned pass 2.5 @40cm", vb, 2.5, 0.40),
        ("VB tuned corner rule (r<=0.4 m, ve<=2.5)", vb, 2.5, "corner"),
        ("581 fixed pass 1.5 @15cm", partial(Trailblazer581X, floor=0.0, vec_accel=5.0, vmax=4.0, amax=5.0,
                                             wmax=6.0, walpha=12.0), 1.5, 0.15),
    ]
    out, tables = {}, []
    for rname, wps in ROUTES.items():
        pts_list = route_moves(wps, n, 31)
        rows = []
        for label, fac, es, sw in configs:
            with Timer(f"{rname}/{label}"):
                rs = [run_move(fac, build(p, es, sw), cfg, 2000 + i) for i, p in enumerate(pts_list)]
            s = summarize_route(rs)
            rows.append((label, s))
            out[f"{rname}/{label}"] = {"summary": s}
        tables.append(md(rows, COLS, f"Q3 route: {rname}, n={n}"))
        print(tables[-1], flush=True)
    save("q3_route", out)
    with open(__file__.replace("q3_route.py", "results/q3_route.md"), "w") as f:
        f.write("\n\n".join(tables) + "\n")
