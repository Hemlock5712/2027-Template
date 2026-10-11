"""Q4a VectorBrake gain sweep on the move bench (perfect localization), one knob at a time from the
matched base, plus a few combined candidates.

    cd tools && python3 alignsim/studies/controllers/q4_tune_vb.py [n]
"""

import sys
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.studies.controllers.bench import BenchCfg, summarize_moves  # noqa: E402
from alignsim.studies.controllers.common import Timer, make_moves, md, run_batch, save  # noqa: E402
from alignsim.studies.controllers.variants import VectorBrakeX  # noqa: E402

BASE = dict(vmax=4.0, decel=4.0, accel=5.0, kp=4.0, react=0.05, wmax=6.0, wdecel=10.0, waccel=12.0, kp_w=5.0)


def vb(**kw):
    p = dict(BASE)
    p.update(kw)
    return partial(VectorBrakeX, **p)


SWEEP = {
    "base (matched)": vb(),
    "kp 6": vb(kp=6.0),
    "kp 8": vb(kp=8.0),
    "kp 12": vb(kp=12.0),
    "decel 5 / accel 6": vb(decel=5.0, accel=6.0),
    "decel 3 / accel 5": vb(decel=3.0),
    "react 0": vb(react=0.0),
    "react 0.1": vb(react=0.1),
    "rot 8/8 (wdecel/waccel)": vb(wdecel=8.0, waccel=8.0),
    "rot 6/6": vb(wdecel=6.0, waccel=6.0),
    "rot 10/6 (limiter < decel!)": vb(wdecel=10.0, waccel=6.0),
    "kp_w 8": vb(kp_w=8.0),
    "rotation_cap": vb(rotation_cap=True),
    "cand A: kp 8, rot 8/8": vb(kp=8.0, wdecel=8.0, waccel=8.0),
    "cand B: kp 8, rot 6/6, decel 5/6": vb(kp=8.0, wdecel=6.0, waccel=6.0, decel=5.0, accel=6.0),
    "cand C: kp 6, rot 8/8, decel 5/6": vb(kp=6.0, wdecel=8.0, waccel=8.0, decel=5.0, accel=6.0),
}

COLS = [("t_in5", "t 5cm", "{:.2f}"), ("t_settle", "t settle", "{:.2f}"), ("t_settle_p95", "t settle p95", "{:.2f}"),
        ("over_p95_cm", "over p95 cm", "{:.1f}"), ("yaw_over_p95", "yaw over p95°", "{:.1f}"),
        ("slip_ms", "slip ms", "{:.0f}"), ("odo_p95_cm", "odo p95 cm", "{:.1f}"), ("pk_jerk", "pk jerk", "{:.0f}")]

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 80
    cfg = BenchCfg(t_end=6.0)
    moves = make_moves(n, 11, speed=(0.0, 0.0)) + make_moves(n, 12, speed=(0.5, 3.0))
    rows, out = [], {}
    for label, fac in SWEEP.items():
        with Timer(label):
            r = run_batch(fac, moves, cfg)
        s = summarize_moves(r, cfg.t_end)
        rows.append((label, s))
        out[label] = s
    t = md(rows, COLS, f"Q4 VectorBrake sweep, perfect localization, {n} rest + {n} moving starts")
    print(t)
    save("q4_tune_vb", out)
    with open(__file__.replace("q4_tune_vb.py", "results/q4_tune_vb.md"), "w") as f:
        f.write(t + "\n")
