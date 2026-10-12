"""Q5: how hard can VectorBrake accelerate? Sweep ACCEL/DECEL (and heading, slip current) on the move
bench, perfect localization; reports time, slip and the wheel-odometry error it leaves behind.

    cd tools && python3 alignsim/studies/controllers/q5_accel.py [n] [procs]
"""

import multiprocessing as mp
import sys
from dataclasses import replace
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.params import RobotParams  # noqa: E402
from alignsim.studies.controllers.bench import BenchCfg, summarize_moves  # noqa: E402
from alignsim.studies.controllers.common import make_moves, md, run_batch  # noqa: E402
from alignsim.studies.controllers.variants import VectorBrakeX  # noqa: E402

SHIP = dict(vmax=4.0, decel=4.0, accel=5.0, kp=6.0, react=0.05, wmax=6.0, wdecel=6.0, waccel=8.0, kp_w=5.0,
            wreact=0.05, deadband=0.003)


def vb(**kw):
    p = dict(SHIP)
    p.update(kw)
    return partial(VectorBrakeX, **p)


SLIP55 = replace(RobotParams(), stator_limit=55.0)
ROWS = {
    "ship: accel 5 / decel 4": (vb(), None),
    "accel 7 / decel 6": (vb(accel=7.0, decel=6.0), None),
    "accel 8.5 / decel 7.5": (vb(accel=8.5, decel=7.5), None),
    "accel 10 / decel 9": (vb(accel=10.0, decel=9.0), None),
    "accel 12 / decel 11 (past grip)": (vb(accel=12.0, decel=11.0), None),
    "accel 10 / decel 9, slip current 55 A": (vb(accel=10.0, decel=9.0), SLIP55),
    "accel 10 / decel 6, slip current 55 A": (vb(accel=10.0, decel=6.0), SLIP55),
    "ship: accel 5 / decel 4, slip current 55 A": (vb(), SLIP55),
    "SHIPPED: accel 10 / decel 6, slip current 60 A": (vb(accel=10.0, decel=6.0),
                                                      replace(RobotParams(), stator_limit=60.0)),
    "accel 10 / decel 6, 60 A, 68 kg robot": (vb(accel=10.0, decel=6.0),
                                             replace(RobotParams(), stator_limit=60.0, mass=68.0)),
    "accel 10 / decel 9, heading 10/12, wmax 9": (vb(accel=10.0, decel=9.0, wdecel=10.0, waccel=12.0, wmax=9.0),
                                                 None),
    "accel 10 / decel 9, vmax 4.4": (vb(accel=10.0, decel=9.0, vmax=4.4), None),
}
COLS = [("t_in5", "t 5cm", "{:.2f}"), ("t_settle", "t settle", "{:.2f}"), ("over_p95_cm", "over p95 cm", "{:.1f}"),
        ("yaw_over_p95", "yaw over p95°", "{:.1f}"), ("slip_ms", "slip ms", "{:.0f}"),
        ("odo_p95_cm", "odo p95 cm", "{:.1f}")]


def _run(args):
    label, moves, t_end = args
    fac, rp = ROWS[label]
    cfg = BenchCfg(t_end=t_end) if rp is None else BenchCfg(t_end=t_end, rp=rp)
    return label, summarize_moves(run_batch(fac, moves, cfg), t_end)


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    procs = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    for kind, moves in (("from rest", make_moves(n, 11, speed=(0.0, 0.0))),
                        ("moving start 0.5-3 m/s", make_moves(n, 12, speed=(0.5, 3.0)))):
        with mp.get_context("fork").Pool(procs) as pool:
            res = dict(pool.map(_run, [(k, moves, 6.0) for k in ROWS]))
        print(md([(k, res[k]) for k in ROWS], COLS, f"Q5 accel sweep, {kind}, n={n}"), flush=True)
