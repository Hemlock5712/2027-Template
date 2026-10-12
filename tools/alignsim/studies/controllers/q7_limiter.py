"""Q7: the 2026-style physics limiter (traction friction circle + motor torque-speed limit at a model
current, hardware current limit at 360 A) vs a fixed accel limit. Move bench + final tag approach.

    cd tools && python3 alignsim/studies/controllers/q7_limiter.py [n] [procs] [bench|world|all]
"""

import math
import multiprocessing as mp
import sys
from dataclasses import replace
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.params import RobotParams  # noqa: E402
from alignsim.studies.controllers.bench import BenchCfg, summarize_moves  # noqa: E402
from alignsim.studies.controllers.common import make_moves, md, run_batch  # noqa: E402
from alignsim.studies.controllers.variants import VectorBrakePhys, VectorBrakeX  # noqa: E402

LAW = dict(decel=6.0, accel=10.0, kp=6.0, react=0.05, wmax=6.0, wdecel=6.0, waccel=8.0, kp_w=5.0, wreact=0.05,
           deadband=0.003, wdeadband=math.radians(0.3))
HW = 360.0  # hardware current limit (A)


def fixed(vmax=4.0):
    return partial(VectorBrakeX, vmax=vmax, **LAW)


def phys(vmax=4.0, traction=10.0, model_current=150.0):
    return partial(VectorBrakePhys, vmax=vmax, traction=traction, model_current=model_current, **LAW)


CTRL = {
    "fixed accel 10 (current DriveToPose)": fixed(),
    "physics: traction 10, motor model 150 A": phys(),
    "physics, vmax 4.4": phys(vmax=4.4),
    "physics, traction 10.8 (1.1 g, 2026)": phys(traction=10.8),
}
COLS = [("t_in5", "t 5cm", "{:.2f}"), ("t_settle", "t settle", "{:.2f}"), ("over_p95_cm", "over p95 cm", "{:.1f}"),
        ("yaw_over_p95", "yaw over p95°", "{:.1f}"), ("slip_ms", "slip ms", "{:.0f}"),
        ("odo_p95_cm", "odo p95 cm", "{:.1f}"), ("pk_acc", "peak cmd accel", "{:.1f}")]


def _bench(args):
    label, cg, moves = args
    rp = replace(RobotParams(), stator_limit=HW, cg_height=cg)
    return (label, cg), summarize_moves(run_batch(CTRL[label], moves, BenchCfg(t_end=6.0, rp=rp)), 6.0)


def bench(n, procs):
    for kind, moves in (("from rest", make_moves(n, 11, speed=(0.0, 0.0))),
                        ("moving start 0.5-3 m/s", make_moves(n, 12, speed=(0.5, 3.0)))):
        jobs = [(k, cg, moves) for cg in (0.0, 0.2) for k in CTRL]
        with mp.get_context("fork").Pool(procs) as pool:
            res = dict(pool.map(_bench, jobs))
        rows = [(f"CG {cg} m: {k}", res[(k, cg)]) for cg in (0.0, 0.2) for k in CTRL]
        print(md(rows, COLS, f"Q7 limiter, hardware limit {int(HW)} A, {kind}, n={n}"), flush=True)


def world(procs):
    from alignsim.experiment import summarize, table
    from alignsim.studies.tag.bias import HARD, run_many
    from alignsim.studies.tag.final import RECOMMENDED, VERY_HARD
    for wname, sc in (("hard", HARD), ("very hard", VERY_HARD)):
        s2 = replace(sc, robot=replace(sc.robot, stator_limit=HW, cg_height=0.2))
        out = [(k, summarize(run_many(s2, replace(RECOMMENDED, controller=c), 100, "percam", procs)))
               for k, c in CTRL.items()]
        print(table(out, f"Final tag approach, CG 0.2 m, {int(HW)} A, {wname} world, n=100"), flush=True)


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    procs = int(sys.argv[2]) if len(sys.argv) > 2 else 4
    which = sys.argv[3] if len(sys.argv) > 3 else "all"
    if which in ("bench", "all"):
        bench(n, procs)
    if which in ("world", "all"):
        world(procs)
