"""Q6: physics and timing. Weight transfer (CG height) vs motor current limit, control-loop rate,
wheel force feedforward (equal or split by expected wheel load), and an onboard per-motor setpoint ramp
(Motion Magic velocity). Five fixed 3 m moves with 0-180 deg turns, from rest.

    cd tools && python3 alignsim/studies/controllers/q6_physics.py [procs]          # moves
    cd tools && python3 alignsim/studies/controllers/q6_physics.py [procs] world    # tag approach
"""

import math
import multiprocessing as mp
import sys
from dataclasses import replace

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.geom import rot  # noqa: E402
from alignsim.odometry import SwerveOdometry  # noqa: E402
from alignsim.params import RobotParams  # noqa: E402
from alignsim.plant import SwervePlant  # noqa: E402
from alignsim.studies.controllers.bench import wheel_forces  # noqa: E402
from alignsim.studies.controllers.variants import VectorBrakeX  # noqa: E402

LAW = dict(vmax=4.0, kp=6.0, react=0.05, wmax=6.0, wdecel=6.0, waccel=8.0, kp_w=5.0, wreact=0.05, deadband=0.003)
MOVES = [(3.0, 0.0, 0.0), (3.0, 0.0, math.pi / 2), (0.0, 3.0, 0.0), (2.0, 2.0, math.pi), (-3.0, 1.0, -math.pi / 2)]
SETUPS = {  # label: (loop period s, stator limit A, feedforward: None / "equal" / "load", onboard ramp)
    "50 Hz, 120 A": (0.02, 120.0, None, False),
    "50 Hz, 60 A (low slip current)": (0.02, 60.0, None, False),
    "50 Hz, 360 A (~Kraken stall, no cap)": (0.02, 360.0, None, False),
    "250 Hz, 120 A": (0.004, 120.0, None, False),
    "50 Hz + onboard ramp = accel (Motion Magic)": (0.02, 120.0, None, True),
    "250 Hz + equal force FF": (0.004, 120.0, "equal", False),
    "50 Hz + load-split force FF": (0.02, 120.0, "load", False),
    "250 Hz + load-split force FF": (0.004, 120.0, "load", False),
}
ACCELS = (6.0, 8.0, 10.0)
CGS = (0.0, 0.2, 0.3)


def run_move(cg, setup, accel, goal):
    period, slip_a, ff, ramp = SETUPS[setup]
    rp = replace(RobotParams(), cg_height=cg, stator_limit=slip_a, drive_ramp=accel if ramp else 0.0, steer_noise=0.0)
    c = VectorBrakeX(accel=accel, decel=min(accel - 1.0, 9.0), **LAW)
    p = SwervePlant(rp, (0, 0, 0))
    odo = SwerveOdometry(rp.module_positions, p.read_gyro(), p.read_modules(), (0, 0, 0))
    c.reset((0, 0, 0), (0, 0, 0))
    t, nxt, nodo, slip, t_end, last = 0.0, 0.0, 0.0, 0, None, (0.0, 0.0, 0.0)
    while t < 3.5:
        if t >= nodo - 1e-9:
            nodo += 0.004
            odo.update(p.read_gyro(), p.read_modules())
        if t >= nxt - 1e-9:
            nxt += period
            cmd = c.step(p.pose(), goal, p.field_velocity(), period)
            vr = rot((cmd[0], cmd[1]), -p.th)
            f = None
            if ff:
                a = rot(((cmd[0] - last[0]) / period, (cmd[1] - last[1]) / period), -p.th)
                f = wheel_forces(rp, a, (cmd[2] - last[2]) / period, cg if ff == "load" else 0.0)
            last = cmd
            p.set_chassis_robot(vr[0], vr[1], cmd[2], f)
        p.step(0.002)
        slip += any(p.slipping)
        if t_end is None and math.hypot(goal[0] - p.x, goal[1] - p.y) < 0.02 and math.hypot(p.vx, p.vy) < 0.05:
            t_end = t
        t += 0.002
    return slip * 0.002 * 1000, t_end or 3.5, math.hypot(odo.pose[0] - p.x, odo.pose[1] - p.y) * 100


def cell(args):
    cg, setup, accel = args
    res = [run_move(cg, setup, accel, g) for g in MOVES]
    return args, (sum(r[0] for r in res) / len(res), sum(r[1] for r in res) / len(res), max(r[2] for r in res))


def world(procs):
    """The final tag approach (recommended tag pipeline) at 120 A vs 60 A, with and without weight transfer."""
    from functools import partial

    from alignsim.experiment import summarize, table
    from alignsim.studies.tag.bias import HARD, run_many
    from alignsim.studies.tag.final import RECOMMENDED, VERY_HARD

    def vb(a, d):
        return partial(VectorBrakeX, accel=a, decel=d, wdeadband=math.radians(0.3), **LAW)

    rows = [("accel 5 / decel 4", 5, 4), ("accel 8 / decel 6", 8, 6), ("accel 10 / decel 6", 10, 6)]
    for wname, sc in (("hard", HARD), ("very hard", VERY_HARD)):
        out = []
        for cg in (0.0, 0.2):
            for slip_a in (120.0, 60.0):
                s2 = replace(sc, robot=replace(sc.robot, stator_limit=slip_a, cg_height=cg))
                for label, a, d in rows:
                    if slip_a == 60.0 and a != 10:
                        continue
                    st = replace(RECOMMENDED, controller=vb(a, d))
                    out.append((f"CG {cg} m, {int(slip_a)} A: {label}", summarize(run_many(s2, st, 100, "percam", procs))))
        print(table(out, f"Final tag approach, {wname} world, n=100"), flush=True)


if __name__ == "__main__":
    procs = int(sys.argv[1]) if len(sys.argv) > 1 else 4
    if len(sys.argv) > 2 and sys.argv[2] == "world":
        world(procs)
        sys.exit()
    jobs = [(cg, s, a) for cg in CGS for s in SETUPS for a in ACCELS]
    with mp.get_context("fork").Pool(procs) as pool:
        out = dict(pool.map(cell, jobs))
    for cg in CGS:
        print(f"\n**CG height {cg} m** - mean slip ms / mean time to 2 cm s / max odometry error cm (decel = accel - 1)\n")
        print("| setup | " + " | ".join(f"accel {a:g}" for a in ACCELS) + " |")
        print("|---|" + "---|" * len(ACCELS))
        for s in SETUPS:
            cells = [f"{out[(cg, s, a)][0]:.0f} / {out[(cg, s, a)][1]:.2f} / {out[(cg, s, a)][2]:.1f}" for a in ACCELS]
            print(f"| {s} | " + " | ".join(cells) + " |")
