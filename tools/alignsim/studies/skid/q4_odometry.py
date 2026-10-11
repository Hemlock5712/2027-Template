"""Q4 at the odometry level (cameras off): WPILib least squares vs (b) drop slipping modules vs
(f) IMU bridging, plus how sensitive the IMU detector/bridge is to accelerometer noise and bias.

    python3 alignsim/studies/skid/q4_odometry.py [n] [--period 0.010]
"""

import functools
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from skidsim import Config, Push, SkidWorld, window_error  # noqa: E402

from alignsim.controllers import VectorBrake  # noqa: E402
from alignsim.params import LoopParams  # noqa: E402
from alignsim.world import Errors, Scenario, Strategy  # noqa: E402


def scen(period, **kw):
    base = dict(cameras=(), start_dist=4.0, t_end=4.0, errors=Errors(wheel_radius=0.0),
                loop=LoopParams(odom_period=period))
    base.update(kw)
    return Scenario(**base)


def cases(period):
    acc12 = Strategy(controller=functools.partial(VectorBrake, accel=12))
    push = lambda f, d: (Push(phase="time", t=(0.4, 1.6), force=(f, f), dur=(d, d)),)  # noqa: E731
    return {
        "clean, big turn": (scen(period, start_heading=math.radians(90)), Strategy(), {}),
        "accel 12": (scen(period), acc12, {}),
        "reversal 90 deg, accel 12": (scen(period, start_speed=2.0), acc12, {"start_vel_angle": math.radians(90)}),
        "reversal 180 deg, accel 12": (scen(period, start_speed=2.0), acc12, {"start_vel_angle": math.radians(180)}),
        "push 500 N x 0.5 s": (scen(period), Strategy(), {"pushes": push(500, 0.5)}),
        "push 800 N x 0.2 s": (scen(period), Strategy(), {"pushes": push(800, 0.2)}),
        "push 800 N x 0.5 s": (scen(period), Strategy(), {"pushes": push(800, 0.5)}),
    }


def drift(sc, st, cfg, n):
    d, alarms = [], []
    for seed in range(n):
        r = SkidWorld(sc, seed, cfg).run(st)
        s = r["series"]
        t_stop = r["t_done"] if r["done"] else float(s[-1, 0])
        d.append(100 * window_error(s, 0.0, t_stop)[0])
        alarms.append(r["flag_count"] > 0)
    return np.array(d), 100 * float(np.mean(alarms))


def fmt(d):
    return f"{np.median(d):.1f} / {np.percentile(d, 95):.1f}"


if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else 30
    period = float(sys.argv[sys.argv.index("--period") + 1]) if "--period" in sys.argv else 0.004
    cs = cases(period)
    modes = {"WPILib LS": "ls", "(b) drop slipping modules": "drop", "(f) IMU bridging": "bridge"}
    print(f"**Odometry drift over the approach, cm (median / p95), n={n}, {1 / period:.0f} Hz**\n")
    print("| case | " + " | ".join(modes) + " | skid flagged % |")
    print("|---|" + "---|" * (len(modes) + 1))
    for name, (sc, st, over) in cs.items():
        cells, flagged = [], 0.0
        for mode in modes.values():
            d, fl = drift(sc, st, Config(record=True, radius_error=0.0, odom=mode, **over), n)
            cells.append(fmt(d))
            flagged = fl
        print(f"| {name} | " + " | ".join(cells) + f" | {flagged:.0f} |", flush=True)

    print("\n**IMU sensitivity (IMU bridging): drift cm (median / p95); clean-run alarms %**\n")
    imus = {
        "nominal: noise 0.01 g, bias error 0.002 g": dict(imu_noise_g=0.01, imu_bias_resid_g=0.002),
        "noisy: 0.03 g, 0.002 g": dict(imu_noise_g=0.03, imu_bias_resid_g=0.002),
        "biased: 0.01 g, 0.01 g": dict(imu_noise_g=0.01, imu_bias_resid_g=0.01),
        "both: 0.03 g, 0.01 g": dict(imu_noise_g=0.03, imu_bias_resid_g=0.01),
    }
    sel = ["clean, big turn", "reversal 180 deg, accel 12", "push 800 N x 0.2 s", "push 800 N x 0.5 s"]
    print("| IMU | " + " | ".join(sel) + " |")
    print("|---|" + "---|" * len(sel))
    for label, kw in imus.items():
        cells = []
        for name in sel:
            sc, st, over = cs[name]
            d, fl = drift(sc, st, Config(record=True, radius_error=0.0, odom="bridge", **kw, **over), n)
            cells.append(fmt(d) + (f"; alarms {fl:.0f}%" if name.startswith("clean") else ""))
        print(f"| {label} | " + " | ".join(cells) + " |", flush=True)
