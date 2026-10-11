"""Operating conditions sweep for the leading tracker: camera layout, frame rate, pixel noise,
clock bias (timestamp error), handoff speed, and odometry rate (SystemCore without a CANivore)."""

import math
import sys
import time
from dataclasses import replace

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.experiment import run_many, summarize, table  # noqa: E402
from alignsim.params import CameraParams, LoopParams, front_cameras  # noqa: E402
from alignsim.studies.tag.geotracker import GeoTracker  # noqa: E402
from alignsim.world import Errors, Scenario, Strategy, TrackerSpec, default_tracker  # noqa: E402

N = int(sys.argv[1]) if len(sys.argv) > 1 else 60
P = int(sys.argv[2]) if len(sys.argv) > 2 else 2
WHICH = sys.argv[3] if len(sys.argv) > 3 else "all"

GEO = Strategy(tracker=TrackerSpec(GeoTracker, needs_cams=True, resolve_flip=False))
KF = Strategy(tracker=default_tracker(frame="odom", max_amb=1.0))
STRATS = {"odom KF": KF, "geo": GEO}


def cams_layout(kind):
    if kind == "2 corners, toe-in 10deg (default)":
        return front_cameras()
    if kind == "2 corners, toe-in 25deg":
        return (CameraParams(name="ll-left", mount=(0.25, 0.20, 0.30, 0, math.radians(-15), math.radians(-25))),
                CameraParams(name="ll-right", mount=(0.25, -0.20, 0.30, 0, math.radians(-15), math.radians(25))))
    if kind == "2 corners, parallel":
        return (CameraParams(name="ll-left", mount=(0.25, 0.20, 0.30, 0, math.radians(-15), 0.0)),
                CameraParams(name="ll-right", mount=(0.25, -0.20, 0.30, 0, math.radians(-15), 0.0)))
    if kind == "1 center camera":
        return (CameraParams(name="ll-center", mount=(0.25, 0.0, 0.30, 0, math.radians(-15), 0.0)),)
    if kind == "1 corner camera":
        return (front_cameras()[0],)
    raise ValueError(kind)


SWEEPS = {
    "camera layout": {k: Scenario(cameras=cams_layout(k)) for k in (
        "2 corners, toe-in 10deg (default)", "2 corners, toe-in 25deg", "2 corners, parallel",
        "1 center camera", "1 corner camera")},
    "clock bias (timestamp error)": {f"{int(b * 1000)} ms @ {v} m/s": Scenario(
        errors=Errors(clock_bias=0.0), start_speed=v,
        cameras=tuple(replace(c, clock_bias=b) for c in front_cameras()))
        for v in (1.5, 3.0) for b in (0.0, 0.020, 0.050, 0.100)},
    "frame rate": {f"{f} fps": Scenario(cameras=tuple(replace(c, fps=f) for c in front_cameras()))
                   for f in (20, 30, 60, 120)},
    "pixel noise": {f"{px} px": Scenario(cameras=tuple(replace(c, pixel_noise=px) for c in front_cameras()))
                    for px in (0.2, 0.35, 0.7, 1.5)},
    "handoff speed": {f"{v} m/s": Scenario(start_speed=v) for v in (0.0, 1.5, 3.0)},
    "odometry rate": {"250 Hz (CANivore)": Scenario(), "100 Hz (SystemCore CAN 2.0)":
                      Scenario(loop=LoopParams(odom_period=0.010))},
}

if __name__ == "__main__":
    for sweep, scens in SWEEPS.items():
        if WHICH != "all" and WHICH not in sweep:
            continue
        out = []
        for cond, sc in scens.items():
            for sl, st in STRATS.items():
                t0 = time.time()
                # the clock-bias sweep sets the bias directly, so zero the random part
                out.append((f"{cond} / {sl}", summarize(run_many(sc, st, n=N, processes=P))))
                print(f"  {sweep}: {cond}/{sl} {time.time() - t0:.0f}s", flush=True)
        print(table(out, f"{sweep}, n={N}"), flush=True)
