"""Approach shaping + the headline comparison: the recommended pipeline vs every baseline,
across default / hard / very hard worlds. Uses CalWorld so per-camera calibration is available."""

import math
import sys
import time
from dataclasses import replace

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.experiment import summarize, table  # noqa: E402
from alignsim.params import FieldParams, front_cameras  # noqa: E402
from alignsim.studies.tag.bias import HARD, CorrSpec, run_many  # noqa: E402
from alignsim.studies.tag.geotracker import GeoTracker  # noqa: E402
from alignsim.trackers import GlobalVision, RawTargetSpace, TxTy6328  # noqa: E402
from alignsim.world import Errors, Scenario, Strategy, TrackerSpec, default_tracker  # noqa: E402
from alignsim.studies.controllers.variants import VectorBrakeX  # noqa: E402
from functools import partial  # noqa: E402

# Controller study's ship parameters (docs/alignment/studies/controllers.md)
SHIP = partial(VectorBrakeX, vmax=4.0, decel=4.0, accel=5.0, kp=6.0, react=0.05, wmax=6.0, wdecel=6.0,
               waccel=8.0, kp_w=5.0, wreact=0.05, deadband=0.003, wdeadband=math.radians(0.3))

N = int(sys.argv[1]) if len(sys.argv) > 1 else 100
P = int(sys.argv[2]) if len(sys.argv) > 2 else 2
WHICH = sys.argv[3] if len(sys.argv) > 3 else "all"

VERY_HARD = Scenario(
    errors=Errors(cam_yaw=math.radians(1.5), cam_pitch=math.radians(1.0), pnp_yaw=math.radians(1.5),
                  clock_bias=0.015, init_xy=0.15, fx=0.01, center_px=4.0),
    field=FieldParams(element_yaw_error=math.radians(4.0), element_xy_error=0.08),
    start_speed=2.5,
    cameras=tuple(replace(c, pixel_noise=0.6) for c in front_cameras()))

REC = dict(resolve_flip=False)
RECOMMENDED = Strategy(controller=SHIP, tracker=CorrSpec(GeoTracker, **REC), teach=True)

APPROACH = {
    "handoff (direct)": RECOMMENDED,
    "staged 0.6 m (Nav2)": replace(RECOMMENDED, approach="staged"),
    "shaped (6328 sliding target)": replace(RECOMMENDED, approach="shaped"),
    "slow zone 0.4 m @ 0.6 m/s": replace(RECOMMENDED, slow_zone=(0.4, 0.6)),
}

HEADLINE = {
    "global (all tags)": (Strategy(approach="global"), "goal"),
    "global (target tag only, 581/254)": (Strategy(approach="global", vision=GlobalVision(only_tag=1)), "goal"),
    "raw tag pose (old DriveToTag)": (Strategy(tracker=TrackerSpec(RawTargetSpace)), "goal"),
    "6328 txTy (map yaw)": (Strategy(tracker=TrackerSpec(TxTy6328, needs_cams=True)), "goal"),
    "fused history (2026 samplePoseAt style)": (Strategy(tracker=default_tracker(frame="fused", max_amb=1.0)), "goal"),
    "odom KF (current TagTracker.java)": (Strategy(tracker=default_tracker(frame="odom", max_amb=1.0)), "goal"),
    "geo KF + per-camera cal, old controller tune": (Strategy(tracker=CorrSpec(GeoTracker, **REC), teach=True), "percam"),
    "RECOMMENDED: + retuned VectorBrake": (RECOMMENDED, "percam"),
}

GATES = {
    "2 cm / 1.5 deg done gate (default)": RECOMMENDED,
    "1 cm / 0.75 deg done gate": replace(RECOMMENDED, pos_tol=0.01, yaw_tol=math.radians(0.75)),
}

if __name__ == "__main__":
    if WHICH in ("all", "approach"):
        out = []
        for label, st in APPROACH.items():
            t0 = time.time()
            out.append((label, summarize(run_many(HARD, st, N, "percam", P))))
            print(f"  approach/{label}: {time.time() - t0:.0f}s", flush=True)
        print(table(out, f"Approach shaping, hard world, recommended tracker, n={N}"), flush=True)
    if WHICH in ("all", "headline"):
        for wname, sc in (("default", Scenario()), ("hard", HARD), ("very hard", VERY_HARD)):
            out = []
            for label, (st, mode) in HEADLINE.items():
                t0 = time.time()
                out.append((label, summarize(run_many(sc, st, N, mode, P))))
                print(f"  {wname}/{label}: {time.time() - t0:.0f}s", flush=True)
            print(table(out, f"Headline: {wname} world, n={N}"), flush=True)
    if WHICH in ("all", "gate"):
        for wname, sc in (("default", Scenario()), ("hard", HARD), ("very hard", VERY_HARD)):
            out = [(label, summarize(run_many(sc, st, N, "percam", P))) for label, st in GATES.items()]
            print(table(out, f"Done gate: {wname} world, n={N}"), flush=True)
