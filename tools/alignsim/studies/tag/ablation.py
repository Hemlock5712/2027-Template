"""Error budget: start from a perfect world and turn on ONE error source at a time.
Shows which errors each strategy is sensitive to."""

import math
import sys
import time
from dataclasses import replace

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.experiment import run_many, summarize  # noqa: E402
from alignsim.params import FieldParams  # noqa: E402
from alignsim.studies.tag.geotracker import GeoTracker  # noqa: E402
from alignsim.trackers import GlobalVision  # noqa: E402
from alignsim.world import Errors, Scenario, Strategy, TrackerSpec, default_tracker  # noqa: E402

N = int(sys.argv[1]) if len(sys.argv) > 1 else 60
PROCS = int(sys.argv[2]) if len(sys.argv) > 2 else 2

ZERO = Errors(init_xy=0, init_yaw=0, wheel_radius=0, gyro_scale=0, cam_xy=0, cam_z=0, cam_yaw=0,
              cam_pitch=0, cam_roll=0, fx=0, center_px=0, clock_bias=0, pnp_yaw=0)
FZERO = FieldParams(element_xy_error=0, element_yaw_error=0, tag_mount_xy_error=0, tag_mount_yaw_error=0,
                    other_tag_xy_error=0, other_tag_yaw_error=0)

SOURCES = {
    "none": ({}, {}),
    "element 4cm/2deg": ({}, dict(element_xy_error=0.04, element_yaw_error=math.radians(2))),
    "cam mount yaw 0.8deg": (dict(cam_yaw=math.radians(0.8)), {}),
    "cam mount pitch 0.8deg": (dict(cam_pitch=math.radians(0.8)), {}),
    "cam mount xy 5mm": (dict(cam_xy=0.005), {}),
    "focal length 1%": (dict(fx=0.01), {}),
    "principal pt 4px": (dict(center_px=4.0), {}),
    "pnp yaw bias 1deg": (dict(pnp_yaw=math.radians(1.0)), {}),
    "clock bias 10ms": (dict(clock_bias=0.010), {}),
    "global pose 8cm off": (dict(init_xy=0.08, init_yaw=math.radians(1.0)), {}),
    "wheel radius 2%": (dict(wheel_radius=0.02), {}),
}

STRATS = {
    "global tag-only": Strategy(approach="global", vision=GlobalVision(only_tag=1)),
    "fused hist": Strategy(tracker=default_tracker(frame="fused", max_amb=1.0)),
    "odom KF": Strategy(tracker=default_tracker(frame="odom", max_amb=1.0)),
    "odom KF taught": Strategy(tracker=default_tracker(frame="odom", max_amb=1.0), teach=True),
    "geo": Strategy(tracker=TrackerSpec(GeoTracker, needs_cams=True)),
    "geo taught": Strategy(tracker=TrackerSpec(GeoTracker, needs_cams=True), teach=True),
}

if __name__ == "__main__":
    rows = {}
    for src, (emods, fmods) in SOURCES.items():
        sc = Scenario(errors=replace(ZERO, **emods), field=replace(FZERO, **fmods))
        for label, st in STRATS.items():
            t0 = time.time()
            s = summarize(run_many(sc, st, n=N, processes=PROCS))
            rows[(src, label)] = s
            print(f"  {src} / {label}: {time.time() - t0:.0f}s", flush=True)
    labels = list(STRATS)
    print(f"\n**Success % (finished AND truly < 2 cm & < 1 deg), n={N}**\n")
    print("| error source | " + " | ".join(labels) + " |")
    print("|---|" + "---|" * len(labels))
    for src in SOURCES:
        print(f"| {src} | " + " | ".join(f"{rows[(src, l)]['success_%']:.0f}" for l in labels) + " |")
    print(f"\n**95th-percentile position error at done (cm) / yaw (deg)**\n")
    print("| error source | " + " | ".join(labels) + " |")
    print("|---|" + "---|" * len(labels))
    for src in SOURCES:
        print(f"| {src} | " + " | ".join(
            f"{rows[(src, l)]['pos_done_p95_cm']:.1f} / {rows[(src, l)]['yaw_done_p95_deg']:.1f}" for l in labels) + " |")
