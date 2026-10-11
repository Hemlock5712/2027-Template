"""Quick A/B of tracker designs on a default and a hard scenario."""
import math
import sys
import time

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.experiment import run_many, summarize, table  # noqa: E402
from alignsim.studies.tag.geotracker import GeoTracker  # noqa: E402
from alignsim.world import Errors, Scenario, Strategy, TrackerSpec, default_tracker  # noqa: E402
from alignsim.params import FieldParams  # noqa: E402

N = int(sys.argv[1]) if len(sys.argv) > 1 else 80
PROCS = int(sys.argv[2]) if len(sys.argv) > 2 else 1

hard_err = Errors(cam_yaw=math.radians(0.8), cam_pitch=math.radians(0.8), pnp_yaw=math.radians(1.0),
                  clock_bias=0.005, init_xy=0.08)
SCEN = {
    "default": Scenario(),
    "hard (elem 2deg, cam cal worse)": Scenario(errors=hard_err,
                                                field=FieldParams(element_yaw_error=math.radians(2.0),
                                                                  element_xy_error=0.04)),
}
STRATS = {
    "odom KF (TagTracker.java)": Strategy(tracker=default_tracker(frame="odom", max_amb=1.0)),
    "geo: full model": Strategy(tracker=TrackerSpec(GeoTracker, needs_cams=True)),
    "geo: no prior": Strategy(tracker=TrackerSpec(GeoTracker, needs_cams=True, prior=False)),
    "geo: no flip fix": Strategy(tracker=TrackerSpec(GeoTracker, needs_cams=True, resolve_flip=False)),
    "geo: + taught": Strategy(tracker=TrackerSpec(GeoTracker, needs_cams=True), teach=True),
}
for sname, sc in SCEN.items():
    out = []
    for label, st in STRATS.items():
        t0 = time.time()
        out.append((label, summarize(run_many(sc, st, n=N, processes=PROCS))))
        print(f"  {sname} / {label}: {time.time() - t0:.0f}s", flush=True)
    print(table(out, f"{sname}, n={N}"), flush=True)
