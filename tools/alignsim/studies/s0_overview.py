"""First look: every local/global strategy with the VectorBrake controller, default scenario."""

import sys
import time

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.experiment import run_many, summarize, table  # noqa: E402
from alignsim.trackers import FrameTracker, GlobalVision, RawTargetSpace, TxTy6328  # noqa: E402
from alignsim.world import Scenario, Strategy, TrackerSpec, default_tracker  # noqa: E402

txty = TrackerSpec(TxTy6328, needs_cams=True)
raw = TrackerSpec(RawTargetSpace)


STRATEGIES = {
    "global (all tags)": Strategy(approach="global"),
    "global (target tag only)": Strategy(approach="global", vision=GlobalVision(only_tag=1)),
    "raw tag pose (old DriveToTag)": Strategy(tracker=raw),
    "fused history (samplePoseAt)": Strategy(tracker=default_tracker(frame="fused")),
    "odom frame (TagTracker)": Strategy(tracker=default_tracker(frame="odom")),
    "odom frame, map yaw": Strategy(tracker=default_tracker(frame="odom", yaw="map")),
    "odom frame, amb<=1 (no amb filter)": Strategy(tracker=default_tracker(frame="odom", max_amb=1.0)),
    "6328 txTy (map yaw, blend)": Strategy(tracker=txty),
    "odom frame + taught goal": Strategy(tracker=default_tracker(frame="odom"), teach=True),
}

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    sc = Scenario()
    out = []
    for label, st in STRATEGIES.items():
        t0 = time.time()
        rows = run_many(sc, st, n=n)
        out.append((label, summarize(rows)))
        print(f"{label}: {time.time() - t0:.0f}s", flush=True)
    print(table(out, f"Overview, default scenario, VectorBrake, n={n}"))
