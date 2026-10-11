"""With per-camera calibration in place, does the geo filter still earn its extra code over the
first TagTracker.java (per-axis Kalman)? Plus geo ablations. All rows use the retuned controller."""

import copy
import math
import sys
from dataclasses import replace

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.experiment import summarize, table  # noqa: E402
from alignsim.geom import compose  # noqa: E402
from alignsim.studies.tag.bias import HARD, CorrSpec, run_many  # noqa: E402
from alignsim.studies.tag.final import RECOMMENDED, SHIP, VERY_HARD  # noqa: E402
from alignsim.studies.tag.geotracker import GeoTracker  # noqa: E402
from alignsim.trackers import FrameTracker  # noqa: E402
from alignsim.world import Scenario, Strategy, TrackerSpec  # noqa: E402

N = int(sys.argv[1]) if len(sys.argv) > 1 else 100
P = int(sys.argv[2]) if len(sys.argv) > 2 else 2


class CorrectedFrame(FrameTracker):
    """The first TagTracker.java (per-axis Kalman) with the per-camera correction applied, and
    optionally yaw noise that grows with ambiguity (yaw_amb rad per unit ambiguity)."""

    def __init__(self, tag_id, goal_in_tag, corrections=None, yaw_amb=0.0, **kw):
        self.corr = corrections or {}
        self.yaw_amb = yaw_amb
        self._amb = 0.0
        super().__init__(tag_id, goal_in_tag, noise=self._noise, **kw)

    def _noise(self, d):
        xy = 0.005 + 0.01 * d * d
        yaw = math.radians(0.5 + 1.5 * d) + self.yaw_amb * self._amb
        return (xy * xy, xy * xy, yaw * yaw)

    def _sightings(self, t, frames):
        for fr, f in super()._sightings(t, frames):
            self._amb = f.ambiguity
            A = self.corr.get(fr.camera)
            if A is not None:
                f = copy.copy(f)
                f.tag_in_robot = compose(A, f.tag_in_robot)
            yield fr, f


class CorrFrameSpec(TrackerSpec):
    def __call__(self, tag_id, goal_in_tag, scenario, cams):
        corr = {c.cp.name: getattr(c, "correction", None) for c in cams}
        return CorrectedFrame(tag_id, goal_in_tag, corrections=corr, **self.kw)


GEO = dict(resolve_flip=False)
ROWS = {
    "per-axis KF, keep all frames + per-camera cal":
        Strategy(controller=SHIP, tracker=CorrFrameSpec(FrameTracker, frame="odom", max_amb=1.0), teach=True),
    "per-axis KF, keep all frames, ambiguity yaw noise (9 deg) + cal":
        Strategy(controller=SHIP, tracker=CorrFrameSpec(FrameTracker, frame="odom", max_amb=1.0,
                                                        yaw_amb=math.radians(9.0)), teach=True),
    "SHIPPED: per-axis KF, drop ambiguity > 0.3 + per-camera cal":
        Strategy(controller=SHIP, tracker=CorrFrameSpec(FrameTracker, frame="odom", max_amb=0.3), teach=True),
    "geo + per-camera cal": RECOMMENDED,
    "geo, no map guess": Strategy(controller=SHIP, tracker=CorrSpec(GeoTracker, prior=False, **GEO), teach=True),
    "geo, ambiguity-blind yaw": Strategy(controller=SHIP, tracker=CorrSpec(GeoTracker, yaw_amb=0.0, **GEO),
                                         teach=True),
    "geo, no yaw gate": Strategy(controller=SHIP, tracker=CorrSpec(GeoTracker, gate_yaw_sigma=1e9, **GEO),
                                 teach=True),
}

ONLY = sys.argv[3] if len(sys.argv) > 3 else ""

if __name__ == "__main__":
    for wname, sc in (("default", Scenario()), ("hard", HARD), ("very hard", VERY_HARD)):
        rows = {k: v for k, v in ROWS.items() if ONLY in k}
        out = [(label, summarize(run_many(sc, st, N, "percam", P))) for label, st in rows.items()]
        print(table(out, f"Which filter, with per-camera calibration: {wname} world, n={N}"), flush=True)
