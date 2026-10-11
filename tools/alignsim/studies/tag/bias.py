"""Bias study: camera calibration error is what's left once timing and filtering are right.

Compares ways to remove it:
  * nothing (trust the camera's configured mount)
  * taught goal: average robot-in-tag at the true goal -> new goal_in_tag (TeachTagGoal OpMode)
  * per-camera correction: at the teach pose, measure each camera's error vs the nominal goal and
    correct every later frame from that camera (a robot-frame transform per camera)
  * recency weighting: bigger process noise so near frames (smallest bias) dominate
"""

import math
import multiprocessing as mp
import sys
import time
from dataclasses import replace

import numpy as np

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.experiment import summarize, table  # noqa: E402
from alignsim.geom import compose, inverse, relative  # noqa: E402
from alignsim.params import FieldParams  # noqa: E402
from alignsim.studies.tag.geotracker import GeoTracker  # noqa: E402
from alignsim.world import Errors, Scenario, Strategy, TrackerSpec, World  # noqa: E402


def _mean_pose(ps):
    a = np.array(ps)
    return (float(a[:, 0].mean()), float(a[:, 1].mean()),
            math.atan2(np.sin(a[:, 2]).mean(), np.cos(a[:, 2]).mean()))


class CalWorld(World):
    """teach() also computes a per-camera correction A_c with A_c (+) measured = expected."""

    mode = "goal"  # "goal" (taught goal) or "percam"

    def teach(self, nominal_goal):
        """Back-off teach (see World.teach) + a per-camera correction A_c so that
        A_c (+) measured = what the camera should see if the parked pose were the nominal goal."""
        taught = super().teach(nominal_goal)
        e = self.sc.errors
        rng = self.rng
        park = compose(self.true_goal, (rng.normal(0, e.teach_xy), rng.normal(0, e.teach_xy),
                                        rng.normal(0, e.teach_yaw)))
        tag_expected = compose(park, inverse(nominal_goal))  # odom frame, parked = nominal goal
        scale = 1.0 + rng.normal(0, 0.01)
        per = {c.cp.name: [] for c in self.cams}
        for k in range(120):
            back = 0.10 + 0.50 * k / 119
            true_pose = compose(park, (-back, 0.0, 0.0))
            odom_pose = compose(park, (-back * scale, 0.0, 0.0))
            for cam in self.cams:
                fr = cam.capture(k / 60.0, true_pose, (0, 0, 0), self.field, true_pose[2])
                for f in fr.fiducials:
                    if f.id == self.field.TARGET_ID:
                        expected = relative(odom_pose, tag_expected)
                        per[cam.cp.name].append(compose(expected, inverse(f.tag_in_robot)))
        for cam in self.cams:
            cam.correction = _mean_pose(per[cam.cp.name]) if per[cam.cp.name] else None
        return nominal_goal if self.mode == "percam" else taught


class CorrectedGeo(GeoTracker):
    """GeoTracker that applies each camera's taught correction to its frames."""

    def __init__(self, tag_id, goal_in_tag, cams_believed, corrections=None, **kw):
        self.corr = corrections or {}
        super().__init__(tag_id, goal_in_tag, cams_believed, **kw)

    def _meas(self, fr, f, robot_then):
        A = self.corr.get(fr.camera)
        tir = compose(A, f.tag_in_robot) if A is not None else f.tag_in_robot
        best = compose(robot_then, tir)
        return best, best


class CorrSpec(TrackerSpec):
    def __call__(self, tag_id, goal_in_tag, scenario, cams):
        corr = {c.cp.name: getattr(c, "correction", None) for c in cams}
        return CorrectedGeo(tag_id, goal_in_tag, {c.cp.name: c.mount_believed for c in cams},
                            corrections=corr, **self.kw)


def _run(args):
    sc, st, seed, mode = args
    w = CalWorld(sc, seed)
    w.mode = mode
    return w.run(st)


def run_many(sc, st, n, mode="goal", procs=2):
    with mp.get_context("fork").Pool(procs) as pool:
        return pool.map(_run, [(sc, st, s, mode) for s in range(n)], chunksize=4)


HARD = Scenario(
    errors=Errors(cam_yaw=math.radians(0.8), cam_pitch=math.radians(0.8), pnp_yaw=math.radians(1.0),
                  clock_bias=0.005, init_xy=0.08),
    field=FieldParams(element_yaw_error=math.radians(2.0), element_xy_error=0.04))

if __name__ == "__main__":
    N = int(sys.argv[1]) if len(sys.argv) > 1 else 80
    P = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    geo = dict(resolve_flip=False)
    variants = [
        ("geo, no calibration", Strategy(tracker=CorrSpec(GeoTracker, **geo)), "goal"),
        ("geo, taught goal", Strategy(tracker=CorrSpec(GeoTracker, **geo), teach=True), "goal"),
        ("geo, per-camera correction", Strategy(tracker=CorrSpec(GeoTracker, **geo), teach=True), "percam"),
        ("geo, recency (q x4), taught goal",
         Strategy(tracker=CorrSpec(GeoTracker, q_xy=0.006, q_yaw=math.radians(0.2), **geo), teach=True), "goal"),
        ("geo, recency (q x4), per-camera",
         Strategy(tracker=CorrSpec(GeoTracker, q_xy=0.006, q_yaw=math.radians(0.2), **geo), teach=True), "percam"),
    ]
    for sname, sc in (("default", Scenario()), ("hard", HARD)):
        out = []
        for label, st, mode in variants:
            t0 = time.time()
            out.append((label, summarize(run_many(sc, st, N, mode, P))))
            print(f"  {sname}/{label}: {time.time() - t0:.0f}s", flush=True)
        print(table(out, f"{sname}, n={N}"), flush=True)
