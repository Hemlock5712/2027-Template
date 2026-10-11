"""Final-approach helpers: the default World with the odom-frame tracker, plus a goal-jump tracker.

JumpTracker models a late refinement: until the robot first gets within `trigger` of the goal, the
goal it reports is off by `jump` (1-5 cm, random direction, fixed per seed); then it snaps to the
tracker's own estimate. So success still measures whether the law recovers from a step in the goal.
"""

import math

import numpy as np

from alignsim.experiment import summarize
from alignsim.trackers import FrameTracker
from alignsim.world import Scenario, Strategy, TrackerSpec, World, default_tracker


class JumpTracker(FrameTracker):
    def __init__(self, tag_id, goal_in_tag, jump=(0.01, 0.05), trigger=0.05, seed=0, **kw):
        super().__init__(tag_id, goal_in_tag, **kw)
        rng = np.random.default_rng(seed + 7777)
        m, a = rng.uniform(*jump), rng.uniform(-math.pi, math.pi)
        self.offset = (m * math.cos(a), m * math.sin(a))
        self.trigger = trigger
        self.jumped = False
        self.t_jump = None

    def goal_field(self, snap, est, field):
        g = super().goal_field(snap, est, field)
        if g is None:
            return None
        if not self.jumped:
            gx, gy = g[0] + self.offset[0], g[1] + self.offset[1]
            p = snap["pose"]
            if math.hypot(gx - p[0], gy - p[1]) < self.trigger:
                self.jumped = True
                self.t_jump = snap["t"]
            else:
                return (gx, gy, g[2])
        return g


def run_approach(controller, n, seed0=0, sc=None, jump=None, trigger=0.05, **st_kw):
    """Default final approach (odom-frame tracker) with `controller`; optional goal jump."""
    sc = sc or Scenario()
    rows = []
    for i in range(n):
        seed = seed0 + i
        if jump is None:
            tr = default_tracker(frame="odom")
        else:
            tr = TrackerSpec(JumpTracker, frame="odom", jump=jump, trigger=trigger, seed=seed)
        st = Strategy(controller=controller, tracker=tr, **st_kw)
        rows.append(World(sc, seed).run(st))
    return rows


def approach_summary(rows):
    s = summarize(rows)
    t = [r["t_done"] for r in rows if r["done"]]
    s["t_done_p90"] = float(np.percentile(t, 90)) if t else float("nan")
    s["end_err_p95_cm"] = float(np.percentile([math.hypot(*r["err_end"][:2]) for r in rows], 95) * 100)
    return s


APPROACH_COLS = [
    ("success_%", "success %", "{:.0f}"), ("false_done_%", "false done %", "{:.0f}"),
    ("finished_%", "finished %", "{:.0f}"), ("t_done_med", "t done", "{:.2f}"),
    ("t_done_p90", "t done p90", "{:.2f}"), ("pos_done_p95_cm", "pos@done p95 cm", "{:.2f}"),
    ("yaw_done_p95_deg", "yaw@done p95°", "{:.2f}"), ("end_speed_cm_s", "jitter cm/s", "{:.2f}"),
    ("slip_ms", "slip ms", "{:.0f}"), ("peak_acc", "pk acc", "{:.1f}"),
]
