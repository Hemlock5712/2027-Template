"""Candidate improved tag tracker for the local (odometry-frame) pose.

Differences from FrameTracker (TagTracker.java):
  * full 3x3 Kalman filter on the tag pose (x, y, yaw) in the wheel-odometry frame
  * measurement noise from geometry: depth ~ d^2 along the camera ray, lateral ~ d across it,
    yaw ~ d plus a large term when the solve is ambiguous (head-on views)
  * IPPE flip resolved with the current estimate (or the map prior): pick the solution whose yaw
    is closer - the same trick MegaTag2 uses with the gyro
  * optional map prior: start from the field map's tag pose with an honest uncertainty, so the
    measured yaw only wins once it has earned it
  * separate gates for position and yaw
"""

import math

import numpy as np

from alignsim.geom import compose, relative, tf3_to_pose2, wrap
from alignsim.trackers import Tracker


class GeoTracker(Tracker):
    name = "geo"

    def __init__(self, tag_id, goal_in_tag, cams_believed, *,
                 depth_k=0.006, lat_k=0.003, floor=0.004,
                 yaw_k=math.radians(0.45), yaw_amb=math.radians(9.0), yaw_floor=math.radians(0.3),
                 q_xy=0.0015, q_yaw=math.radians(0.05),
                 prior=True, prior_xy=0.08, prior_yaw=math.radians(1.5),
                 resolve_flip=True, gate_xy=0.25, gate_yaw_sigma=4.0,
                 max_amb=1.0, lock_sigma=0.03, lock_frames=3, **kw):
        self.cams = cams_believed
        self.p = dict(depth_k=depth_k, lat_k=lat_k, floor=floor, yaw_k=yaw_k, yaw_amb=yaw_amb,
                      yaw_floor=yaw_floor, q_xy=q_xy, q_yaw=q_yaw, prior=prior, prior_xy=prior_xy,
                      prior_yaw=prior_yaw, resolve_flip=resolve_flip, gate_xy=gate_xy,
                      gate_yaw_sigma=gate_yaw_sigma, lock_sigma=lock_sigma, lock_frames=lock_frames)
        super().__init__(tag_id, goal_in_tag, max_amb=max_amb, **kw)

    def reset(self):
        super().reset()
        self.x = None  # (x, y, yaw) of the tag in the odom frame
        self.P = None
        self.n = 0
        self.jumps = 0

    # -------------------------------------------------------------- measurement model
    def _meas(self, fr, f, robot_then):
        mount = self.cams[fr.camera]
        best = compose(robot_then, f.tag_in_robot)
        alt = compose(robot_then, tf3_to_pose2(mount @ f.alt_in_cam))
        return best, alt

    def _R(self, f, robot_then, meas):
        p = self.p
        d = f.distance
        sd = p["floor"] + p["depth_k"] * d * d
        sl = p["floor"] + p["lat_k"] * d
        # ray direction (robot -> tag) in the odom frame
        ang = math.atan2(meas[1] - robot_then[1], meas[0] - robot_then[0])
        c, s = math.cos(ang), math.sin(ang)
        Rxy = np.array([[c, -s], [s, c]]) @ np.diag([sd * sd, sl * sl]) @ np.array([[c, s], [-s, c]])
        sy = p["yaw_floor"] + p["yaw_k"] * d + p["yaw_amb"] * f.ambiguity
        R = np.zeros((3, 3))
        R[:2, :2] = Rxy
        R[2, 2] = sy * sy
        return R

    def _init(self, m, R):
        self.x = np.array(m, dtype=float)
        self.P = R.copy()
        self.n = 1

    def update(self, t, snap, frames, est, field):
        p = self.p
        if self.x is None and p["prior"]:
            # map tag pose, moved into the odom frame through the current fused<->odom offset
            tag_map = field.target.map_pose
            prior = compose(snap["odom"], relative(snap["pose"], tag_map))
            self.x = np.array(prior, dtype=float)
            self.P = np.diag([p["prior_xy"] ** 2, p["prior_xy"] ** 2, p["prior_yaw"] ** 2])
            self.n = 0
        for fr, f in self._sightings(t, frames):
            robot_then = est.odom_buffer.sample(fr.t_capture_est)
            if robot_then is None:
                continue
            best, alt = self._meas(fr, f, robot_then)
            m = best
            if p["resolve_flip"] and self.x is not None:
                if abs(wrap(alt[2] - self.x[2])) < abs(wrap(best[2] - self.x[2])):
                    m = alt
            R = self._R(f, robot_then, m)
            if self.x is None:
                self._init(m, R)
                self.last_seen = t
                continue
            # predict: the tag doesn't move, but odometry drifts
            self.P = self.P + np.diag([p["q_xy"] ** 2, p["q_xy"] ** 2, p["q_yaw"] ** 2])
            innov = np.array([m[0] - self.x[0], m[1] - self.x[1], wrap(m[2] - self.x[2])])
            if math.hypot(innov[0], innov[1]) > p["gate_xy"] and self.n >= p["lock_frames"]:
                self.jumps += 1
                if self.jumps >= 3:
                    self._init(m, R)
                continue
            self.jumps = 0
            S = self.P + R
            if abs(innov[2]) > p["gate_yaw_sigma"] * math.sqrt(S[2, 2]):
                # position is fine, the orientation solve is not: update x/y only
                Pxy, Sxy = self.P[:2, :2], S[:2, :2]
                K = Pxy @ np.linalg.inv(Sxy)
                self.x[:2] = self.x[:2] + K @ innov[:2]
                self.P[:2, :2] = (np.eye(2) - K) @ Pxy
            else:
                K = self.P @ np.linalg.inv(S)
                self.x = self.x + K @ innov
                self.x[2] = wrap(self.x[2])
                self.P = (np.eye(3) - K) @ self.P
            self.n += 1
            self.last_seen = t
        self.count = self.n

    def locked(self):
        p = self.p
        return (self.x is not None and self.n >= p["lock_frames"]
                and math.sqrt(max(self.P[0, 0], self.P[1, 1])) < p["lock_sigma"])

    def goal_field(self, snap, est, field):
        if not self.locked():
            return None
        goal = compose(tuple(self.x), self.goal_in_tag)
        return compose(snap["pose"], relative(snap["odom"], goal))
