"""Global vision fusion and local (target-relative) pose trackers.

Every tracker returns the GOAL expressed in the fused field frame for this loop, so all
controllers can run on the fused pose; local trackers re-anchor their goal onto it:
    goal_field = fused_now (+) (odom_now^-1 (+) goal_odom)
which cancels any vision jump in the fused pose.
"""

import math

import numpy as np

from .geom import compose, interp_pose, inverse, relative, wrap


# ---------------------------------------------------------------------------- global fusion
class GlobalVision:
    """The template's Vision.java logic: MT1 for 2+ tags else MT2, xy std = base * d^2 / sqrt(n),
    heading untrusted. Options: sort frames oldest-first, only use the target tag."""

    def __init__(self, base_std=0.15, max_dist=4.0, sort=True, only_tag=None, max_spin=2 * math.pi):
        self.base_std, self.max_dist, self.sort = base_std, max_dist, sort
        self.only_tag, self.max_spin = only_tag, max_spin

    def apply(self, frames, est, field, omega):
        frames = sorted(frames, key=lambda f: f.t_capture_est) if self.sort else frames
        for fr in frames:
            fids = fr.fiducials
            if self.only_tag is not None:
                fids = [f for f in fids if f.id == self.only_tag]
            fids = [f for f in fids if f.distance <= self.max_dist]
            if not fids:
                continue
            n = len(fids)
            d = float(np.mean([f.distance for f in fids]))
            if n >= 2 and self.only_tag is None and fr.mt1 is not None:
                pose = fr.mt1
            elif abs(omega) <= self.max_spin:
                pose = _mt2(fids, field, fr.mt2[2] if fr.mt2 else est.estimate[2])
            else:
                continue
            std = self.base_std * d * d / math.sqrt(n)
            est.add_vision((pose[0], pose[1], est.sample_at(fr.t_capture_est)[2]), fr.t_capture_est,
                           (std, std, 1e6))


def _mt2(fids, field, heading):
    c, s = math.cos(heading), math.sin(heading)
    xs, ys, ws = [], [], []
    for f in fids:
        tag = field.by_id[f.id]
        px, py = f.tag_in_robot[0], f.tag_in_robot[1]
        xs.append(tag.map_pose[0] - (c * px - s * py))
        ys.append(tag.map_pose[1] - (s * px + c * py))
        ws.append(1.0 / max(0.05, f.distance) ** 2)
    w = np.array(ws)
    return (float(np.dot(w, xs) / w.sum()), float(np.dot(w, ys) / w.sum()), heading)


# ---------------------------------------------------------------------------- local trackers
class Tracker:
    """Base: holds the target id and the goal in the tag frame."""

    name = "base"

    def __init__(self, tag_id, goal_in_tag, max_dist=3.5, max_amb=0.3, max_age=0.25):
        self.tag_id, self.goal_in_tag = tag_id, goal_in_tag
        self.max_dist, self.max_amb, self.max_age = max_dist, max_amb, max_age
        self.reset()

    def reset(self):
        self.count = 0
        self.last_seen = -1e9

    def _sightings(self, t, frames):
        for fr in frames:
            if t - fr.t_capture_est > self.max_age:
                continue
            for f in fr.fiducials:
                if f.id == self.tag_id and f.ambiguity <= self.max_amb and f.distance <= self.max_dist:
                    yield fr, f

    def locked(self):
        return False

    def goal_field(self, snap, est, field):
        return None


class RawTargetSpace(Tracker):
    """Old template DriveToTag: newest robot-in-tag pose, treated as 'now' (no compensation)."""

    name = "raw"

    def reset(self):
        super().reset()
        self.robot_to_goal = None

    def update(self, t, snap, frames, est, field):
        for fr, f in self._sightings(t, frames):
            self.robot_to_goal = compose(f.tag_in_robot, self.goal_in_tag)
            self.count += 1
            self.last_seen = t

    def locked(self):
        return self.count >= 3

    def goal_field(self, snap, est, field):
        if not self.locked():
            return None
        return compose(snap["pose"], self.robot_to_goal)


class KalmanPose:
    """Per-axis scalar Kalman filter on (x, y, yaw) with a jump gate (TagTracker.java)."""

    def __init__(self, gate=0.25, jumps_to_restart=3, drift=(0.002, math.radians(0.1))):
        self.gate, self.jumps_to_restart, self.drift = gate, jumps_to_restart, drift
        self.est = None
        self.var = [0, 0, 0]
        self.jumps = 0
        self.n = 0

    def update(self, m, r):
        if self.est is None:
            self.est, self.var, self.n = m, list(r), 1
            return True
        dx, dy, dth = m[0] - self.est[0], m[1] - self.est[1], wrap(m[2] - self.est[2])
        if math.hypot(dx, dy) > self.gate:
            self.jumps += 1
            if self.jumps >= self.jumps_to_restart:
                self.est, self.var, self.n, self.jumps = m, list(r), 1, 0
                return True
            return False
        self.jumps = 0
        q = (self.drift[0] ** 2, self.drift[0] ** 2, self.drift[1] ** 2)
        k = []
        for i in range(3):
            self.var[i] += q[i]
            k.append(self.var[i] / (self.var[i] + r[i]))
            self.var[i] *= 1 - k[i]
        self.est = (self.est[0] + k[0] * dx, self.est[1] + k[1] * dy, wrap(self.est[2] + k[2] * dth))
        self.n += 1
        return True


def default_noise(d):
    """TagTracker.java noise model: xy = 0.005 + 0.01 d^2, yaw = 0.5 deg + 1.5 deg * d."""
    xy = 0.005 + 0.01 * d * d
    yaw = math.radians(0.5 + 1.5 * d)
    return (xy * xy, xy * xy, yaw * yaw)


class FrameTracker(Tracker):
    """Tag pose tracked in a chosen frame ('odom' = wheel-only, 'fused' = samplePoseAt), filtered.

    yaw: 'measured' (PnP), 'map' (field map yaw, gyro-referenced), or 'blend' (measured with a
    variance floor so it can't over-trust frames).
    filt: 'kalman' or 'ema' (Nav2 style).
    """

    def __init__(self, tag_id, goal_in_tag, frame="odom", yaw="measured", filt="kalman", ema=0.2,
                 noise=default_noise, lock_frames=3, yaw_floor=math.radians(0.7), **kw):
        self.frame, self.yaw_mode, self.filt, self.ema = frame, yaw, filt, ema
        self.noise, self.lock_frames, self.yaw_floor = noise, lock_frames, yaw_floor
        super().__init__(tag_id, goal_in_tag, **kw)
        self.name = f"{frame}-{yaw}-{filt}"

    def reset(self):
        super().reset()
        self.kf = KalmanPose()

    def _frame_pose(self, est, t):
        return est.odom_buffer.sample(t) if self.frame == "odom" else est.sample_at(t)

    def update(self, t, snap, frames, est, field):
        for fr, f in self._sightings(t, frames):
            robot = self._frame_pose(est, fr.t_capture_est)
            if robot is None:
                continue
            m = compose(robot, f.tag_in_robot)
            if self.yaw_mode == "map":
                # map yaw, expressed in this frame via the current heading offset (same gyro)
                offset = wrap(snap["pose"][2] - (snap["odom"][2] if self.frame == "odom" else snap["pose"][2]))
                m = (m[0], m[1], wrap(field.target.map_pose[2] - offset))
            r = list(self.noise(f.distance))
            if self.yaw_mode == "blend":
                r[2] += self.yaw_floor ** 2
            if self.filt == "ema":
                if self.kf.est is None:
                    self.kf.est, self.kf.n = m, 1
                else:
                    a = self.ema
                    e = self.kf.est
                    self.kf.est = (e[0] + a * (m[0] - e[0]), e[1] + a * (m[1] - e[1]), wrap(e[2] + a * wrap(m[2] - e[2])))
                    self.kf.n += 1
                ok = True
            else:
                ok = self.kf.update(m, r)
            if ok:
                self.last_seen = t
        self.count = self.kf.n

    def locked(self):
        return self.kf.est is not None and self.kf.n >= self.lock_frames

    def goal_field(self, snap, est, field):
        if not self.locked():
            return None
        goal = compose(self.kf.est, self.goal_in_tag)
        if self.frame == "odom":
            return compose(snap["pose"], relative(snap["odom"], goal))
        return goal

    def robot_in_tag(self, snap):
        if self.kf.est is None:
            return None
        here = snap["odom"] if self.frame == "odom" else snap["pose"]
        return relative(self.kf.est, here)


class TxTy6328(Tracker):
    """6328 2025 'txTy' pose: bearing from the tag's pixel centroid (tx/ty), range from the PnP
    translation, heading from the gyro at capture; tag yaw/position from the MAP. Stored per tag,
    propagated by the odometry twist since capture, dropped after 0.2 s. Optional distance blend
    with the global goal (24-36 in)."""

    name = "txty6328"

    def __init__(self, tag_id, goal_in_tag, cameras_believed, blend=(0.61, 0.91), max_age=0.2, **kw):
        self.cams = cameras_believed  # name -> believed mount 4x4
        self.blend = blend
        super().__init__(tag_id, goal_in_tag, max_age=max_age, **kw)

    def reset(self):
        super().reset()
        self.obs = None  # (robot field pose relative to map tag, odom pose at capture, t)

    def update(self, t, snap, frames, est, field):
        best = None
        for fr, f in self._sightings(t, frames):
            if best is None or f.distance < best[1].distance:
                best = (fr, f)
        if best is not None:
            fr, f = best
            mount = self.cams[fr.camera]
            tx, ty = math.radians(f.tx), math.radians(f.ty)
            v = np.array([1.0, -math.tan(tx), math.tan(ty)])
            v = v / np.linalg.norm(v) * f.distance
            p = mount[:3, :3] @ v + mount[:3, 3]  # tag center in robot frame (3D)
            heading = est.sample_at(fr.t_capture_est)[2]
            tag = field.target.map_pose
            c, s = math.cos(heading), math.sin(heading)
            robot = (tag[0] - (c * p[0] - s * p[1]), tag[1] - (s * p[0] + c * p[1]), heading)
            odom_then = est.odom_buffer.sample(fr.t_capture_est)
            if odom_then is not None:
                self.obs = (robot, odom_then, fr.t_capture_est)
                self.count += 1
                self.last_seen = t

    def locked(self):
        return self.obs is not None and self.count >= 1

    def goal_field(self, snap, est, field):
        if self.obs is None:
            return None
        robot_then, odom_then, tc = self.obs
        if snap["t"] - tc > 0.2 + 0.25:  # stale: let the caller fall back (blend to global)
            return None
        robot_now = compose(robot_then, relative(odom_then, snap["odom"]))
        map_goal = compose(field.target.map_pose, self.goal_in_tag)
        robot_to_goal = relative(robot_now, map_goal)
        local = compose(snap["pose"], robot_to_goal)
        if self.blend is None:
            return local
        d = math.hypot(robot_to_goal[0], robot_to_goal[1])
        w = min(1.0, max(0.0, (self.blend[1] - d) / (self.blend[1] - self.blend[0])))
        return interp_pose(map_goal, local, w)
