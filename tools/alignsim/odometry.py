"""Swerve odometry (WPILib forward kinematics + gyro) and a faithful port of WPILib's
PoseEstimator (latency-compensated vision with an odometry buffer; adding an older measurement
discards stored newer ones - "Step 7" in PoseEstimator.java)."""

import bisect
import math

import numpy as np

from .geom import compose, inverse, relative, twist_exp, twist_log, wrap


class TimeBuffer:
    """WPILib TimeInterpolatableBuffer<Pose2d>."""

    def __init__(self, history=1.5):
        self.history = history
        self.t = []
        self.p = []

    def add(self, t, pose):
        if self.t and t <= self.t[-1]:
            i = bisect.bisect_left(self.t, t)
            if i < len(self.t) and self.t[i] == t:
                self.p[i] = pose
                return
            self.t.insert(i, t)
            self.p.insert(i, pose)
        else:
            self.t.append(t)
            self.p.append(pose)
        cutoff = self.t[-1] - self.history
        k = bisect.bisect_left(self.t, cutoff)
        if k > 0:
            del self.t[:k]
            del self.p[:k]

    def sample(self, t):
        if not self.t:
            return None
        i = bisect.bisect_left(self.t, t)
        if i < len(self.t) and self.t[i] == t:
            return self.p[i]
        if i == 0:
            return self.p[0]
        if i >= len(self.t):
            return self.p[-1]
        t0, t1 = self.t[i - 1], self.t[i]
        a, b = self.p[i - 1], self.p[i]
        u = (t - t0) / (t1 - t0)
        # WPILib Pose2d.interpolate: twist-based
        tw = twist_log(relative(a, b))
        return compose(a, twist_exp(tw[0] * u, tw[1] * u, tw[2] * u))


class SwerveOdometry:
    """WPILib SwerveDriveOdometry: least-squares module twist, heading from the gyro."""

    def __init__(self, module_positions, gyro, modules, pose=(0.0, 0.0, 0.0)):
        rows = []
        for mx, my in module_positions:
            rows.append([1.0, 0.0, -my])
            rows.append([0.0, 1.0, mx])
        self.pinv = np.linalg.pinv(np.array(rows))
        self.prev_dist = [m[0] for m in modules]
        self.gyro_offset = wrap(pose[2] - gyro)
        self.prev_angle = pose[2]
        self.pose = pose

    def update(self, gyro, modules):
        d = []
        for i, (dist, ang) in enumerate(modules):
            delta = dist - self.prev_dist[i]
            d.append(delta * math.cos(ang))
            d.append(delta * math.sin(ang))
            self.prev_dist[i] = dist
        tw = self.pinv @ np.array(d)
        angle = wrap(gyro + self.gyro_offset)
        dth = wrap(angle - self.prev_angle)
        new = compose(self.pose, twist_exp(float(tw[0]), float(tw[1]), dth))
        self.pose = (new[0], new[1], angle)
        self.prev_angle = angle
        return self.pose

    def reset(self, gyro, pose):
        self.gyro_offset = wrap(pose[2] - gyro)
        self.prev_angle = pose[2]
        self.pose = pose


class PoseEstimator:
    """Port of WPILib PoseEstimator (2027 alpha). Odometry pose = the wheel-only 'odom' frame."""

    BUFFER = 1.5

    def __init__(self, odometry: SwerveOdometry, state_std=(0.1, 0.1, 0.1)):
        self.odometry = odometry
        self.q = [s * s for s in state_std]
        self.odom_buffer = TimeBuffer(self.BUFFER)
        self.vision_t = []  # sorted timestamps
        self.vision = []  # (vision_pose, odometry_pose)
        self.estimate = odometry.pose

    @staticmethod
    def _compensate(update, pose):
        vision_pose, odom_pose = update
        return compose(vision_pose, relative(odom_pose, pose))

    def update(self, t, gyro, modules):
        odom = self.odometry.update(gyro, modules)
        self.odom_buffer.add(t, odom)
        if not self.vision:
            self.estimate = odom
        else:
            self.estimate = self._compensate(self.vision[-1], odom)
        return self.estimate

    def sample_at(self, t):
        if not self.odom_buffer.t:
            return None
        oldest = self.odom_buffer.t[0]
        newest = self.odom_buffer.t[-1]
        t = min(max(t, oldest), newest)
        if not self.vision or t < self.vision_t[0]:
            return self.odom_buffer.sample(t)
        i = bisect.bisect_right(self.vision_t, t) - 1
        return self._compensate(self.vision[i], self.odom_buffer.sample(t))

    def _clean_vision(self):
        if not self.odom_buffer.t:
            return
        oldest = self.odom_buffer.t[0]
        if not self.vision_t or oldest < self.vision_t[0]:
            return
        i = bisect.bisect_right(self.vision_t, oldest) - 1
        if i > 0:
            del self.vision_t[:i]
            del self.vision[:i]

    def add_vision(self, pose, t, std):
        buf = self.odom_buffer
        if not buf.t or buf.t[-1] - self.BUFFER > t:
            return False
        self._clean_vision()
        odom_sample = buf.sample(t)
        vision_sample = self.sample_at(t)
        if odom_sample is None or vision_sample is None:
            return False
        k = []
        for i in range(3):
            q, r = self.q[i], std[i] * std[i]
            k.append(q / (q + math.sqrt(q * r)) if q > 0 else 0.0)
        tw = twist_log(relative(vision_sample, pose))
        scaled = twist_exp(k[0] * tw[0], k[1] * tw[1], k[2] * tw[2])
        update = (compose(vision_sample, scaled), odom_sample)
        i = bisect.bisect_left(self.vision_t, t)
        # Step 7: remove later vision measurements
        del self.vision_t[i:]
        del self.vision[i:]
        self.vision_t.append(t)
        self.vision.append(update)
        self.estimate = self._compensate(update, self.odometry.pose)
        return True
