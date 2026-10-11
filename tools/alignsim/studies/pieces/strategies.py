"""Drive-to-piece strategies. Each one: reset(ctx, snap), step(t, snap, frames, est) -> field (odom)
frame velocity command (vx, vy, w), locked(), estimate() -> piece (x, y) in the odom frame or None.

  (a) JavaPiece      - PieceTracker.java + DriveToPiece.java, as written
  (b) Strafe2026     - driver forward + (-kP * tx) strafe (2026-AutoTesting)
      BearingPursuit - turn + drive along the bearing, speed from ty, no metric position
  (c) KalmanPiece    - constant-velocity Kalman filter in the odom frame, intercept + velocity feedforward
  (d) Hybrid         - position-based far away, edge-clipped boxes rejected, commit near the intake,
                       drive through the piece instead of stopping on it
      BearingMemory  - bearing pursuit in view + odom-frame KF memory once lost (the study's pick)
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

from alignsim.controllers import VectorBrake  # noqa: E402
from alignsim.geom import compose, wrap  # noqa: E402

D2R = math.pi / 180.0


# ---------------------------------------------------------------------------- geometry
def floor_point(mount, tx_deg, ty_deg, h):
    """Detection ray (1, -tan tx, tan ty), rotated by the camera mount, hits z = h: robot (x, y)."""
    R, c = mount[:3, :3], mount[:3, 3]
    a, b = -math.tan(tx_deg * D2R), math.tan(ty_deg * D2R)
    rx = R[0, 0] + R[0, 1] * a + R[0, 2] * b
    ry = R[1, 0] + R[1, 1] * a + R[1, 2] * b
    rz = R[2, 0] + R[2, 1] * a + R[2, 2] * b
    if rz > -1e-6:
        return None
    s = (h - c[2]) / rz
    return (c[0] + s * rx, c[1] + s * ry)


def fov_half(cp):
    return math.atan(cp.width / 2 / cp.fx), math.atan(cp.height / 2 / cp.fy)


def near_edge(cp, mount, tx, ty, pr, r, h, guard_deg=1.0):
    """True if the box around this detection may be clipped by the image edge."""
    hf, vf = fov_half(cp)
    d = math.hypot(math.hypot(pr[0] - mount[0, 3], pr[1] - mount[1, 3]), mount[2, 3] - h)
    rad = math.atan(r / max(d, 0.05)) + guard_deg * D2R
    return abs(tx * D2R) + rad > hf or abs(ty * D2R) + rad > vf


class Slew:
    """Vector acceleration limit on a field velocity command + angular slew (DriveToPose limits)."""

    def __init__(self, accel=6.0, waccel=20.0):
        self.accel, self.waccel = accel, waccel
        self.cmd = (0.0, 0.0, 0.0)

    def reset(self, vel):
        self.cmd = tuple(vel)

    def __call__(self, vx, vy, w, dt):
        dvx, dvy = vx - self.cmd[0], vy - self.cmd[1]
        dv = math.hypot(dvx, dvy)
        if dv > self.accel * dt:
            dvx, dvy = dvx * self.accel * dt / dv, dvy * self.accel * dt / dv
        dw = max(-self.waccel * dt, min(self.waccel * dt, w - self.cmd[2]))
        self.cmd = (self.cmd[0] + dvx, self.cmd[1] + dvy, self.cmd[2] + dw)
        return self.cmd


def heading_rate(err, wmax=6.0, wdecel=8.0, kp=5.0):
    return math.copysign(min(wmax, math.sqrt(2 * wdecel * abs(err)), kp * abs(err)), err)


def newest_per_camera(frames):
    best = {}
    for fr in frames:
        if fr.camera not in best or fr.t_arrive > best[fr.camera].t_arrive:
            best[fr.camera] = fr
    return list(best.values())


# ---------------------------------------------------------------------------- trackers
class EmaTracker:
    """PieceTracker.java: newest frame only, floor point placed with odometry at capture, nearest
    detection to the estimate, reset if > gate away, else interpolate by `smoothing`."""

    def __init__(self, smoothing=0.3, gate=0.4, lock=3, max_age=0.25, max_dist=4.0, newest_only=True,
                 edge_reject=False, at_capture=True):
        self.at_capture = at_capture  # False: place with the odometry pose NOW (no latency compensation)
        self.smoothing, self.gate, self.lock, self.max_age = smoothing, gate, lock, max_age
        self.max_dist, self.newest_only, self.edge_reject = max_dist, newest_only, edge_reject

    def reset(self, ctx):
        self.ctx = ctx
        self.p = None
        self.n = 0
        self.last_seen = -1e9
        self.edge_seen = False

    def update(self, t, frames, est):
        ctx = self.ctx
        frames = newest_per_camera(frames) if self.newest_only else sorted(frames, key=lambda f: f.t_capture_est)
        for fr in frames:
            if t - fr.t_capture_est > self.max_age:
                continue
            robot = est.odom_buffer.sample(fr.t_capture_est if self.at_capture else t)
            if robot is None:
                continue
            mount = ctx["mounts"][fr.camera]
            best, best_score = None, float("inf")
            for tx, ty in fr.pieces:
                pr = floor_point(mount, tx, ty, ctx["h"])
                if pr is None or math.hypot(*pr) > self.max_dist:
                    continue
                if self.edge_reject and near_edge(ctx["cps"][fr.camera], mount, tx, ty, pr, ctx["r"], ctx["h"]):
                    self.edge_seen = True
                    continue
                po = compose(robot, (pr[0], pr[1], 0.0))
                score = math.hypot(*pr) if self.p is None else math.hypot(po[0] - self.p[0], po[1] - self.p[1])
                if score < best_score:
                    best, best_score = (po[0], po[1]), score
            if best is None:
                continue
            if self.p is None or math.hypot(best[0] - self.p[0], best[1] - self.p[1]) > self.gate:
                self.p, self.n = best, 1
            else:
                a = self.smoothing
                self.p = (self.p[0] + a * (best[0] - self.p[0]), self.p[1] + a * (best[1] - self.p[1]))
                self.n += 1
            self.last_seen = t

    def locked(self):
        return self.p is not None and self.n >= self.lock

    def position(self, t):
        return self.p, (0.0, 0.0)


class KalmanTracker:
    """Constant-velocity (or constant-position, static=True) Kalman filter on the piece in the odom
    frame. Every frame is used in capture-time order; the measurement covariance is long along the
    line of sight: range sigma = (H^2 + d^2) / H * angle sigma."""

    def __init__(self, q_acc=1.0, v0_sigma=0.8, px_sigma=2.0, ang_floor_deg=0.25, pitch_sigma_deg=0.7,
                 h_sigma=0.015, gate_chi2=11.8, lock=3, max_age=0.25, max_dist=4.0, static=False,
                 edge_reject=False, reinit_after=4, q_static=0.02, v_gate=(0.15, 1.5)):
        self.q, self.v0, self.px, self.q_static = q_acc, v0_sigma, px_sigma, q_static
        self.ang_floor, self.pitch_sigma, self.h_sigma = ang_floor_deg * D2R, pitch_sigma_deg * D2R, h_sigma
        self.gate, self.lock, self.max_age, self.max_dist = gate_chi2, lock, max_age, max_dist
        self.static, self.edge_reject, self.reinit_after = static, edge_reject, reinit_after
        self.v_gate = v_gate  # velocity counts once |v| > max(v_gate[0], v_gate[1] * sigma_v)

    def reset(self, ctx):
        self.ctx = ctx
        self.x = None
        self.P = None
        self.tk = None
        self.n = 0
        self.rejects = 0
        self.last_seen = -1e9
        self.edge_seen = False

    def _meas(self, robot, mount, cp, pr):
        ctx = self.ctx
        po = compose(robot, (pr[0], pr[1], 0.0))
        dx, dy = pr[0] - mount[0, 3], pr[1] - mount[1, 3]
        d = math.hypot(dx, dy)
        H = max(0.1, mount[2, 3] - ctx["h"])
        a_px = self.px / cp.fx
        s_lat = d * math.hypot(a_px, self.ang_floor)
        s_rng = (H * H + d * d) / H * math.hypot(a_px, self.pitch_sigma) + d * self.h_sigma / H
        bearing = robot[2] + math.atan2(dy, dx)
        c, s = math.cos(bearing), math.sin(bearing)
        Rr = np.array([[c, -s], [s, c]])
        Rm = Rr @ np.diag([s_rng * s_rng, s_lat * s_lat]) @ Rr.T
        return np.array([po[0], po[1]]), Rm, math.hypot(*pr)

    def _predict(self, x, P, dt):
        if dt <= 0:
            return x, P
        if self.static:  # slow random walk so odometry slip can be absorbed
            P = P.copy()
            P[0, 0] += self.q_static * dt
            P[1, 1] += self.q_static * dt
            return x, P
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        q = self.q
        Q = np.zeros((4, 4))
        for i in (0, 1):
            Q[i, i] = q * dt ** 3 / 3
            Q[i, i + 2] = Q[i + 2, i] = q * dt ** 2 / 2
            Q[i + 2, i + 2] = q * dt
        return F @ x, F @ P @ F.T + Q

    def _init(self, z, Rm, tk):
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.zeros((4, 4))
        self.P[:2, :2] = Rm
        v = 1e-6 if self.static else self.v0 ** 2
        self.P[2, 2] = self.P[3, 3] = v
        self.tk = tk
        self.n = 1
        self.rejects = 0

    def update(self, t, frames, est):
        ctx = self.ctx
        for fr in sorted(frames, key=lambda f: f.t_capture_est):
            if t - fr.t_capture_est > self.max_age:
                continue
            robot = est.odom_buffer.sample(fr.t_capture_est)
            if robot is None:
                continue
            mount, cp = ctx["mounts"][fr.camera], ctx["cps"][fr.camera]
            cands = []
            for tx, ty in fr.pieces:
                pr = floor_point(mount, tx, ty, ctx["h"])
                if pr is None or math.hypot(*pr) > self.max_dist:
                    continue
                if self.edge_reject and near_edge(cp, mount, tx, ty, pr, ctx["r"], ctx["h"]):
                    self.edge_seen = True
                    continue
                cands.append(self._meas(robot, mount, cp, pr))
            if not cands:
                continue
            tk = fr.t_capture_est
            if self.x is None:
                z, Rm, _ = min(cands, key=lambda c: c[2])
                self._init(z, Rm, tk)
                self.last_seen = t
                continue
            x, P = self._predict(self.x, self.P, tk - self.tk)
            best = None
            for z, Rm, rng_ in cands:
                S = P[:2, :2] + Rm
                y = z - x[:2]
                m2 = float(y @ np.linalg.solve(S, y))
                if best is None or m2 < best[0]:
                    best = (m2, z, Rm, S, y)
            m2, z, Rm, S, y = best
            if m2 > self.gate:
                self.rejects += 1
                if self.rejects >= self.reinit_after:
                    near = min(cands, key=lambda c: float(np.hypot(*(c[0] - x[:2]))))
                    self._init(near[0], near[1], tk)
                    self.last_seen = t
                continue
            K = P[:, :2] @ np.linalg.inv(S)
            self.x = x + K @ y
            self.P = (np.eye(4) - K[:, :2] @ np.eye(2, 4)) @ P
            self.P = 0.5 * (self.P + self.P.T)
            self.tk = tk
            self.n += 1
            self.rejects = 0
            self.last_seen = t

    def locked(self):
        return self.x is not None and self.n >= self.lock

    def position(self, t):
        """Piece now and its velocity (velocity zeroed unless clearly nonzero)."""
        if self.x is None:
            return None, (0.0, 0.0)
        x, P = self._predict(self.x, self.P, t - self.tk)
        v = (float(x[2]), float(x[3]))
        sv = math.sqrt(max(0.0, 0.5 * (P[2, 2] + P[3, 3])))
        if math.hypot(*v) < max(self.v_gate[0], self.v_gate[1] * sv):
            v = (0.0, 0.0)
            x = self.x  # don't extrapolate on noise
        return (float(x[0]), float(x[1])), v


# ---------------------------------------------------------------------------- strategies
class DrivePiece:
    """Position-based approach (a, c, d): DriveToPose's VectorBrake to a goal `reach` behind the
    piece, facing it (facing re-aimed from the robot every loop, as DriveToPiece.goal()).

    reach    - robot center to piece at the goal (Java INTAKE_REACH 0.45 = piece at the bumper face)
    ff       - add the tracked piece velocity as feedforward (moving pieces)
    lead     - aim at where the piece will be after the expected time to get there
    commit   - once the piece is this close to the bumper (m), stop re-aiming the heading
    """

    name = "drive-piece"

    def __init__(self, tracker, reach=0.45, ff=False, lead=False, commit=None, vb=None):
        self.tracker, self.reach, self.ff, self.lead, self.commit = tracker, reach, ff, lead, commit
        self.vb_kw = vb or {}

    def reset(self, ctx, snap):
        self.ctx = ctx
        self.tracker.reset(ctx)
        self.vb = VectorBrake(**self.vb_kw)
        self.vb.reset(snap["pose"], snap["vel"])
        self.hold = snap["pose"]
        self.facing = None

    def locked(self):
        return self.tracker.locked()

    def estimate(self):
        p, _ = self.tracker.position(self._t) if hasattr(self, "_t") else (None, None)
        return p

    def step(self, t, snap, frames, est):
        self._t = t
        dt = self.ctx["dt"]
        self.tracker.update(t, frames, est)
        pose = snap["pose"]  # == odom frame here
        if not self.tracker.locked():
            return self.vb.step(pose, self.hold, snap["vel"], dt)
        p, v = self.tracker.position(t)
        if not self.ff:
            v = (0.0, 0.0)
        if self.lead and (v[0] or v[1]):
            d = math.hypot(p[0] - pose[0], p[1] - pose[1])
            tau = min(1.0, max(0.0, d - self.reach) / 2.0)
            p = (p[0] + v[0] * tau, p[1] + v[1] * tau)
        to = (p[0] - pose[0], p[1] - pose[1])
        d = math.hypot(*to)
        if d < 1e-3:
            return self.vb.step(pose, pose, snap["vel"], dt)
        facing = math.atan2(to[1], to[0])
        if self.commit is not None and d - self.ctx["mouth"] < self.commit:
            if self.facing is None:
                self.facing = facing
            facing = self.facing
        else:
            self.facing = None
        goal = (p[0] - self.reach * math.cos(facing), p[1] - self.reach * math.sin(facing), facing)
        if self.ff and (v[0] or v[1]):
            # chase in the piece's frame: controller sees the relative motion, feedforward adds v
            rel_vel = (snap["vel"][0] - v[0], snap["vel"][1] - v[1], snap["vel"][2])
            c = self.vb.step(pose, goal, rel_vel, dt)
            return (c[0] + v[0], c[1] + v[1], c[2])
        return self.vb.step(pose, goal, snap["vel"], dt)


def JavaPiece(**kw):
    """(a) PieceTracker.java + DriveToPiece.java as written."""
    tk = kw.pop("tracker_kw", {})
    return DrivePiece(EmaTracker(**tk), reach=kw.pop("reach", 0.45), **kw)


class BearingPursuit:
    """(b) IBVS-like: heading target = gyro yaw at capture - tx; drive along that bearing with a speed
    set by ty (far -> v_far, near -> v_near). When the piece is lost low in the image, keep driving
    the last bearing at v_near for `blind_time` (commit); lost high -> stop.
    comp=False uses the yaw now instead of at capture (no latency compensation)."""

    name = "bearing"

    def __init__(self, v_far=3.0, v_near=1.2, d_far=1.6, d_near=0.6, comp=True, blind_time=0.8,
                 lost_time=0.12, strafe_only=False):
        self.v_far, self.v_near, self.d_far, self.d_near = v_far, v_near, d_far, d_near
        self.comp, self.blind_time, self.lost_time = comp, blind_time, lost_time
        self.strafe_only = strafe_only

    def reset(self, ctx, snap):
        self.ctx = ctx
        self.slew = Slew()
        self.slew.reset(snap["vel"])
        self.bearing = None
        self.ty = None
        self.last_seen = -1e9
        self.n = 0
        cam = next(iter(ctx["cps"]))
        self.cam = cam
        cp, m = ctx["cps"][cam], ctx["mounts"][cam]
        pitch = -math.asin(max(-1.0, min(1.0, m[2, 0])))  # believed pitch (positive = down)
        H = m[2, 3] - ctx["h"]
        # ty thresholds from the believed mount: no metric position in the loop
        self.ty_far = (-math.atan(H / self.d_far) + pitch) / D2R
        self.ty_near = (-math.atan(H / self.d_near) + pitch) / D2R
        self.ty_low = -fov_half(cp)[1] / D2R + 12.0  # lost below this -> it went under the bumper

    def locked(self):
        return self.n >= 1

    def estimate(self):
        return None

    def step(self, t, snap, frames, est):
        dt = self.ctx["dt"]
        for fr in sorted(frames, key=lambda f: f.t_capture_est):
            if t - fr.t_capture_est > 0.25 or not fr.pieces:
                continue
            yaw = est.odom_buffer.sample(fr.t_capture_est)[2] if self.comp else snap["pose"][2]
            cands = [(wrap(yaw - tx * D2R), ty, tx) for tx, ty in fr.pieces]
            if self.bearing is None:
                pick = min(cands, key=lambda c: c[1])  # lowest in the image = nearest
            else:
                pick = min(cands, key=lambda c: abs(wrap(c[0] - self.bearing)))
            self.bearing, self.ty, self.tx = pick
            self.last_seen = t
            self.n += 1
        if self.bearing is None:
            return self.slew(0.0, 0.0, 0.0, dt)
        lost = t - self.last_seen
        if lost <= self.lost_time:
            u = (self.ty - self.ty_near) / (self.ty_far - self.ty_near)
            speed = self.v_near + (self.v_far - self.v_near) * max(0.0, min(1.0, u))
        elif self.ty < self.ty_low and lost < self.blind_time:
            speed = self.v_near
        else:
            speed = 0.0
        yaw = snap["pose"][2]
        w = heading_rate(wrap(self.bearing - yaw))
        return self.slew(speed * math.cos(self.bearing), speed * math.sin(self.bearing), w, dt)


class BearingMemory(BearingPursuit):
    """(d') Bearing pursuit while the piece is in view; a KF keeps the piece in the odom frame, and once
    the piece is lost the robot drives at v_near toward the KF's predicted position (for up to
    `memory_time`) instead of the last bearing."""

    name = "bearing+memory"

    def __init__(self, memory_time=1.0, tracker_kw=None, **kw):
        super().__init__(**kw)
        self.memory_time = memory_time
        self.tk = {"q_acc": 0.3, "max_dist": 8.0, **(tracker_kw or {})}

    def reset(self, ctx, snap):
        super().reset(ctx, snap)
        self.tracker = KalmanTracker(**self.tk)
        self.tracker.reset(ctx)

    def estimate(self):
        return self.tracker.position(self._t)[0] if hasattr(self, "_t") else None

    def step(self, t, snap, frames, est):
        self._t = t
        self.tracker.update(t, frames, est)
        lost = t - self.last_seen
        if self.bearing is None or lost <= self.lost_time or not self.tracker.locked() or lost > self.memory_time:
            return super().step(t, snap, frames, est)
        for fr in frames:  # a new sighting hands control back to the bearing servo next loop
            if fr.pieces and t - fr.t_capture_est <= 0.25:
                return super().step(t, snap, frames, est)
        p, _ = self.tracker.position(t)
        pose = snap["pose"]
        b = math.atan2(p[1] - pose[1], p[0] - pose[0])
        w = heading_rate(wrap(b - pose[2]))
        return self.slew(self.v_near * math.cos(b), self.v_near * math.sin(b), w, self.ctx["dt"])


class Strafe2026:
    """(b0) 2026-AutoTesting: the driver drives forward, code adds -kP * tx strafe. Heading held."""

    name = "strafe2026"

    def __init__(self, v_drive=2.0, kp=0.04, vmax_strafe=1.5, blind_time=1.0):
        self.v_drive, self.kp, self.vmax_strafe, self.blind_time = v_drive, kp, vmax_strafe, blind_time

    def reset(self, ctx, snap):
        self.ctx = ctx
        self.slew = Slew()
        self.slew.reset(snap["vel"])
        self.heading = snap["pose"][2]
        self.tx = None
        self.last_seen = -1e9

    def locked(self):
        return self.tx is not None

    def estimate(self):
        return None

    def step(self, t, snap, frames, est):
        dt = self.ctx["dt"]
        for fr in newest_per_camera(frames):
            if fr.pieces:
                if self.tx is None:
                    self.tx = min(fr.pieces, key=lambda p: p[1])[0]
                else:
                    self.tx = min(fr.pieces, key=lambda p: abs(p[0] - self.tx))[0]
                self.last_seen = t
        yaw = snap["pose"][2]
        w = heading_rate(wrap(self.heading - yaw))
        if self.tx is None:
            return self.slew(0.0, 0.0, w, dt)
        lost = t - self.last_seen
        fwd = self.v_drive if lost < self.blind_time else 0.0
        side = 0.0 if lost > 0.1 else max(-self.vmax_strafe, min(self.vmax_strafe, -self.kp * self.tx))
        c, s = math.cos(yaw), math.sin(yaw)
        return self.slew(c * fwd - s * side, s * fwd + c * side, w, dt)


def KalmanPiece(**kw):
    """(c) Constant-velocity KF in the odom frame, lead + velocity feedforward, stops on the piece."""
    tk = {"q_acc": 0.3, **kw.pop("tracker_kw", {})}
    return DrivePiece(KalmanTracker(**tk), reach=kw.pop("reach", 0.45), ff=True, lead=True, **kw)


def Hybrid(**kw):
    """(d) Position-based far away (CV KF, no 4 m gate), edge-clipped boxes rejected so the estimate
    commits to the last clean odom-frame fix and keeps predicting under the bumper, goal past the
    mouth so the robot drives through the piece."""
    tk = {"q_acc": 0.3, "max_dist": 8.0, "edge_reject": True, **kw.pop("tracker_kw", {})}
    static = tk.get("static", False)
    return DrivePiece(KalmanTracker(**tk), reach=kw.pop("reach", 0.35), ff=not static, lead=not static,
                      commit=kw.pop("commit", None), **kw)
