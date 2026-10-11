"""Skid / collision study: a World with scheduled pushes, skid detectors, and mitigation hooks.

SkidWorld.run is a copy of World.run (core, Oct 2026) with hooks marked "# HOOK". Keep it in sync
if world.py changes. Every extra random draw uses its own generator, so the core rng stream (and
therefore the camera noise, start pose, field errors) matches World for the same seed until the
trajectories diverge.
"""

import math
import sys
from collections import deque
from dataclasses import dataclass, field as dfield, replace

import numpy as np

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.geom import compose, relative, rot, twist_exp, wrap  # noqa: E402
from alignsim.odometry import PoseEstimator, SwerveOdometry  # noqa: E402
from alignsim.plant import SwervePlant  # noqa: E402
from alignsim.trackers import FrameTracker, GlobalVision, KalmanPose  # noqa: E402
from alignsim.world import World, _shape_6328  # noqa: E402

G = 9.81


# ---------------------------------------------------------------------------- configuration
@dataclass
class Push:
    """One collision. phase: 'time' (at t), 'prelock' (at t, tracker not locked yet), 'inview'
    (locked, tag seen < 50 ms ago, measured goal distance < dist), 'coast' (locked, tag unseen
    for >= gap s, measured goal distance < dist, not done yet)."""

    phase: str = "time"
    t: tuple = (0.5, 1.5)  # s, uniform range (time / prelock)
    force: tuple = (300.0, 800.0)  # N, uniform range
    dur: tuple = (0.2, 0.5)  # s, uniform range
    lever: float = 0.3  # m, torque = force * U(-lever, lever)
    direction: str = "random"  # 'random' (field angle) or 'lateral' (+-90 deg to the tag normal)
    dist: float = 0.6  # inview trigger
    gap: float = 0.06  # coast trigger


@dataclass
class Config:
    pushes: tuple = ()
    start_vel_angle: float = 0.0  # rad, start velocity direction relative to the goal direction
    record: bool = False  # keep per-tick series (detection study)
    # measurement noise for the detectors (own rng)
    wheel_speed_noise: float = 0.01  # m/s
    steer_noise: float = math.radians(0.15)
    gyro_rate_noise: float = 0.005  # rad/s
    imu_noise_g: float = 0.01  # accelerometer white noise per 250 Hz sample (g)
    imu_bias_g: float = 0.01  # per-trial constant bias, 1 sigma (g); estimated online
    imu_bias_resid_g: float = 0.002  # bias estimate error at the start of the trial, 1 sigma (g)
    # skid flag used by mitigations: signal -> threshold; flagged if any exceeds; held for `hold` s
    detect: dict = dfield(default_factory=lambda: {"imu_vel": 0.12, "imu_acc": 3.0, "skid_dev": 0.25})
    hold: float = 0.15
    # mitigations in the odometry
    odom: str = "ls"  # 'ls' (WPILib), 'drop' (drop modules flagged by skid_dev), 'bridge' (IMU)
    drop_dev: float = 0.15  # m/s, module deviation from the median translation to drop it
    radius_error: float = None  # fixed wheel-radius error (fraction); None = World's random draw
    fix_plant: bool = True  # use FixedPlant (core encoder/pose integration-order workaround)


# ---------------------------------------------------------------------------- plant workaround
# The core plant now moves the pose with the velocity the encoders integrated (the bug this
# study found), so the old workaround is just the core plant.
FixedPlant = SwervePlant


# ---------------------------------------------------------------------------- odometry variants
class SkidOdometry(SwerveOdometry):
    """WPILib least squares, plus optional per-module weights and an external field-frame
    displacement added after each update (IMU bridging)."""

    def __init__(self, module_positions, gyro, modules, pose=(0.0, 0.0, 0.0)):
        super().__init__(module_positions, gyro, modules, pose)
        rows = []
        for mx, my in module_positions:
            rows.append([1.0, 0.0, -my])
            rows.append([0.0, 1.0, mx])
        self.A = np.array(rows)
        self.weights = None  # None = all modules
        self.extra = (0.0, 0.0)

    def update(self, gyro, modules):
        if self.weights is None or all(w > 0 for w in self.weights):
            pose = super().update(gyro, modules)
        else:
            d = []
            for i, (dist, ang) in enumerate(modules):
                delta = dist - self.prev_dist[i]
                d.append(delta * math.cos(ang))
                d.append(delta * math.sin(ang))
                self.prev_dist[i] = dist
            w = np.repeat(np.array(self.weights, dtype=float), 2)
            tw = np.linalg.lstsq(self.A * w[:, None], np.array(d) * w, rcond=None)[0]
            angle = wrap(gyro + self.gyro_offset)
            dth = wrap(angle - self.prev_angle)
            new = compose(self.pose, twist_exp(float(tw[0]), float(tw[1]), dth))
            self.pose = (new[0], new[1], angle)
            self.prev_angle = angle
            pose = self.pose
        if self.extra != (0.0, 0.0):
            self.pose = (pose[0] + self.extra[0], pose[1] + self.extra[1], pose[2])
            self.extra = (0.0, 0.0)
        return self.pose


# ---------------------------------------------------------------------------- detectors
SIGNALS = ("skid_ratio", "skid_dev", "ls_resid", "imu_acc", "imu_vel", "cur_frac", "cmd_err")


class Monitor:
    """Skid detectors at the odometry rate, from what the robot can measure:
    module velocities (noisy), gyro rate, a synthesized accelerometer (true accel + bias + noise),
    and the drive motors' stator current / velocity error.

    Also keeps the ground truth: per-tick odometry error vs the true motion, and wheel slip."""

    def __init__(self, rp, cfg: Config, rng, dt):
        self.rp, self.cfg, self.rng, self.dt = rp, cfg, rng, dt
        self.mods = rp.module_positions
        self.r2 = rp.module_xy ** 2 * 2
        self.bias = (rng.normal(0, cfg.imu_bias_g * G), rng.normal(0, cfg.imu_bias_g * G))
        # online bias estimate, already converged to within imu_bias_resid_g from earlier driving
        r = cfg.imu_bias_resid_g * G
        self.b_est = [self.bias[0] + rng.normal(0, r), self.bias[1] + rng.normal(0, r)]
        self.dv = [0.0, 0.0]  # leaky integral of (IMU accel - wheel accel), field frame
        self.prev_true_v = None
        self.prev_vw = None  # wheel-derived field velocity at the previous tick
        self.acc_hist = deque(maxlen=10)  # (a_imu_field, v_wheel_field) for the 40 ms average
        self.skid_t = -1e9  # last tick flagged
        self.first_flag = None
        self.flag_count = 0
        self.sig = {k: 0.0 for k in SIGNALS}
        self.peak = {k: 0.0 for k in SIGNALS}
        self.prev_true = None
        self.prev_odom = None
        self.series = [] if cfg.record else None
        # IMU bridging state
        self.bridging = False
        self.v_b = (0.0, 0.0)
        self.br_start = 0.0
        self.br_quiet = 0.0
        self.hist = deque(maxlen=int(0.4 / dt) + 1)  # (dv_mag, v_wheel_f, a_imu_f)
        self.bridge_count = 0

    def active(self, t):
        return t - self.skid_t <= self.cfg.hold

    # ---------------------------------------------------------------- before the odometry update
    def pre(self, t, plant: SwervePlant, odo: SkidOdometry):
        cfg, rp, rng, dt = self.cfg, self.rp, self.rng, self.dt
        heading = odo.pose[2]  # gyro-based heading (previous tick)
        # measured module velocities
        mv = []
        for s, a in plant.module_velocities():
            mv.append((s + rng.normal(0, cfg.wheel_speed_noise), a + rng.normal(0, cfg.steer_noise)))
        w_meas = plant.w * (1.0 + rp.gyro_scale_error) + rng.normal(0, cfg.gyro_rate_noise)
        # Orbit: translation part of each module, t_i = v_i - w x r_i
        T = []
        for (s, a), (mx, my) in zip(mv, self.mods):
            T.append((s * math.cos(a) + w_meas * my, s * math.sin(a) - w_meas * mx))
        mags = [math.hypot(x, y) for x, y in T]
        top = max(mags)
        ratio = top / max(min(mags), 0.02) if top >= 0.25 else 1.0
        medx = float(np.median([x for x, _ in T]))
        medy = float(np.median([y for _, y in T]))
        devs = [math.hypot(x - medx, y - medy) for x, y in T]
        # least-squares chassis fit (symmetric square layout: closed form) and its RMS residual
        bx = [s * math.cos(a) for s, a in mv]
        by = [s * math.sin(a) for s, a in mv]
        vx, vy = sum(bx) / 4.0, sum(by) / 4.0
        wz = sum(-my * bx[i] + mx * by[i] for i, (mx, my) in enumerate(self.mods)) / (4 * self.r2)
        res = 0.0
        for i, (mx, my) in enumerate(self.mods):
            res += (bx[i] - (vx - wz * my)) ** 2 + (by[i] - (vy + wz * mx)) ** 2
        ls_resid = math.sqrt(res / 8.0)
        # wheel-derived and IMU accelerations, field frame
        vw = rot((vx, vy), heading)
        tv = (plant.vx, plant.vy)
        if self.prev_true_v is None:
            a_true = (0.0, 0.0)
        else:
            a_true = ((tv[0] - self.prev_true_v[0]) / dt, (tv[1] - self.prev_true_v[1]) / dt)
        self.prev_true_v = tv
        ar = rot(a_true, -plant.th)  # robot frame (true heading)
        n = cfg.imu_noise_g * G
        a_imu_r = (ar[0] + self.bias[0] + rng.normal(0, n), ar[1] + self.bias[1] + rng.normal(0, n))
        a_imu_c = (a_imu_r[0] - self.b_est[0], a_imu_r[1] - self.b_est[1])
        a_imu = rot(a_imu_c, heading)
        if self.prev_vw is None:
            self.prev_vw = vw
        dvw = (vw[0] - self.prev_vw[0], vw[1] - self.prev_vw[1])
        self.prev_vw = vw
        k = 1.0 - dt / 0.5
        self.dv = [k * self.dv[0] + a_imu[0] * dt - dvw[0], k * self.dv[1] + a_imu[1] * dt - dvw[1]]
        imu_vel = math.hypot(*self.dv)
        self.acc_hist.append((a_imu, vw))
        if len(self.acc_hist) == self.acc_hist.maxlen:
            h = self.acc_hist
            span = dt * (len(h) - 1)
            ai = (sum(x[0][0] for x in h) / len(h), sum(x[0][1] for x in h) / len(h))
            aw = ((h[-1][1][0] - h[0][1][0]) / span, (h[-1][1][1] - h[0][1][1]) / span)
            imu_acc = math.hypot(ai[0] - aw[0], ai[1] - aw[1])
        else:
            imu_acc = 0.0
        # drive motor current / velocity error (same law as the plant, at this instant)
        cur, cerr = 0.0, 0.0
        for i in range(4):
            sw = plant.wheel[i]
            if plant.neutral:
                volts, sp = 0.0, 0.0
            else:
                sp = plant.target_speed[i]
                if rp.cosine_compensation:
                    sp *= math.cos(wrap(plant.target_angle[i] - plant.steer[i]))
                volts = plant.kv_volts * sp + rp.drive_kp * (sp - sw)
                if abs(sp) > 1e-3:
                    volts += math.copysign(rp.drive_ks, sp)
                volts = max(-12.0, min(12.0, volts))
            rotor = sw / rp.wheel_radius * rp.drive_gear
            amps = max(-rp.stator_limit, min(rp.stator_limit, (volts - rotor / rp.motor_kv) / rp.motor_r))
            cur = max(cur, abs(amps) / rp.stator_limit)
            cerr = max(cerr, abs(sp - sw))
        sig = self.sig
        sig["skid_ratio"], sig["skid_dev"], sig["ls_resid"] = ratio, max(devs), ls_resid
        sig["imu_acc"], sig["imu_vel"], sig["cur_frac"], sig["cmd_err"] = imu_acc, imu_vel, cur, cerr
        for key, val in sig.items():
            if val > self.peak[key]:
                self.peak[key] = val
        flagged = any(sig[key] > thr for key, thr in cfg.detect.items())
        if flagged:
            if self.first_flag is None:
                self.first_flag = t
            if not self.active(t):
                self.flag_count += 1
            self.skid_t = t
        # bias estimate while no skid (robot frame, 1 s time constant, noisy wheel accel averaged)
        if not self.active(t) and len(self.acc_hist) == self.acc_hist.maxlen:
            a_w_r = rot(aw, -heading)
            kb = dt / 1.0
            self.b_est[0] += kb * (a_imu_r[0] - a_w_r[0] - self.b_est[0])
            self.b_est[1] += kb * (a_imu_r[1] - a_w_r[1] - self.b_est[1])
        # (b) drop modules that disagree with the others
        if cfg.odom == "drop":
            w = [0.0 if (top >= 0.1 and d > cfg.drop_dev) else 1.0 for d in devs]
            if sum(w) < 2:
                w = [1.0] * 4
            odo.weights = w
        # (f) IMU bridging: integrate the IMU through a flagged skid, replaying from its onset
        if cfg.odom == "bridge":
            self._bridge(t, vw, a_imu, odo)

    def _bridge(self, t, vw, a_imu, odo):
        """(f) While the IMU and wheels disagree, dead-reckon velocity from the IMU, starting from the
        last wheel velocity before the disagreement began (replayed), until the wheels grip again
        (their acceleration matches the IMU's and the modules agree) for 0.1 s; at most 2 s."""
        dt, sig = self.dt, self.sig
        thr_v, thr_a = self.cfg.detect.get("imu_vel", 0.12), self.cfg.detect.get("imu_acc", 3.0)
        self.hist.append((sig["imu_vel"], vw, a_imu))
        ex, ey = 0.0, 0.0
        if not self.bridging and (sig["imu_vel"] > thr_v or sig["imu_acc"] > thr_a):
            h = list(self.hist)
            j = len(h) - 1
            while j > 0 and h[j][0] > 0.02:
                j -= 1
            vb = h[j][1]
            for k in range(j + 1, len(h)):
                vb = (vb[0] + h[k][2][0] * dt, vb[1] + h[k][2][1] * dt)
                ex += (vb[0] - h[k][1][0]) * dt
                ey += (vb[1] - h[k][1][1]) * dt
            self.v_b = vb
            self.bridging = True
            self.br_start = t
            self.br_quiet = 0.0
            self.bridge_count += 1
        elif self.bridging:
            vb = (self.v_b[0] + a_imu[0] * dt, self.v_b[1] + a_imu[1] * dt)
            self.v_b = vb
            ex += (vb[0] - vw[0]) * dt
            ey += (vb[1] - vw[1]) * dt
            grip = sig["imu_acc"] < 1.5 and sig["skid_dev"] < 0.1
            self.br_quiet = self.br_quiet + dt if grip else 0.0
            if self.br_quiet >= 0.1 or t - self.br_start > 2.0:
                self.bridging = False
                self.dv = [0.0, 0.0]
                self.hist.clear()
        if ex or ey:
            odo.extra = (ex, ey)

    # ---------------------------------------------------------------- after the odometry update
    def post(self, t, plant, odo):
        true = plant.pose()
        if self.prev_true is not None:
            dt_true = relative(self.prev_true, true)
            dt_odom = relative(self.prev_odom, odo.pose)
            e = math.hypot(dt_odom[0] - dt_true[0], dt_odom[1] - dt_true[1])
        else:
            e = 0.0
        self.prev_true, self.prev_odom = true, odo.pose
        if self.series is not None:
            s = self.sig
            self.series.append((t, true[0], true[1], true[2], odo.pose[0], odo.pose[1], odo.pose[2], e,
                                float(any(plant.slipping)), s["skid_ratio"], s["skid_dev"], s["ls_resid"],
                                s["imu_acc"], s["imu_vel"], s["cur_frac"], s["cmd_err"],
                                math.hypot(plant.vx, plant.vy)))


SERIES_COLS = ("t", "tx", "ty", "tth", "ox", "oy", "oth", "e", "slip") + SIGNALS + ("speed",)


# ---------------------------------------------------------------------------- tracker variants
class SkidTracker(FrameTracker):
    """FrameTracker plus skid responses, driven by snap['skid'] (the monitor's flag):
    inflate=(xy m, yaw rad): (a) during a skid and `after` s past it, raise the filter variance
        so new frames dominate.
    reacquire=True: (c) after a skid the lock is suspect until `lock_frames` fresh frames arrive.
        A suspect lock can't finish. If the tag is out of view (unseen > coast_gap) while suspect,
        drop the estimate and back straight off `backoff` m from where the robot is now (facing the
        old goal heading; the gyro keeps heading good) until it re-locks.
    """

    def __init__(self, tag_id, goal_in_tag, inflate=None, reacquire=False, backoff=0.30, coast_gap=0.06,
                 after=0.3, **kw):
        self.inflate, self.reacquire, self.backoff, self.coast_gap = inflate, reacquire, backoff, coast_gap
        self.after = after
        super().__init__(tag_id, goal_in_tag, **kw)
        self.name = "skid-" + self.name

    def reset(self):
        super().reset()
        self.stale_goal = None  # odom-frame backoff point while re-acquiring
        self.invalidations = 0
        self.skid_end = None  # last update time a skid was flagged while we had an estimate
        self.fresh = 0  # frames accepted since then

    def _kf_locked(self):
        return self.kf.est is not None and self.kf.n >= self.lock_frames

    def suspect(self):
        return self.skid_end is not None and self.fresh < self.lock_frames

    def update(self, t, snap, frames, est, field):
        skid_on = snap.get("skid", False)
        if skid_on and self.kf.est is not None:
            self.skid_end, self.fresh = t, 0
        if self.inflate is not None and self.kf.est is not None and self.skid_end is not None \
                and t - self.skid_end <= self.after:
            xy2, yaw2 = self.inflate[0] ** 2, self.inflate[1] ** 2
            v = self.kf.var
            self.kf.var = [max(v[0], xy2), max(v[1], xy2), max(v[2], yaw2)]
        n0 = self.kf.n
        super().update(t, snap, frames, est, field)
        if not skid_on and self.skid_end is not None:
            self.fresh += self.kf.n - n0 if self.kf.n >= n0 else self.kf.n
        if self.reacquire and self.suspect() and self.stale_goal is None and self.kf.est is not None \
                and not skid_on and t - self.last_seen > self.coast_gap:
            here = snap["odom"] if self.frame == "odom" else snap["pose"]
            old = compose(self.kf.est, self.goal_in_tag)
            self.stale_goal = compose((here[0], here[1], old[2]), (-self.backoff, 0.0, 0.0))
            self.kf = KalmanPose()
            self.invalidations += 1
        if self.stale_goal is not None and self._kf_locked():
            self.stale_goal = None
        if self.skid_end is not None and self.fresh >= self.lock_frames:
            self.skid_end = None

    def locked(self):
        if self.reacquire and (self.stale_goal is not None or self.suspect()):
            return False
        return self._kf_locked()

    def goal_field(self, snap, est, field):
        frame = snap["odom"] if self.frame == "odom" else snap["pose"]
        if self.stale_goal is not None and not self._kf_locked():
            return compose(snap["pose"], relative(frame, self.stale_goal))
        if not self._kf_locked():
            return None
        goal = compose(self.kf.est, self.goal_in_tag)
        return compose(snap["pose"], relative(frame, goal)) if self.frame == "odom" else goal


class SkidGlobalVision(GlobalVision):
    """(e) Global fusion that trusts vision more (std * factor) for 1 s after a skid."""

    def __init__(self, factor=0.2, **kw):
        super().__init__(**kw)
        self.factor = factor
        self.monitor = None
        self._base = self.base_std

    def apply(self, frames, est, field, omega):
        recent = self.monitor is not None and self.monitor.skid_t > -1e8 and \
            (est.odom_buffer.t[-1] - self.monitor.skid_t) < 1.0
        self.base_std = self._base * (self.factor if recent else 1.0)
        super().apply(frames, est, field, omega)
        self.base_std = self._base


# ---------------------------------------------------------------------------- the world
class SkidWorld(World):
    def __init__(self, sc, seed, cfg: Config):
        super().__init__(sc, seed)
        self.cfg = cfg
        if cfg.radius_error is not None:
            self.rp = replace(self.rp, wheel_radius_error=cfg.radius_error)
        self.rng2 = np.random.default_rng(seed + 7_000_001)  # pushes
        self.rng3 = np.random.default_rng(seed + 9_000_011)  # detector noise
        self.push_plan = []
        for p in cfg.pushes:
            r = self.rng2
            self.push_plan.append({
                "spec": p,
                "t": r.uniform(*p.t),
                "force": r.uniform(*p.force),
                "dur": r.uniform(*p.dur),
                "lever": r.uniform(-p.lever, p.lever),
                "angle": r.uniform(-math.pi, math.pi),
                "side": 1.0 if r.uniform() < 0.5 else -1.0,
                "fired": None,
            })

    def _fire(self, plan, t, plant, odo):
        p = plan["spec"]
        if p.direction == "lateral":
            ang = self.field.target.true_pose[2] + plan["side"] * math.pi / 2
        else:
            ang = plan["angle"]
        f = plan["force"]
        plant.push(f * math.cos(ang), f * math.sin(ang), f * plan["lever"], plan["dur"])
        plan["fired"] = t
        plan["true_at"] = plant.pose()
        plan["odom_at"] = odo.pose

    def _check_pushes(self, t, plant, odo, tracker, goal, snap, done_t):
        for plan in self.push_plan:
            if plan["fired"] is not None:
                continue
            ph = plan["spec"].phase
            if ph == "time":
                if t >= plan["t"]:
                    self._fire(plan, t, plant, odo)
            elif ph == "prelock":
                if t >= plan["t"] and (tracker is None or not tracker.locked()):
                    self._fire(plan, t, plant, odo)
            elif tracker is not None and tracker.locked() and done_t is None:
                d = math.hypot(goal[0] - snap["pose"][0], goal[1] - snap["pose"][1])
                seen_ago = t - tracker.last_seen
                if ph == "inview" and seen_ago < 0.05 and d < plan["spec"].dist:
                    self._fire(plan, t, plant, odo)
                elif ph == "coast" and seen_ago >= plan["spec"].gap and d < plan["spec"].dist:
                    self._fire(plan, t, plant, odo)

    def run(self, st, record=False):
        sc, rng, rp, fld = self.sc, self.rng, self.rp, self.field
        cfg = self.cfg
        lp = sc.loop
        goal_in_tag = sc.field.goal_in_tag
        if st.teach:
            goal_in_tag = self.teach(goal_in_tag)

        tg = self.true_goal
        lat = rng.uniform(-sc.start_lateral, sc.start_lateral)
        start = compose(tg, (-sc.start_dist, lat, rng.uniform(-sc.start_heading, sc.start_heading)))
        to_goal = (tg[0] - start[0], tg[1] - start[1])
        n = math.hypot(*to_goal)
        # HOOK: start velocity may point away from the goal (direction reversals)
        u = rot((to_goal[0] / n, to_goal[1] / n), cfg.start_vel_angle)
        v0 = (u[0] * sc.start_speed, u[1] * sc.start_speed, 0.0)
        plant = (FixedPlant if cfg.fix_plant else SwervePlant)(rp, start, v0, rng)
        e = sc.errors
        believed = (start[0] + rng.normal(0, e.init_xy), start[1] + rng.normal(0, e.init_xy),
                    wrap(start[2] + rng.normal(0, e.init_yaw)))
        # HOOK: odometry variant + monitor
        odo = SkidOdometry(rp.module_positions, plant.read_gyro(), plant.read_modules(), believed)
        est = PoseEstimator(odo, state_std=(0.1, 0.1, 0.1))
        est.update(0.0, plant.read_gyro(), plant.read_modules())
        mon = Monitor(rp, cfg, self.rng3, lp.odom_period)
        self.monitor = mon
        if isinstance(st.vision, SkidGlobalVision):
            st.vision.monitor = mon

        ctrl = st.controller()
        tracker = st.tracker(fld.TARGET_ID, goal_in_tag, sc, self.cams) if st.tracker else None
        map_goal = compose(fld.target.map_pose, goal_in_tag)

        dt = lp.sim_dt
        t = 0.0
        next_odom = 0.0
        next_main = lp.main_period
        pending = []
        requests = []
        cur_req = (v0[0], v0[1], 0.0)
        snap = self._snap2(t, est, odo, plant, mon)
        ctrl.reset(snap["pose"], snap["vel"])
        staged = st.approach != "staged"
        done_t = None
        hold_since = None
        lock_t = None
        res = {"err_done": None}
        peak_acc = 0.0
        last_cmd = (v0[0], v0[1])
        last_frame_t = None  # HOOK: last time the tracker accepted a frame
        goal = map_goal
        steps = int(sc.t_end / dt)
        for _ in range(steps):
            for cam in self.cams:
                fr = cam.maybe_capture(t, plant.pose(), plant.field_velocity(), fld, est.estimate[2], sc.pieces)
                if fr is not None:
                    pending.append(fr)
            if t >= next_odom - 1e-9:
                next_odom += lp.odom_period
                while requests and requests[0][0] <= t:
                    cur_req = requests.pop(0)[1]
                if cur_req is None:
                    plant.set_neutral()
                else:
                    vr = rot((cur_req[0], cur_req[1]), -est.estimate[2])
                    plant.set_chassis_robot(vr[0], vr[1], cur_req[2])
                # HOOK: same rng order as World (gyro, then modules); detectors before the update
                gyro = plant.read_gyro()
                modules = plant.read_modules()
                mon.pre(t, plant, odo)
                est.update(t, gyro, modules)
                mon.post(t, plant, odo)
                snap = self._snap2(t, est, odo, plant, mon)
            if t >= next_main - 1e-9:
                next_main += lp.main_period + rng.normal(0, lp.main_jitter)
                arrived = [f for f in pending if f.t_arrive <= t]
                pending = [f for f in pending if f.t_arrive > t]
                st.vision.apply(arrived, est, fld, snap["vel"][2])
                snap = self._snap2(t, est, odo, plant, mon)
                if tracker is not None:
                    ls_before = tracker.last_seen
                    tracker.update(t, snap, arrived, est, fld)
                    if tracker.last_seen != ls_before:
                        last_frame_t = t
                    if lock_t is None and tracker.locked():
                        lock_t = t
                local = tracker.goal_field(snap, est, fld) if tracker is not None else None
                if st.approach == "global" or local is None:
                    goal = map_goal
                else:
                    goal = local
                if not staged:
                    stage_goal = compose(goal, (-st.stage_dist, 0.0, 0.0))
                    d_stage = math.hypot(stage_goal[0] - snap["pose"][0], stage_goal[1] - snap["pose"][1])
                    if d_stage < 0.15:
                        staged = True
                        if hasattr(ctrl, "end_speed"):
                            ctrl.end_speed = 0.0
                    else:
                        goal = stage_goal
                        if hasattr(ctrl, "end_speed"):
                            ctrl.end_speed = st.stage_speed
                if st.approach == "shaped":
                    goal = _shape_6328(snap["pose"], goal)
                # HOOK: collisions
                self._check_pushes(t, plant, odo, tracker, goal, snap, done_t)
                cmd = ctrl.step(snap["pose"], goal, snap["vel"], lp.main_period)
                if st.slow_zone is not None:
                    d = math.hypot(goal[0] - snap["pose"][0], goal[1] - snap["pose"][1])
                    if d < st.slow_zone[0]:
                        sp = math.hypot(cmd[0], cmd[1])
                        if sp > st.slow_zone[1]:
                            cmd = (cmd[0] * st.slow_zone[1] / sp, cmd[1] * st.slow_zone[1] / sp, cmd[2])
                acc = math.hypot(cmd[0] - last_cmd[0], cmd[1] - last_cmd[1]) / lp.main_period
                peak_acc = max(peak_acc, acc)
                last_cmd = (cmd[0], cmd[1])
                requests.append((t + lp.actuation_delay, cmd))
                if done_t is None and (tracker is None or tracker.locked() or st.approach == "global"):
                    err = relative(goal, snap["pose"])
                    speed = math.hypot(snap["vel"][0], snap["vel"][1])
                    if math.hypot(err[0], err[1]) < st.pos_tol and abs(err[2]) < st.yaw_tol \
                            and speed < st.settle_speed:
                        hold_since = hold_since if hold_since is not None else t
                        if t - hold_since >= st.debounce:
                            done_t = t
                            res["err_done"] = relative(self.true_goal, plant.pose())
                            res["odom_at_done"] = odo.pose
                            res["true_at_done"] = plant.pose()
                            res["last_frame_t"] = last_frame_t
                    else:
                        hold_since = None
            plant.step(dt)
            t += dt

        res.update(self._metrics(plant, est, odo, start, done_t, lock_t, peak_acc))
        res["pushes"] = [{k: v for k, v in p.items() if k != "spec"} for p in self.push_plan]
        res["flag_first"] = mon.first_flag
        res["flag_count"] = mon.flag_count
        res["peak"] = dict(mon.peak)
        res["bridges"] = mon.bridge_count
        res["invalidations"] = getattr(tracker, "invalidations", 0) if tracker is not None else 0
        if mon.series is not None:
            res["series"] = np.array(mon.series, dtype=np.float32)
        return res

    def _snap2(self, t, est, odo, plant, mon):
        s = self._snap(t, est, odo, plant)
        s["skid"] = mon.active(t)
        s["skid_t"] = mon.skid_t
        return s


def run_skid(sc, st, cfg, seed):
    return SkidWorld(sc, seed, cfg).run(st)


def run_set(sc, st, cfg, n, seed0=0):
    return [run_skid(sc, st, cfg, seed0 + i) for i in range(n)]


# ---------------------------------------------------------------------------- metrics
def window_error(series, a, b):
    """Odometry error over [a, b]: the robot's relative motion per odometry vs truth.
    Returns (position m, heading rad)."""
    t = series[:, 0]
    i, j = int(np.searchsorted(t, a)), int(np.searchsorted(t, b))
    j = min(j, len(t) - 1)
    i = min(i, j)
    ta, tb = tuple(map(float, series[i, 1:4])), tuple(map(float, series[j, 1:4]))
    oa, ob = tuple(map(float, series[i, 4:7])), tuple(map(float, series[j, 4:7]))
    e = relative(relative(ta, tb), relative(oa, ob))
    return math.hypot(e[0], e[1]), abs(e[2])


def pct(x, q):
    return float(np.percentile(x, q)) if len(x) else float("nan")


__all__ = ["Push", "Config", "SkidWorld", "SkidOdometry", "Monitor", "SkidTracker", "SkidGlobalVision",
           "run_skid", "run_set", "window_error", "SIGNALS", "SERIES_COLS", "replace"]
