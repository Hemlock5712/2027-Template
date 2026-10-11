#!/usr/bin/env python3
"""Monte Carlo test bench: global vs local alignment strategies for an FRC swerve robot.

Models: 50 Hz control, 250 Hz odometry with scale error, a camera with latency/noise/FOV and
extrinsic error, a field element whose real pose differs from the map, and a fused (global) pose
estimator that blends odometry with MegaTag-style vision (other tags + the target tag).

The controller is the same law as src/main/java/frc/robot/commands/DriveToPose.java. All noise and
error sizes in Cfg are ASSUMPTIONS - replace them with numbers measured from robot logs.

Strategies (all drive on the global pose until the target is locked):
  global              drive to map goal on the fused field pose
  global_tagfilter    same, but the fused pose only ever sees the target tag (581/254/604 trick)
  local_raw           servo on the latest raw robot-in-tag pose, no latency comp (old DriveToTag)
  local_fused_hist    tag placed with the fused pose at capture time (samplePoseAt style)
  local_odom          tag placed with WHEEL-ONLY odometry at capture time (TagTracker)
  local_odom_mapyaw   local_odom, but trust the field map's tag yaw instead of the measured yaw
  local_odom_prior    local_odom seeded from the map pose as a prior
  local_odom_taught   local_odom with a taught tagToGoal (Teach Tag Goal OpMode)

Usage: python3 align_sim.py [--trials N] [--scenario base|element|timestamp|pnpbias|rest|all]
"""
import argparse
import bisect
import math
import numpy as np

TAU = 2 * math.pi


def wrap(a):
    return (a + math.pi) % TAU - math.pi


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def relative(a, b):
    """b expressed in a's frame."""
    return compose(inverse(a), b)


def rot(v, th):
    c, s = math.cos(th), math.sin(th)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1])


class History:
    def __init__(self):
        self.t, self.p = [], []

    def add(self, t, p):
        self.t.append(t)
        self.p.append(p)

    def sample(self, t):
        i = bisect.bisect_left(self.t, t)
        if i <= 0:
            return self.p[0]
        if i >= len(self.t):
            return self.p[-1]
        t0, t1 = self.t[i - 1], self.t[i]
        u = (t - t0) / (t1 - t0) if t1 > t0 else 0
        a, b = self.p[i - 1], self.p[i]
        return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), wrap(a[2] + u * wrap(b[2] - a[2])))


# ----------------------------------------------------------------------------------------------
# Controller: drive-to-point with braking law + linear zone + acceleration limiting.
# ----------------------------------------------------------------------------------------------
class DriveToPoint:
    def __init__(self, cfg, linear_zone=True):
        self.cfg = cfg
        self.linear_zone = linear_zone
        self.v = np.zeros(2)  # last commanded velocity (frame of the controller)
        self.w = 0.0

    def seed(self, v, w):
        self.v = np.array(v, dtype=float)
        self.w = w

    def step(self, pose, goal, dt):
        c = self.cfg
        e = np.array([goal[0] - pose[0], goal[1] - pose[1]])
        d = float(np.hypot(*e))
        speed_now = float(np.hypot(*self.v))
        # Braking law: fastest speed that can still stop at the goal, minus a reaction buffer.
        d_buf = max(0.0, d - speed_now * c.react)
        v_mag = min(c.vmax, math.sqrt(2 * c.decel * d_buf))
        if self.linear_zone:
            v_mag = min(v_mag, c.kp * d)
        v_des = e / d * v_mag if d > 1e-6 else np.zeros(2)

        eth = wrap(goal[2] - pose[2])
        w_mag = min(c.wmax, math.sqrt(2 * c.alpha * abs(eth)))
        if self.linear_zone:
            w_mag = min(w_mag, c.kth * abs(eth))
        w_des = math.copysign(w_mag, eth)

        dv = v_des - self.v
        n = float(np.hypot(*dv))
        if n > c.amax * dt:
            dv *= c.amax * dt / n
        self.v = self.v + dv
        dw = w_des - self.w
        self.w += max(-c.alpha_max * dt, min(c.alpha_max * dt, dw))
        return self.v.copy(), self.w


# ----------------------------------------------------------------------------------------------
# Target tracker: one running estimate of the tag pose in some frame (odom or fused).
# ----------------------------------------------------------------------------------------------
class TagTracker:
    def __init__(self, q_pos=1e-6, q_yaw=1e-6, gate=0.25):
        self.est = None
        self.var = None
        self.q = np.array([q_pos, q_pos, q_yaw])
        self.gate = gate
        self.rejects = 0
        self.count = 0

    def update(self, tag_pose, r):
        m = np.array(tag_pose, dtype=float)
        r = np.asarray(r, dtype=float)
        if self.est is None:
            self.est, self.var, self.count = m, r.copy(), 1
            return
        innov = m - self.est
        innov[2] = wrap(innov[2])
        if math.hypot(innov[0], innov[1]) > self.gate:
            self.rejects += 1
            if self.rejects >= 3:  # three in a row: the old estimate was wrong, restart
                self.est, self.var, self.rejects, self.count = m, r.copy(), 0, 1
            return
        self.rejects = 0
        self.var = self.var + self.q
        k = self.var / (self.var + r)
        self.est = self.est + k * innov
        self.est[2] = wrap(self.est[2])
        self.var = (1 - k) * self.var
        self.count += 1


# ----------------------------------------------------------------------------------------------
# One trial.
# ----------------------------------------------------------------------------------------------
class Cfg:
    dt = 0.002
    t_end = 4.0
    ctrl_period = 0.02
    odom_period = 0.004
    act_delay = 0.010  # control output -> wheels
    tau = 0.04  # velocity tracking time constant
    a_phys = 9.0  # traction limit (m/s^2)
    # controller
    vmax, decel, react, kp = 3.5, 3.0, 0.05, 4.0
    wmax, alpha, kth, alpha_max = 6.0, 8.0, 5.0, 20.0
    amax = 6.0
    # camera
    fps = 40.0
    lat_mean, lat_jit = 0.030, 0.008
    fov = math.radians(35)
    rmin, rmax = 0.25, 4.5
    cam = (0.30, 0.0, 0.0)  # camera on robot, facing forward
    # standoff: robot center 0.55 m out from the tag face, facing it
    standoff = (0.55, 0.0, math.pi)
    # errors (1 sigma)
    elem_xy, elem_yaw = 0.025, math.radians(1.0)
    odom_scale = 0.015
    cam_xy, cam_yaw = 0.003, math.radians(0.3)
    pnp_yaw_bias = math.radians(0.5)  # per-trial systematic PnP yaw error (calibration, tag flatness)
    global_bias = 0.03
    other_rate = 0.5  # chance a frame also sees other (non-target) tags
    other_sigma = 0.04
    prior_xy, prior_yaw = 0.05, math.radians(1.5)
    yaw_floor = math.radians(0.7)  # assumed per-frame systematic part of PnP yaw
    lock_frames = 3


def meas_noise(d):
    lat = 0.002 + 0.0015 * d
    dep = 0.002 + 0.006 * d * d
    yaw = math.radians(0.8 + 1.2 * d)
    return lat, dep, yaw


def run_trial(strategy, rng, cfg, linear_zone=True, start_speed=1.5, lat_error=0.0):
    tag_map = (0.0, 0.0, 0.0)  # tag at origin, face pointing +x
    tag_true = (rng.normal(0, cfg.elem_xy), rng.normal(0, cfg.elem_xy), rng.normal(0, cfg.elem_yaw))
    goal_map = compose(tag_map, cfg.standoff)
    goal_true = compose(tag_true, cfg.standoff)
    cam_true = compose(cfg.cam, (rng.normal(0, cfg.cam_xy), rng.normal(0, cfg.cam_xy),
                                 rng.normal(0, cfg.cam_yaw)))
    yaw_bias = rng.normal(0, cfg.pnp_yaw_bias)
    sx, sy = 1 + rng.normal(0, cfg.odom_scale), 1 + rng.normal(0, cfg.odom_scale)
    other_bias = np.array([rng.normal(0, cfg.global_bias), rng.normal(0, cfg.global_bias)])

    # start ~2.8 m out, random lateral offset and heading, moving toward the goal
    cam_at_goal = compose(goal_true, cam_true)
    rel_goal = relative(cam_at_goal, tag_true)
    taught_r2t = compose(cfg.cam, (rel_goal[0], rel_goal[1], wrap(rel_goal[2] + yaw_bias)))
    taught_tag_to_goal = inverse(taught_r2t)  # goal (robot) pose in the tag frame, as measured
    lateral = rng.uniform(-0.8, 0.8)
    start = (2.8, lateral, wrap(math.pi + rng.uniform(-0.45, 0.45)))
    x = list(start)
    to_goal = np.array([goal_map[0] - start[0], goal_map[1] - start[1]])
    vel = to_goal / np.linalg.norm(to_goal) * start_speed
    w = 0.0

    odom = list(start)
    corr = np.array([other_bias[0], other_bias[1]])  # fused = odom + corr (translation only)
    odom_hist, fused_hist = History(), History()
    odom_hist.add(0.0, tuple(odom))
    fused_hist.add(0.0, (odom[0] + corr[0], odom[1] + corr[1], odom[2]))

    ctrl = DriveToPoint(cfg, linear_zone)
    ctrl.seed(vel, w)
    tracker = TagTracker()
    raw_latest = None  # (robot pose in tag frame) for the naive strategy
    seen = 0
    locked = False

    cmd_queue = [(0.0, vel.copy(), w)]  # (apply_time, field velocity, omega)
    cmd_v, cmd_w = vel.copy(), w
    pending = []  # camera frames in flight: (arrive_time, capture_time_reported, meas)
    next_frame = rng.uniform(0, 1 / cfg.fps)
    next_ctrl, next_odom = 0.0, 0.0
    traj = []
    t = 0.0
    while t < cfg.t_end:
        # ---- camera capture ----
        if t >= next_frame:
            next_frame += 1 / cfg.fps
            cam_pose = compose(tuple(x), cam_true)
            rel = relative(cam_pose, tag_true)  # tag in camera frame
            d = math.hypot(rel[0], rel[1])
            bearing = math.atan2(rel[1], rel[0])
            facing = abs(wrap(rel[2] + math.pi - bearing))  # angle between ray and tag normal
            if cfg.rmin < d < cfg.rmax and abs(bearing) < cfg.fov and facing < math.radians(70):
                lat, dep, yaw = meas_noise(d)
                # noise along ray (depth) and across it (lateral)
                n_dep, n_lat = rng.normal(0, dep), rng.normal(0, lat)
                ux, uy = math.cos(bearing), math.sin(bearing)
                noisy = (rel[0] + ux * n_dep - uy * n_lat, rel[1] + uy * n_dep + ux * n_lat,
                         wrap(rel[2] + yaw_bias + rng.normal(0, yaw)))
                robot_to_tag = compose(cfg.cam, noisy)  # robot code assumes nominal extrinsics
                latency = max(0.01, rng.normal(cfg.lat_mean, cfg.lat_jit))
                pending.append((t + latency, t + lat_error, robot_to_tag, d))

        # ---- odometry (250 Hz) ----
        if t >= next_odom:
            next_odom += cfg.odom_period
            odom_hist.add(t, tuple(odom))
            fused_hist.add(t, (odom[0] + corr[0], odom[1] + corr[1], odom[2]))

        # ---- control (50 Hz) ----
        if t >= next_ctrl:
            next_ctrl += cfg.ctrl_period
            arrived = [f for f in pending if f[0] <= t]
            pending = [f for f in pending if f[0] > t]
            if arrived and strategy != "global_tagfilter" and rng.random() < cfg.other_rate:
                # other field tags: unbiased w.r.t. the field except a per-trial bias
                tc0 = arrived[-1][1]
                fused_tc = fused_hist.sample(tc0)
                true_tc = tuple(x)  # close enough: frames are <= 40 ms old
                meas = np.array(true_tc[:2]) + other_bias + rng.normal(0, cfg.other_sigma, 2)
                q, r = 0.03 ** 2, 0.3 ** 2
                corr = corr + q / (q + math.sqrt(q * r)) * (meas - np.array(fused_tc[:2]))
            for _, tc, r2t, d in arrived:
                # global vision update (MegaTag2-style: gyro heading, tag map position)
                fused_tc = fused_hist.sample(tc)
                odom_tc = odom_hist.sample(tc)
                tag_in_fused = compose(fused_tc, r2t)
                meas_robot_xy = np.array([tag_map[0], tag_map[1]]) - np.array(
                    rot((r2t[0], r2t[1]), fused_tc[2]))
                sig_v = 0.05 + 0.1 * d * d
                q = 0.03 ** 2
                k = q / (q + math.sqrt(q * sig_v ** 2))
                corr = corr + k * (meas_robot_xy - np.array(fused_tc[:2]))
                lat, dep, yaw = meas_noise(d)
                r_xy = (dep ** 2 + lat ** 2)
                seen += 1
                if strategy == "local_raw":
                    raw_latest = relative(r2t, (0.0, 0.0, 0.0))  # robot pose in tag frame
                elif strategy == "local_fused_hist":
                    tracker.update(tag_in_fused, (r_xy, r_xy, yaw ** 2))
                elif strategy == "local_odom_prior":
                    if tracker.est is None:
                        fused_now0 = (odom[0] + corr[0], odom[1] + corr[1], odom[2])
                        prior = compose(tuple(odom), relative(fused_now0, tag_map))
                        tracker.est = np.array(prior, dtype=float)
                        tracker.var = np.array([cfg.prior_xy ** 2, cfg.prior_xy ** 2, cfg.prior_yaw ** 2])
                    tracker.update(compose(odom_tc, r2t), (r_xy, r_xy, yaw ** 2 + cfg.yaw_floor ** 2))
                elif strategy == "local_odom_taught":
                    tracker.update(compose(odom_tc, r2t), (r_xy, r_xy, yaw ** 2))
                elif strategy in ("local_odom", "local_odom_mapyaw"):
                    tag_in_odom = compose(odom_tc, r2t)
                    if strategy == "local_odom_mapyaw":
                        # trust the map's tag yaw (odom heading == gyro heading == field heading)
                        tag_in_odom = (tag_in_odom[0], tag_in_odom[1], tag_map[2])
                    tracker.update(tag_in_odom, (r_xy, r_xy, yaw ** 2))
            if strategy == "local_raw" and seen >= cfg.lock_frames:
                locked = True
            elif tracker.count >= cfg.lock_frames:
                locked = True

            fused_now = (odom[0] + corr[0], odom[1] + corr[1], odom[2])
            if strategy in ("global", "global_tagfilter") or not locked:
                v, wc = ctrl.step(fused_now, goal_map, cfg.ctrl_period)
                field_v = rot(v, x[2] - fused_now[2])
            elif strategy == "local_raw":
                v, wc = ctrl.step(raw_latest, cfg.standoff, cfg.ctrl_period)
                # tag-frame velocity -> robot frame using the stale measured yaw -> true field
                v_robot = rot(v, -raw_latest[2])
                field_v = rot(v_robot, x[2])
            elif strategy == "local_fused_hist":
                goal = compose(tuple(tracker.est), cfg.standoff)
                v, wc = ctrl.step(fused_now, goal, cfg.ctrl_period)
                field_v = rot(v, x[2] - fused_now[2])
            else:
                to_goal = taught_tag_to_goal if strategy == "local_odom_taught" else cfg.standoff
                goal = compose(tuple(tracker.est), to_goal)
                v, wc = ctrl.step(tuple(odom), goal, cfg.ctrl_period)
                field_v = rot(v, x[2] - odom[2])
            cmd_queue.append((t + cfg.act_delay, np.array(field_v), wc))

        # ---- actuation ----
        while cmd_queue and cmd_queue[0][0] <= t:
            _, cmd_v, cmd_w = cmd_queue.pop(0)
        dv = (cmd_v - vel) * min(1.0, cfg.dt / cfg.tau)
        n = float(np.hypot(*dv))
        if n > cfg.a_phys * cfg.dt:
            dv *= cfg.a_phys * cfg.dt / n
        vel = vel + dv
        w += (cmd_w - w) * min(1.0, cfg.dt / cfg.tau)

        # ---- integrate truth + odometry ----
        x[0] += vel[0] * cfg.dt
        x[1] += vel[1] * cfg.dt
        x[2] = wrap(x[2] + w * cfg.dt)
        vr = rot(vel, -x[2])
        vo = rot((vr[0] * sx, vr[1] * sy), x[2])  # odometry sees scaled wheel motion
        odom[0] += vo[0] * cfg.dt
        odom[1] += vo[1] * cfg.dt
        odom[2] = x[2]  # gyro is near-perfect over a few seconds

        traj.append((t, x[0], x[1], x[2], float(np.hypot(*vel))))
        t += cfg.dt

    err = relative(goal_true, tuple(x))
    pos_err = math.hypot(err[0], err[1])
    # settle time: last time the true error was outside 2 cm / 1 deg
    settle = 0.0
    for (tt, xx, yy, th, sp) in traj:
        e = relative(goal_true, (xx, yy, th))
        if math.hypot(e[0], e[1]) > 0.02 or abs(e[2]) > math.radians(1.0):
            settle = tt
    end_speed = np.mean([s for (tt, *_r, s) in traj if tt > cfg.t_end - 0.5])
    return dict(pos=pos_err, lat=abs(err[1]), dep=abs(err[0]), yaw=abs(math.degrees(err[2])),
                settle=settle, end_speed=end_speed)


def summarize(name, rows):
    pos = np.array([r["pos"] for r in rows]) * 100
    yaw = np.array([r["yaw"] for r in rows])
    settle = np.array([r["settle"] for r in rows])
    ok = np.mean((pos < 2.0) & (yaw < 1.0)) * 100
    jit = np.mean([r["end_speed"] for r in rows]) * 100
    print(f"{name:26s} pos err cm: med {np.median(pos):5.2f}  p95 {np.percentile(pos, 95):5.2f} | "
          f"yaw deg: med {np.median(yaw):4.2f} p95 {np.percentile(yaw, 95):4.2f} | "
          f"<2cm&<1deg {ok:5.1f}% | settle med {np.median(settle):4.2f}s | end speed {jit:4.2f} cm/s")


SCENARIOS = {
    "base": ("Baseline: element 2.5 cm / 1 deg off the map, 30 ms latency, 1.5 m/s handoff", {}, {}),
    "element": ("Element 5 cm / 2 deg off the map", dict(elem_xy=0.05, elem_yaw=math.radians(2)), {}),
    "timestamp": ("Capture timestamp 25 ms wrong", {}, dict(lat_error=-0.025)),
    "pnpbias": ("Camera yaw solve biased 1.5 deg (per trial)", dict(pnp_yaw_bias=math.radians(1.5)), {}),
    "rest": ("Start from rest", {}, dict(start_speed=0.0)),
}
STRATEGIES = ["global", "global_tagfilter", "local_raw", "local_fused_hist", "local_odom",
              "local_odom_mapyaw", "local_odom_prior", "local_odom_taught"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=150)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--scenario", default="base", choices=list(SCENARIOS) + ["all"])
    args = ap.parse_args()
    names = list(SCENARIOS) if args.scenario == "all" else [args.scenario]
    for name in names:
        title, cfg_mods, kwargs = SCENARIOS[name]
        print(f"### {title}")
        for strategy in STRATEGIES:
            cfg = Cfg()
            for k, v in cfg_mods.items():
                setattr(cfg, k, v)
            rng = np.random.default_rng(args.seed)
            summarize(strategy, [run_trial(strategy, rng, cfg, **kwargs) for _ in range(args.trials)])
        if name == "base":
            cfg = Cfg()
            rng = np.random.default_rng(args.seed)
            rows = [run_trial("local_odom", rng, cfg, linear_zone=False) for _ in range(args.trials)]
            summarize("local_odom, sqrt law only", rows)
        print()


if __name__ == "__main__":
    main()
