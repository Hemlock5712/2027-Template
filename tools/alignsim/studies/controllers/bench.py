"""Move bench: the same thread timing as World.run (500 Hz physics, odometry thread that applies
the newest request, 50 Hz loop with jitter, actuation delay) but no cameras, so the controller is
isolated. The controller sees either the TRUE pose at the last odometry tick (perfect localization),
true pose + noise, or pure wheel odometry (which slip corrupts).

A Move is a start state plus one or more legs. A leg with end_speed > 0 is a pass-through waypoint:
like the Java DriveToPose.withEndSpeed, the next leg starts (fresh controller, seeded from the
measured velocity) once the robot is within switch_radius.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from alignsim.geom import rot, wrap
from alignsim.odometry import SwerveOdometry
from alignsim.params import LoopParams, RobotParams
from alignsim.plant import SwervePlant


@dataclass
class Leg:
    goal: tuple
    end_speed: float = 0.0
    switch_radius: float = 0.15
    brake_at_switch: bool = False  # controller brakes to end_speed at the switch circle, not the waypoint


@dataclass
class Move:
    start: tuple
    v0: tuple  # field-frame (vx, vy, w) at t = 0
    legs: list


@dataclass
class BenchCfg:
    rp: RobotParams = field(default_factory=RobotParams)
    loop: LoopParams = field(default_factory=LoopParams)
    pose_src: str = "truth"  # truth | odom
    noise_xy: float = 0.0  # m, 1 sigma, added to the pose the controller sees
    noise_yaw: float = 0.0  # rad
    noise_tau: float = 0.0  # s; 0 = white per loop, else first-order Gauss-Markov
    t_end: float = 7.0
    band_xy: float = 0.02  # settle band
    band_yaw: float = math.radians(1.0)
    done_xy: float = 0.02  # Java DriveToPose.isFinished defaults
    done_yaw: float = math.radians(2.0)
    done_speed: float = 0.05


def _polyline_dist(p, pts):
    best = float("inf")
    for a, b in zip(pts[:-1], pts[1:]):
        ax, ay, bx, by = a[0], a[1], b[0], b[1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        u = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2))
        best = min(best, math.hypot(p[0] - ax - u * dx, p[1] - ay - u * dy))
    return best


def _configure(ctrl, leg):
    if hasattr(ctrl, "end_speed"):
        ctrl.end_speed = leg.end_speed
    if leg.brake_at_switch and hasattr(ctrl, "switch_radius"):
        ctrl.switch_radius = leg.switch_radius


def run_move(make_ctrl, move: Move, cfg: BenchCfg, seed: int, record=False):
    """make_ctrl() -> controller (fresh per leg). Returns a metrics dict (true-pose based)."""
    rng = np.random.default_rng(seed)
    rp, lp = cfg.rp, cfg.loop
    plant = SwervePlant(rp, move.start, move.v0, rng)
    odo = SwerveOdometry(rp.module_positions, plant.read_gyro(), plant.read_modules(), move.start)

    def snap():
        mods = plant.module_velocities()
        vxr = sum(s * math.cos(a) for s, a in mods) / 4.0
        vyr = sum(s * math.sin(a) for s, a in mods) / 4.0
        wr = 0.0
        for (s, a), (mx, my) in zip(mods, plant.mods):
            wr += (-my * s * math.cos(a) + mx * s * math.sin(a)) / (mx * mx + my * my)
        v = rot((vxr, vyr), odo.pose[2])
        return plant.pose(), odo.pose, (v[0], v[1], wr / 4.0)

    noise = [0.0, 0.0, 0.0]
    a_gm = math.exp(-lp.main_period / cfg.noise_tau) if cfg.noise_tau > 0 else 0.0

    def seen(true_pose, odom_pose):
        p = true_pose if cfg.pose_src == "truth" else odom_pose
        if cfg.noise_xy == 0.0 and cfg.noise_yaw == 0.0:
            return p
        s = math.sqrt(1 - a_gm * a_gm)
        sig = (cfg.noise_xy, cfg.noise_xy, cfg.noise_yaw)
        for i in range(3):
            noise[i] = a_gm * noise[i] + s * sig[i] * rng.normal()
        return (p[0] + noise[0], p[1] + noise[1], wrap(p[2] + noise[2]))

    legs = move.legs
    final = legs[-1].goal
    route = [move.start] + [lg.goal for lg in legs]
    leg_i = 0
    ctrl = make_ctrl()
    _configure(ctrl, legs[0])
    true_p, odom_p, vel = snap()
    ctrl.reset(seen(true_p, odom_p), vel)

    dt = lp.sim_dt
    t = 0.0
    next_odom = 0.0
    next_main = lp.main_period
    requests = []
    cur_req = tuple(move.v0)
    last_cmd = tuple(move.v0)
    last_acc = None
    pk_acc = pk_alpha = pk_jerk = 0.0
    acc_tv = 0.0
    t_done = None
    leg_switch_t = []
    # per-odometry-tick truth samples
    ts, xs, ys, ths = [], [], [], []
    wp_min = [float("inf")] * (len(legs) - 1)
    wp_speed = [0.0] * (len(legs) - 1)
    max_dev = 0.0
    trace = [] if record else None
    steps = int(cfg.t_end / dt)
    for _ in range(steps):
        if t >= next_odom - 1e-9:
            next_odom += lp.odom_period
            while requests and requests[0][0] <= t:
                cur_req = requests.pop(0)[1]
            vr = rot((cur_req[0], cur_req[1]), -odo.pose[2])
            plant.set_chassis_robot(vr[0], vr[1], cur_req[2])
            odo.update(plant.read_gyro(), plant.read_modules())
            true_p, odom_p, vel = snap()
            ts.append(t)
            xs.append(true_p[0])
            ys.append(true_p[1])
            ths.append(plant.th_unwrapped)
            for k in range(len(wp_min)):
                d = math.hypot(true_p[0] - legs[k].goal[0], true_p[1] - legs[k].goal[1])
                if d < wp_min[k]:
                    wp_min[k] = d
                    wp_speed[k] = math.hypot(plant.vx, plant.vy)
            max_dev = max(max_dev, _polyline_dist(true_p, route))
        if t >= next_main - 1e-9:
            next_main += lp.main_period + rng.normal(0, lp.main_jitter)
            pose = seen(true_p, odom_p)
            leg = legs[leg_i]
            goal = leg.goal
            # pass-through: switch to the next leg once close (Java WAYPOINT_TOLERANCE)
            if leg.end_speed > 0 and leg_i < len(legs) - 1 and \
                    math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < leg.switch_radius:
                leg_i += 1
                leg_switch_t.append(t)
                leg = legs[leg_i]
                goal = leg.goal
                ctrl = make_ctrl()
                _configure(ctrl, leg)
                ctrl.reset(pose, vel)
            cmd = ctrl.step(pose, goal, vel, lp.main_period)
            # stop-and-go: an end_speed == 0 leg that is not last ends when finished
            ex, ey = goal[0] - pose[0], goal[1] - pose[1]
            fin = (math.hypot(ex, ey) < cfg.done_xy and abs(wrap(goal[2] - pose[2])) < cfg.done_yaw
                   and math.hypot(vel[0], vel[1]) < cfg.done_speed)
            if fin and leg.end_speed == 0.0:
                if leg_i < len(legs) - 1:
                    leg_i += 1
                    leg_switch_t.append(t)
                    ctrl = make_ctrl()
                    _configure(ctrl, legs[leg_i])
                    ctrl.reset(pose, vel)
                elif t_done is None:
                    t_done = t
            ax_, ay_ = (cmd[0] - last_cmd[0]) / lp.main_period, (cmd[1] - last_cmd[1]) / lp.main_period
            pk_acc = max(pk_acc, math.hypot(ax_, ay_))
            pk_alpha = max(pk_alpha, abs(cmd[2] - last_cmd[2]) / lp.main_period)
            if last_acc is not None:
                da = math.hypot(ax_ - last_acc[0], ay_ - last_acc[1])
                pk_jerk = max(pk_jerk, da / lp.main_period)
                acc_tv += da
            last_acc = (ax_, ay_)
            last_cmd = tuple(cmd)
            requests.append((t + lp.actuation_delay, tuple(cmd)))
            if record:
                trace.append((t, true_p, pose, odom_p, goal, tuple(cmd), leg_i))
        plant.step(dt)
        t += dt

    # ---------------------------------------------------------------- metrics (truth, final goal)
    ts = np.array(ts)
    xs, ys, ths = np.array(xs), np.array(ys), np.array(ths)
    ex, ey = final[0] - xs, final[1] - ys
    dist = np.hypot(ex, ey)
    g_ref = ths[-1] + wrap(final[2] - ths[-1])
    eth = g_ref - ths
    inband = (dist < cfg.band_xy) & (np.abs(eth) < cfg.band_yaw)
    in5 = (dist < 0.05) & (np.abs(eth) < math.radians(2))

    def first(mask):
        idx = np.flatnonzero(mask)
        return float(ts[idx[0]]) if len(idx) else float("nan")

    if inband[-1]:
        out_idx = np.flatnonzero(~inband)
        t_settle = float(ts[out_idx[-1] + 1]) if len(out_idx) else 0.0
    else:
        t_settle = float("nan")
    # overshoot: how far past the goal along the final approach direction (set at 0.3 m)
    near = np.flatnonzero(dist < 0.3)
    over = 0.0
    if len(near):
        k0 = near[0]
        if dist[k0] > 1e-6:
            ux, uy = ex[k0] / dist[k0], ey[k0] / dist[k0]
            past = -(ex[k0:] * ux + ey[k0:] * uy)
            over = max(0.0, float(past.max()))
    e0 = eth[0]
    if abs(e0) > math.radians(1):
        yaw_over = max(0.0, float((-math.copysign(1.0, e0) * eth).max()))
    else:
        yaw_over = float(np.abs(eth).max())
    end = (float(dist[-1]), float(abs(eth[-1])))
    odo_err = math.hypot(odo.pose[0] - plant.x, odo.pose[1] - plant.y)
    res = {
        "t_in5": first(in5),
        "t_in2": first(inband),
        "t_settle": t_settle,
        "t_done": t_done if t_done is not None else float("nan"),
        "overshoot": over,
        "yaw_over": yaw_over,
        "err_end": end[0],
        "yaw_end": end[1],
        "slip": plant.slip_time,
        "odo_err": odo_err,
        "pk_acc": pk_acc,
        "pk_alpha": pk_alpha,
        "pk_jerk": pk_jerk,
        "acc_tv": acc_tv,
        "max_dev": max_dev,
        "wp_min": wp_min,
        "wp_speed": wp_speed,
        "end_speed": math.hypot(plant.vx, plant.vy),
        "switch_t": leg_switch_t,
    }
    if record:
        res["trace"] = trace
    return res


# ---------------------------------------------------------------- random long moves
def sample_move(rng, dist=(1.0, 6.0), dheading=(0.0, math.pi), speed=(0.0, 0.0), w0=0.0):
    """Random start at the origin area, goal `dist` away in a random direction, heading change in
    `dheading` (random sign), start speed in `speed` in a uniformly random direction."""
    th0 = rng.uniform(-math.pi, math.pi)
    d = rng.uniform(*dist)
    bearing = rng.uniform(-math.pi, math.pi)
    dh = rng.uniform(*dheading) * (1 if rng.random() < 0.5 else -1)
    sp = rng.uniform(*speed)
    vdir = rng.uniform(-math.pi, math.pi)
    start = (0.0, 0.0, th0)
    goal = (d * math.cos(bearing), d * math.sin(bearing), wrap(th0 + dh))
    v0 = (sp * math.cos(vdir), sp * math.sin(vdir), w0)
    # angle between the start velocity and the direction to the goal (0 = toward, pi = away)
    rel = abs(wrap(vdir - bearing)) if sp > 0 else 0.0
    return Move(start, v0, [Leg(goal)]), {"dist": d, "dh": abs(dh), "speed": sp, "vrel": rel}


def summarize_moves(rows, t_end):
    """Aggregate a list of run_move results into one row of numbers."""
    def arr(k):
        return np.array([r[k] for r in rows], dtype=float)

    def med(k):
        a = arr(k)
        a = a[~np.isnan(a)]
        return float(np.median(a)) if len(a) else float("nan")

    def p95(k):
        a = arr(k)
        a = a[~np.isnan(a)]
        return float(np.percentile(a, 95)) if len(a) else float("nan")

    settled = ~np.isnan(arr("t_settle"))
    return {
        "n": len(rows),
        "t_in5": med("t_in5"),
        "t_settle": med("t_settle"),
        "t_settle_p95": p95("t_settle"),
        "t_done": med("t_done"),
        "settled_%": 100 * settled.mean(),
        "over_p50_cm": 100 * med("overshoot"),
        "over_p95_cm": 100 * p95("overshoot"),
        "yaw_over_p95": math.degrees(p95("yaw_over")),
        "err_p50_cm": 100 * med("err_end"),
        "err_p95_cm": 100 * p95("err_end"),
        "jitter_cm_s": 100 * float(arr("end_speed").mean()),
        "yaw_p95": math.degrees(p95("yaw_end")),
        "slip_ms": 1000 * float(arr("slip").mean()),
        "slip_any_%": 100 * float((arr("slip") > 0.01).mean()),
        "odo_p50_cm": 100 * med("odo_err"),
        "odo_p95_cm": 100 * p95("odo_err"),
        "pk_acc": med("pk_acc"),
        "pk_acc_max": float(arr("pk_acc").max()),
        "pk_jerk": med("pk_jerk"),
        "acc_tv": med("acc_tv"),
        "dev_p95_cm": 100 * p95("max_dev"),
    }
