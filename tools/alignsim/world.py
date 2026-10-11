"""One simulated attempt: physics at 500 Hz, the CTRE odometry thread at 250 Hz, cameras at their
own frame rate with latency, and the 50 Hz robot loop running a Strategy."""

import math
from dataclasses import dataclass, field as dfield, replace
from typing import Callable

import numpy as np

from .camera import Camera
from .controllers import VectorBrake
from .field import Field
from .geom import compose, inverse, relative, rot, wrap
from .odometry import PoseEstimator, SwerveOdometry
from .params import CameraParams, FieldParams, LoopParams, RobotParams, front_cameras
from .plant import SwervePlant
from .trackers import FrameTracker, GlobalVision


# ---------------------------------------------------------------------------- configuration
@dataclass
class Errors:
    """Per-trial random errors (1 sigma). ASSUMED until measured on the robot."""

    init_xy: float = 0.03  # global pose error when the approach starts (m)
    init_yaw: float = math.radians(0.5)
    wheel_radius: float = 0.01  # fractional
    gyro_scale: float = 0.002
    cam_xy: float = 0.003  # mount position error (m)
    cam_z: float = 0.003
    cam_yaw: float = math.radians(0.4)
    cam_pitch: float = math.radians(0.4)
    cam_roll: float = math.radians(0.2)
    fx: float = 0.005  # fractional focal-length error
    center_px: float = 2.0  # principal point error (px)
    clock_bias: float = 0.002  # camera clock sync bias (s)
    pnp_yaw: float = 0.0  # extra systematic yaw in the tag solve (rad)
    teach_xy: float = 0.003  # how exactly a human parks the robot when teaching (m)
    teach_yaw: float = math.radians(0.3)


@dataclass
class Scenario:
    robot: RobotParams = dfield(default_factory=RobotParams)
    cameras: tuple = dfield(default_factory=front_cameras)
    field: FieldParams = dfield(default_factory=FieldParams)
    loop: LoopParams = dfield(default_factory=LoopParams)
    errors: Errors = dfield(default_factory=Errors)
    t_end: float = 5.0
    # start: distance from the goal along the tag normal, lateral spread, heading spread, speed
    start_dist: float = 2.8
    start_lateral: float = 0.8
    start_heading: float = math.radians(25)
    start_speed: float = 1.5
    pieces: tuple = ()


@dataclass
class Strategy:
    controller: Callable = VectorBrake
    tracker: Callable = None  # f(tag_id, goal_in_tag, scenario, cams) -> Tracker; None = global only
    vision: GlobalVision = dfield(default_factory=GlobalVision)
    approach: str = "handoff"  # handoff | global | staged | shaped
    stage_dist: float = 0.6
    stage_speed: float = 1.0
    teach: bool = False
    pos_tol: float = 0.02
    yaw_tol: float = math.radians(1.5)
    settle_speed: float = 0.05
    debounce: float = 0.06
    slow_zone: tuple = None  # (radius m, max speed m/s) near the goal
    name: str = ""


class TrackerSpec:
    """Picklable tracker factory: TrackerSpec(SomeTracker, **kwargs)(tag_id, goal_in_tag, sc, cams).

    Pass cams=True to give the tracker the believed camera mounts (TxTy6328)."""

    def __init__(self, cls, needs_cams=False, **kw):
        self.cls, self.needs_cams, self.kw = cls, needs_cams, kw

    def __call__(self, tag_id, goal_in_tag, scenario, cams):
        if self.needs_cams:
            return self.cls(tag_id, goal_in_tag, {c.cp.name: c.mount_believed for c in cams}, **self.kw)
        return self.cls(tag_id, goal_in_tag, **self.kw)


def default_tracker(**kw):
    return TrackerSpec(FrameTracker, **kw)


# ---------------------------------------------------------------------------- the world
class World:
    def __init__(self, sc: Scenario, seed: int):
        self.sc = sc
        rng = np.random.default_rng(seed)
        self.rng = rng
        e = sc.errors
        self.field = Field(sc.field, rng)
        rp = replace(sc.robot, wheel_radius_error=rng.normal(0, e.wheel_radius),
                     gyro_scale_error=rng.normal(0, e.gyro_scale))
        self.rp = rp
        cams = []
        for cp in sc.cameras:
            cp = replace(
                cp,
                mount_error=(rng.normal(0, e.cam_xy), rng.normal(0, e.cam_xy), rng.normal(0, e.cam_z),
                             rng.normal(0, e.cam_roll), rng.normal(0, e.cam_pitch), rng.normal(0, e.cam_yaw)),
                fx_error=rng.normal(0, e.fx),
                center_error=(rng.normal(0, e.center_px), rng.normal(0, e.center_px)),
                clock_bias=rng.normal(0, e.clock_bias),
                pnp_yaw_bias=rng.normal(0, e.pnp_yaw) if e.pnp_yaw else 0.0)
            cams.append(Camera(cp, rng))
        self.cams = cams
        # where the robot should truly end up: nominal goal on the REAL element
        self.true_goal = self.field.goal(true=True)

    # ------------------------------------------------------------------ teaching
    def teach(self, strategy_goal):
        """Robot parked by hand at the true goal; average what the cameras say robot-in-tag is."""
        e = self.sc.errors
        rng = self.rng
        park = compose(self.true_goal, (rng.normal(0, e.teach_xy), rng.normal(0, e.teach_xy),
                                        rng.normal(0, e.teach_yaw)))
        samples = []
        for k in range(90):
            for cam in self.cams:
                fr = cam.capture(k / 60.0, park, (0, 0, 0), self.field, park[2])
                for f in fr.fiducials:
                    if f.id == self.field.TARGET_ID:
                        samples.append(inverse(f.tag_in_robot))
        if not samples:
            return strategy_goal
        a = np.array(samples)
        return (float(a[:, 0].mean()), float(a[:, 1].mean()),
                math.atan2(np.sin(a[:, 2]).mean(), np.cos(a[:, 2]).mean()))

    # ------------------------------------------------------------------ run
    def run(self, st: Strategy, record=False):
        sc, rng, rp, fld = self.sc, self.rng, self.rp, self.field
        lp = sc.loop
        goal_in_tag = sc.field.goal_in_tag
        if st.teach:
            goal_in_tag = self.teach(goal_in_tag)

        # start pose: out along the (true) tag normal, random lateral/heading, moving at the goal
        tg = self.true_goal
        lat = rng.uniform(-sc.start_lateral, sc.start_lateral)
        start = compose(tg, (-sc.start_dist, lat, rng.uniform(-sc.start_heading, sc.start_heading)))
        to_goal = (tg[0] - start[0], tg[1] - start[1])
        n = math.hypot(*to_goal)
        v0 = (to_goal[0] / n * sc.start_speed, to_goal[1] / n * sc.start_speed, 0.0)
        plant = SwervePlant(rp, start, v0, rng)
        e = sc.errors
        believed = (start[0] + rng.normal(0, e.init_xy), start[1] + rng.normal(0, e.init_xy),
                    wrap(start[2] + rng.normal(0, e.init_yaw)))
        odo = SwerveOdometry(rp.module_positions, plant.read_gyro(), plant.read_modules(), believed)
        est = PoseEstimator(odo, state_std=(0.1, 0.1, 0.1))
        est.update(0.0, plant.read_gyro(), plant.read_modules())

        ctrl = st.controller()
        tracker = st.tracker(fld.TARGET_ID, goal_in_tag, sc, self.cams) if st.tracker else None
        map_goal = compose(fld.target.map_pose, goal_in_tag)

        dt = lp.sim_dt
        t = 0.0
        next_odom = 0.0
        next_main = lp.main_period
        pending = []  # frames in flight
        requests = []  # (apply_time, (vx, vy, w) field) or None for neutral
        cur_req = (v0[0], v0[1], 0.0)
        snap = self._snap(t, est, odo, plant)
        ctrl.reset(snap["pose"], snap["vel"])
        staged = st.approach != "staged"
        done_t = None
        hold_since = None
        lock_t = None
        res = {"err_done": None}
        trace = [] if record else None
        peak_acc = 0.0
        last_cmd = (v0[0], v0[1])
        steps = int(sc.t_end / dt)
        for _ in range(steps):
            # cameras
            for cam in self.cams:
                fr = cam.maybe_capture(t, plant.pose(), plant.field_velocity(), fld, est.estimate[2], sc.pieces)
                if fr is not None:
                    pending.append(fr)
            # odometry thread: apply the newest request, then read sensors
            if t >= next_odom - 1e-9:
                next_odom += lp.odom_period
                while requests and requests[0][0] <= t:
                    cur_req = requests.pop(0)[1]
                if cur_req is None:
                    plant.set_neutral()
                else:
                    vr = rot((cur_req[0], cur_req[1]), -est.estimate[2])
                    plant.set_chassis_robot(vr[0], vr[1], cur_req[2])
                est.update(t, plant.read_gyro(), plant.read_modules())
                snap = self._snap(t, est, odo, plant)
            # robot loop
            if t >= next_main - 1e-9:
                next_main += lp.main_period + rng.normal(0, lp.main_jitter)
                arrived = [f for f in pending if f.t_arrive <= t]
                pending = [f for f in pending if f.t_arrive > t]
                st.vision.apply(arrived, est, fld, snap["vel"][2])
                snap = self._snap(t, est, odo, plant)
                if tracker is not None:
                    tracker.update(t, snap, arrived, est, fld)
                    if lock_t is None and tracker.locked():
                        lock_t = t
                # goal for this loop
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
                # declared finished? (measured, in the robot's own belief)
                if done_t is None and (tracker is None or tracker.locked() or st.approach == "global"):
                    err = relative(goal, snap["pose"])
                    speed = math.hypot(snap["vel"][0], snap["vel"][1])
                    if math.hypot(err[0], err[1]) < st.pos_tol and abs(err[2]) < st.yaw_tol \
                            and speed < st.settle_speed:
                        hold_since = hold_since if hold_since is not None else t
                        if t - hold_since >= st.debounce:
                            done_t = t
                            res["err_done"] = relative(self.true_goal, plant.pose())
                    else:
                        hold_since = None
                if record:
                    trace.append((t, plant.pose(), snap["pose"], snap["odom"], goal, cmd,
                                  tracker.locked() if tracker else False))
            plant.step(dt)
            t += dt

        res.update(self._metrics(plant, est, odo, start, done_t, lock_t, peak_acc))
        if record:
            res["trace"] = trace
        return res

    @staticmethod
    def _snap(t, est, odo, plant):
        # measured chassis velocity from module states (forward kinematics), field frame
        mods = plant.module_velocities()
        vxr = sum(s * math.cos(a) for s, a in mods) / 4.0
        vyr = sum(s * math.sin(a) for s, a in mods) / 4.0
        wr = 0.0
        for (s, a), (mx, my) in zip(mods, plant.mods):
            wr += (-my * s * math.cos(a) + mx * s * math.sin(a)) / (mx * mx + my * my)
        wr /= 4.0
        v = rot((vxr, vyr), est.estimate[2])
        return {"t": t, "pose": est.estimate, "odom": odo.pose, "vel": (v[0], v[1], wr)}

    def _metrics(self, plant, est, odo, start, done_t, lock_t, peak_acc):
        end = relative(self.true_goal, plant.pose())
        speed = math.hypot(plant.vx, plant.vy)
        true_move = relative(start, plant.pose())
        return {
            "done": done_t is not None,
            "t_done": done_t,
            "t_lock": lock_t,
            "err_end": end,
            "end_speed": speed,
            "slip_time": plant.slip_time,
            "peak_cmd_accel": peak_acc,
            "global_err_end": math.hypot(est.estimate[0] - plant.x, est.estimate[1] - plant.y),
        }


def _shape_6328(robot, goal, face_len=0.8):
    """6328 getDriveTarget: slide the target back along the normal / sideways from where we are."""
    off = relative(goal, robot)
    shift_x = max(0.0, min(1.0, abs(off[1]) / (2 * face_len) + (abs(off[0]) - 0.3) / (4 * face_len)))
    # goal frame: +x is the way the robot faces at the goal (toward the tag), so "out" is -x
    shift_y = max(0.0, min(1.0, 0.0 if abs(off[1]) <= 0.2 else -off[0] / face_len))
    return compose(goal, (-shift_x * 1.0, math.copysign(shift_y * 1.2, off[1]), 0.0))


def run_trial(args):
    """Picklable entry point for multiprocessing: (scenario, strategy, seed) -> metrics dict."""
    sc, st, seed = args
    w = World(sc, seed)
    return w.run(st)
