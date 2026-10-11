"""Game-piece pickup world for alignsim.

Adds what the core World doesn't have: a floor-looking detector camera with bounding-box clipping,
static or rolling pieces, an intake contact check, and a 50 Hz strategy hook. Thread timing (500 Hz
physics, 250 Hz odometry, camera frames with latency, 50 Hz loop) is the same as alignsim.world.

No AprilTags are simulated, so the fused pose equals the wheel-odometry pose; the Java code
re-anchors its goal onto the fused pose, which cancels vision jumps, so this changes nothing.
"""

import math
import os
import sys
from dataclasses import dataclass, field as dfield, replace

import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))

from alignsim.camera import Camera  # noqa: E402
from alignsim.geom import relative, rot  # noqa: E402
from alignsim.odometry import PoseEstimator, SwerveOdometry  # noqa: E402
from alignsim.params import CameraParams  # noqa: E402
from alignsim.plant import SwervePlant  # noqa: E402
from alignsim.world import Errors, Scenario, World  # noqa: E402


def piece_camera(x=0.25, y=0.0, z=0.50, pitch_down_deg=25.0, yaw_deg=0.0, fps=30.0, pipeline=0.025):
    """Detector Limelight over the intake, pitched DOWN (positive pitch = down in NWU/WPILib)."""
    return CameraParams(name="ll-piece", mount=(x, y, z, 0.0, math.radians(pitch_down_deg), math.radians(yaw_deg)),
                        fps=fps, pipeline_latency=pipeline, pipeline_jitter=0.004)


class PieceCamera(Camera):
    """Core Camera with a bounding-box detector: the box is clipped by the image edge (its center
    moves inward), boxes less than `min_visible` in frame are dropped, and frames can miss."""

    def __init__(self, cp, rng, piece_radius=0.07, bbox=True, min_visible=0.4, p_miss=0.0):
        super().__init__(cp, rng)
        self.r, self.bbox, self.min_visible, self.p_miss = piece_radius, bbox, min_visible, p_miss

    def _detect_pieces(self, frame, T_cam_field, pieces):
        cp = self.cp
        Kb = self.K_believed
        for (x, y, h) in pieces:
            p = T_cam_field @ np.array([x, y, h, 1.0])
            img = self._project(p[:3].reshape(1, 3))
            if img is None:
                continue
            u, v = img[0]
            if self.bbox:
                if p[0] > 5.0:
                    continue
                rho = self.K_true[0, 0] * self.r / p[0]  # apparent radius (px)
                u0, u1 = max(0.0, u - rho), min(float(cp.width), u + rho)
                v0, v1 = max(0.0, v - rho), min(float(cp.height), v + rho)
                if u1 <= u0 or v1 <= v0 or (u1 - u0) * (v1 - v0) < self.min_visible * 4 * rho * rho:
                    continue
                u, v = 0.5 * (u0 + u1), 0.5 * (v0 + v1)
            elif not (0 < u < cp.width and 0 < v < cp.height) or p[0] > 5.0:
                continue
            if self.p_miss and self.rng.random() < self.p_miss:
                continue
            u += self.rng.normal(0, cp.detector_noise)
            v += self.rng.normal(0, cp.detector_noise)
            tx = math.degrees(math.atan((u - Kb[0, 2]) / Kb[0, 0]))
            ty = math.degrees(math.atan((Kb[1, 2] - v) / Kb[1, 1]))
            frame.pieces.append((tx, ty))


@dataclass
class PieceScenario(Scenario):
    cameras: tuple = dfield(default_factory=lambda: (piece_camera(),))
    errors: Errors = dfield(default_factory=lambda: Errors(cam_pitch=0.0, clock_bias=0.0))
    t_end: float = 4.0
    # calibration / timing stresses (pitch and clock bias get a random sign per trial)
    pitch_bias_deg: float = 0.0
    pitch_sigma_deg: float = 0.3
    clock_bias_s: float = 0.002
    detector_noise: float = 2.0
    p_miss: float = 0.0
    bbox: bool = True
    # piece: center height the code assumes, true height range, horizontal radius
    h_assumed: float = 0.10
    h_true: tuple = (0.10, 0.10)
    h_sigma: float = 0.01  # bbox-center vs real-center height scatter
    piece_radius: float = 0.07
    piece_speed: tuple = (0.0, 0.0)  # rolling speed range (m/s), random direction
    piece_decel: float = 0.15  # rolling friction (m/s^2)
    second_piece: tuple = None  # (min, max) separation of a second piece (m)
    # robot start: piece range/bearing from the robot center, initial speed and its direction spread
    range0: tuple = (1.5, 3.5)
    bearing0_deg: float = 20.0
    speed0: tuple = (0.0, 1.5)
    vel_dir_deg: float = 60.0
    # odometry slip: sideways shove (N, s) once the bumper is within [d0, d1] m of the piece
    shove: tuple = None  # (force, duration, d0, d1)
    mu_scale: float = 1.0
    # intake: mouth on the front bumper face, robot is a square of half-size `half`
    mouth_x: float = 0.45
    half: float = 0.45
    intake_w: float = 0.40
    v_rel_max: float = 2.5


class Piece:
    __slots__ = ("x", "y", "h", "vx", "vy", "alive")

    def __init__(self, x, y, h, vx=0.0, vy=0.0):
        self.x, self.y, self.h, self.vx, self.vy, self.alive = x, y, h, vx, vy, True

    def step(self, dt, decel):
        s = math.hypot(self.vx, self.vy)
        if s > 1e-9:
            k = max(0.0, s - decel * dt) / s
            self.vx *= k
            self.vy *= k
            self.x += self.vx * dt
            self.y += self.vy * dt


class PieceWorld(World):
    def __init__(self, sc: PieceScenario, seed: int):
        super().__init__(sc, seed)
        rng = self.rng
        self.field.tags = []  # floor camera; no tag pipeline (see module doc)
        self.rp = replace(self.rp, mu_static=self.rp.mu_static * sc.mu_scale,
                          mu_kinetic=self.rp.mu_kinetic * sc.mu_scale)
        cams = []
        for cam in self.cams:
            cp = cam.cp
            me = list(cp.mount_error)
            sign = 1.0 if rng.random() < 0.5 else -1.0
            me[4] = math.radians(sign * sc.pitch_bias_deg + rng.normal(0, sc.pitch_sigma_deg))
            cb = (1.0 if rng.random() < 0.5 else -1.0) * sc.clock_bias_s
            cp = replace(cp, mount_error=tuple(me), clock_bias=cb, detector_noise=sc.detector_noise)
            cams.append(PieceCamera(cp, rng, sc.piece_radius, sc.bbox, p_miss=sc.p_miss))
        self.cams = cams

    def ctx(self):
        sc = self.sc
        return {"mounts": {c.cp.name: c.mount_believed for c in self.cams},
                "cps": {c.cp.name: c.cp for c in self.cams},
                "h": sc.h_assumed, "r": sc.piece_radius, "mouth": sc.mouth_x, "half": sc.half,
                "dt": sc.loop.main_period}

    # ------------------------------------------------------------------ run
    def run(self, st, record=False):
        sc, rng, rp = self.sc, self.rng, self.rp
        lp = sc.loop
        heading = rng.uniform(-math.pi, math.pi)
        start = (4.0, 4.0, heading)
        R = rng.uniform(*sc.range0)
        b = heading + math.radians(rng.uniform(-sc.bearing0_deg, sc.bearing0_deg))
        h = rng.uniform(*sc.h_true) + rng.normal(0, sc.h_sigma)
        ps = rng.uniform(*sc.piece_speed)
        pd = rng.uniform(-math.pi, math.pi)
        pieces = [Piece(start[0] + R * math.cos(b), start[1] + R * math.sin(b), h,
                        ps * math.cos(pd), ps * math.sin(pd))]
        if sc.second_piece:
            sep = rng.uniform(*sc.second_piece)
            a = rng.uniform(-math.pi, math.pi)
            pieces.append(Piece(pieces[0].x + sep * math.cos(a), pieces[0].y + sep * math.sin(a), h))
        sp0 = rng.uniform(*sc.speed0)
        vdir = b + math.radians(rng.uniform(-sc.vel_dir_deg, sc.vel_dir_deg))
        v0 = (sp0 * math.cos(vdir), sp0 * math.sin(vdir), 0.0)
        shove_d = rng.uniform(sc.shove[2], sc.shove[3]) if sc.shove else None
        shove_sign = 1.0 if rng.random() < 0.5 else -1.0

        plant = SwervePlant(rp, start, v0, rng)
        odo = SwerveOdometry(rp.module_positions, plant.read_gyro(), plant.read_modules(), start)
        est = PoseEstimator(odo, state_std=(0.1, 0.1, 0.1))
        est.update(0.0, plant.read_gyro(), plant.read_modules())

        dt = lp.sim_dt
        t = 0.0
        next_odom = 0.0
        next_main = lp.main_period
        pending, requests = [], []
        cur_req = (v0[0], v0[1], 0.0)
        snap = self._snap(t, est, odo, plant)
        st.reset(self.ctx(), snap)
        trace = [] if record else None
        out = {"outcome": "timeout", "t": None, "miss": None, "v_rel": None, "est_err": None,
               "t_lock": None}
        lim = sc.half + sc.piece_radius
        steps = int(sc.t_end / dt)
        for _ in range(steps):
            alive = [p for p in pieces if p.alive]
            if not alive:
                break
            for cam in self.cams:
                fr = cam.maybe_capture(t, plant.pose(), plant.field_velocity(), self.field, est.estimate[2],
                                       [(p.x, p.y, p.h) for p in alive])
                if fr is not None:
                    pending.append(fr)
            if t >= next_odom - 1e-9:
                next_odom += lp.odom_period
                while requests and requests[0][0] <= t:
                    cur_req = requests.pop(0)[1]
                vr = rot((cur_req[0], cur_req[1]), -est.estimate[2])
                plant.set_chassis_robot(vr[0], vr[1], cur_req[2])
                est.update(t, plant.read_gyro(), plant.read_modules())
                snap = self._snap(t, est, odo, plant)
            if t >= next_main - 1e-9:
                next_main += lp.main_period + rng.normal(0, lp.main_jitter)
                arrived = [f for f in pending if f.t_arrive <= t]
                pending = [f for f in pending if f.t_arrive > t]
                snap = self._snap(t, est, odo, plant)
                cmd = st.step(t, snap, arrived, est)
                requests.append((t + lp.actuation_delay, cmd))
                if out["t_lock"] is None and st.locked():
                    out["t_lock"] = t
                if shove_d is not None and math.hypot(pieces[0].x - plant.x, pieces[0].y - plant.y) \
                        - sc.mouth_x < shove_d:
                    a = plant.th + math.pi / 2
                    plant.push(shove_sign * sc.shove[0] * math.cos(a), shove_sign * sc.shove[0] * math.sin(a),
                               0.0, sc.shove[1])
                    shove_d = None
                if record:
                    trace.append((t, plant.pose(), snap["odom"], [(p.x, p.y) for p in pieces if p.alive],
                                  _est_field(st, snap, plant), cmd))
            plant.step(dt)
            for p in alive:
                p.step(dt, sc.piece_decel)
            t += dt
            # contact: piece center enters the bumper square grown by the piece radius
            pose = plant.pose()
            for p in alive:
                rel = relative(pose, (p.x, p.y, 0.0))
                if abs(rel[0]) <= lim and abs(rel[1]) <= lim:
                    w = plant.w
                    dx, dy = p.x - plant.x, p.y - plant.y
                    vrx, vry = p.vx - (plant.vx - w * dy), p.vy - (plant.vy + w * dx)
                    v_rel = math.hypot(vrx, vry)
                    mouth = rel[0] > sc.mouth_x and abs(rel[1]) <= sc.intake_w / 2
                    if mouth:
                        out.update(outcome="capture" if v_rel <= sc.v_rel_max else "too_fast", t=t,
                                   miss=abs(rel[1]), v_rel=v_rel, est_err=_est_err(st, snap, pose, p))
                    else:
                        p.alive = False  # pushed away
                        if out["miss"] is None:
                            out.update(miss=abs(rel[1]), v_rel=v_rel, est_err=_est_err(st, snap, pose, p))
                        out["outcome"] = "bump"
                    break
            if out["outcome"] in ("capture", "too_fast"):
                break
        if out["outcome"] == "timeout":
            pose = plant.pose()
            dmin = min(math.hypot(*relative(pose, (p.x, p.y, 0.0))[:2]) - sc.mouth_x for p in pieces)
            out["miss"] = max(0.0, dmin)
        out["success"] = out["outcome"] == "capture"
        out["slip_time"] = plant.slip_time
        out["start_speed"] = sp0
        if record:
            out["trace"] = trace
        return out


def _est_field(st, snap, plant):
    e = st.estimate()
    if e is None:
        return None
    rel = relative(snap["odom"], (e[0], e[1], 0.0))  # robot-relative belief
    c, s = math.cos(plant.th), math.sin(plant.th)
    return (plant.x + c * rel[0] - s * rel[1], plant.y + s * rel[0] + c * rel[1])


def _est_err(st, snap, pose, p):
    """Belief minus truth in the robot frame: (along x, along y) at contact."""
    e = st.estimate()
    if e is None:
        return None
    b = relative(snap["odom"], (e[0], e[1], 0.0))
    tr = relative(pose, (p.x, p.y, 0.0))
    return (b[0] - tr[0], b[1] - tr[1])


def run_piece_trial(args):
    sc, factory, seed = args
    return PieceWorld(sc, seed).run(factory())


def run_set(sc, factory, n, seed0=0):
    return [run_piece_trial((sc, factory, seed0 + i)) for i in range(n)]


def summarize(rows):
    n = len(rows)
    caps = [r for r in rows if r["success"]]
    contacts = [r for r in rows if r["outcome"] in ("capture", "too_fast", "bump")]
    miss = np.array([r["miss"] for r in contacts]) if contacts else np.zeros(0)
    tt = np.array([r["t"] for r in caps]) if caps else np.zeros(0)
    vr = np.array([r["v_rel"] for r in contacts]) if contacts else np.zeros(0)
    cnt = {k: sum(1 for r in rows if r["outcome"] == k) for k in ("capture", "too_fast", "bump", "timeout")}
    return {
        "n": n,
        "success_%": 100.0 * len(caps) / n,
        "too_fast_%": 100.0 * cnt["too_fast"] / n,
        "bump_%": 100.0 * cnt["bump"] / n,
        "timeout_%": 100.0 * cnt["timeout"] / n,
        "t_med": float(np.median(tt)) if len(tt) else float("nan"),
        "t_p90": float(np.percentile(tt, 90)) if len(tt) else float("nan"),
        "miss_med_cm": float(np.median(miss) * 100) if len(miss) else float("nan"),
        "miss_p95_cm": float(np.percentile(miss, 95) * 100) if len(miss) else float("nan"),
        "v_rel_med": float(np.median(vr)) if len(vr) else float("nan"),
    }
