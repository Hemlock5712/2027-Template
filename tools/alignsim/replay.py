"""Replay robot logs through different tag trackers offline.

The robot's TagTracker logs, every loop, one array entry per camera frame of the target tag
(AdvantageKit puts them under /RealOutputs/TagTracker/<id>/):
    TagInRobot      Pose2d[]  tag pose in the robot frame from the camera, BEFORE the camera correction
    RobotAtCapture  Pose2d[]  wheel-odometry pose at the frame's capture time
    FrameAgeSeconds double[]  capture -> now
    Ambiguity       double[]  per-frame pose ambiguity      (optional)
    Distance        double[]  camera -> tag distance (m)    (optional; derived if missing)
    Camera          double[]  camera index                  (optional)
    Approach        int       counts tracker resets; the replay resets wherever it changes (optional)
plus Drivetrain/OdometryPose. That is everything a tracker needs, so any variant can be re-run.

Usage (from tools/):
    python3 -m alignsim.replay LOG.wpilog --tag 1            # compare tracker variants
    python3 -m alignsim.replay LOG.wpilog --corr 0:0.004,-0.002,0.6 1:0,0.003,-0.4
                                       # camera index: x m, y m, yaw deg from CameraCorrections.java
    python3 -m alignsim.replay --synthetic out.wpilog          # write a sim run in the same format

With --synthetic the sim knows the truth, so the replay also reports each variant's error.
"""

import argparse
import math
import sys

import numpy as np

from .geom import compose, relative, wrap
from .wpilog import WpilogWriter, read


class Obs:
    __slots__ = ("t", "tag_in_robot", "robot_at_capture", "age", "amb", "dist", "cam")

    def __init__(self, t, tir, rac, age, amb, dist, cam):
        self.t, self.tag_in_robot, self.robot_at_capture = t, tir, rac
        self.age, self.amb, self.dist, self.cam = age, amb, dist, cam


def load(path, tag):
    pre = f"TagTracker/{tag}/"
    keys = [pre + k for k in ("TagInRobot", "RobotAtCapture", "FrameAgeSeconds", "Ambiguity", "Distance",
                              "Camera", "Approach")] + ["Drivetrain/OdometryPose", "Drivetrain/Pose"]
    d = read(path, keys)

    def series(k):
        for name, v in d.items():
            if name.endswith(k):
                return dict(v)
        return {}

    tir, rac, age = series(pre + "TagInRobot"), series(pre + "RobotAtCapture"), series(pre + "FrameAgeSeconds")
    amb, dist, cam = series(pre + "Ambiguity"), series(pre + "Distance"), series(pre + "Camera")
    odom = sorted(series("Drivetrain/OdometryPose").items())
    resets, last = [], None
    for t, v in sorted(series(pre + "Approach").items()):
        if v != last:
            resets.append((t, None))
            last = v
    obs = []
    for t in sorted(tir):
        poses = tir[t]
        for i, p in enumerate(poses):
            r = rac.get(t, [None] * len(poses))[i]
            if r is None:
                continue
            a = age.get(t, [0.0] * len(poses))
            am = amb.get(t, [0.0] * len(poses))
            ds = dist.get(t, [math.hypot(p[0], p[1])] * len(poses))
            cm = cam.get(t, [0.0] * len(poses))
            obs.append(Obs(t, p, r, a[i] if i < len(a) else 0.0, am[i] if i < len(am) else 0.0,
                           ds[i] if i < len(ds) else math.hypot(p[0], p[1]), int(cm[i]) if i < len(cm) else 0))
    return obs, odom, resets


# ---------------------------------------------------------------------------- tracker variants
class ReplayKF:
    """TagTracker.java as shipped (per-axis Kalman, jump gate, ambiguity cut, per-camera correction),
    fed directly from logged inputs."""

    def __init__(self, xy_floor=0.005, xy_k=0.01, yaw_floor=math.radians(0.5), yaw_k=math.radians(1.5),
                 drift=(0.002, math.radians(0.1)), gate=0.25, max_amb=0.3, yaw_amb=0.0, max_dist=None,
                 cam_corr=None):
        self.p = dict(xy_floor=xy_floor, xy_k=xy_k, yaw_floor=yaw_floor, yaw_k=yaw_k, drift=drift, gate=gate,
                      max_amb=max_amb, yaw_amb=yaw_amb, max_dist=max_dist)
        self.cam_corr = cam_corr or {}
        self.est, self.var, self.jumps, self.n = None, [0, 0, 0], 0, 0

    def update(self, o):
        p = self.p
        if o.amb > p["max_amb"] or (p["max_dist"] is not None and o.dist > p["max_dist"]):
            return
        tir = o.tag_in_robot
        if o.cam in self.cam_corr:
            tir = compose(self.cam_corr[o.cam], tir)
        m = compose(o.robot_at_capture, tir)
        xy = p["xy_floor"] + p["xy_k"] * o.dist ** 2
        yw = p["yaw_floor"] + p["yaw_k"] * o.dist + p["yaw_amb"] * o.amb
        r = (xy * xy, xy * xy, yw * yw)
        if self.est is None:
            self.est, self.var, self.n = m, list(r), 1
            return
        dx, dy, dth = m[0] - self.est[0], m[1] - self.est[1], wrap(m[2] - self.est[2])
        if math.hypot(dx, dy) > p["gate"]:
            self.jumps += 1
            if self.jumps >= 3:
                self.est, self.var, self.n, self.jumps = m, list(r), 1, 0
            return
        self.jumps = 0
        q = (p["drift"][0] ** 2, p["drift"][0] ** 2, p["drift"][1] ** 2)
        k = []
        for i in range(3):
            self.var[i] += q[i]
            k.append(self.var[i] / (self.var[i] + r[i]))
            self.var[i] *= 1 - k[i]
        self.est = (self.est[0] + k[0] * dx, self.est[1] + k[1] * dy, wrap(self.est[2] + k[2] * dth))
        self.n += 1

    def reset(self, prior=None):
        self.est, self.var, self.jumps, self.n = None, [0, 0, 0], 0, 0


class ReplayGeo:
    """The sim's geo filter (studies/tag/geotracker.py): x/y Kalman with depth/side noise along the camera
    ray, yaw with an ambiguity term and a 4-sigma gate, lock on sigma, per-camera correction. Tied with
    the per-axis filter in the sim once cameras are calibrated; kept here to compare on real logs."""

    def __init__(self, floor=0.004, depth_k=0.006, side_k=0.003, yaw_floor=math.radians(0.3),
                 yaw_k=math.radians(0.45), yaw_amb=math.radians(9.0), q_xy=0.0015, q_yaw=math.radians(0.05),
                 prior_xy=0.08, prior_yaw=math.radians(1.5), use_prior=False, gate=0.25, yaw_gate=4.0,
                 lock_frames=3, lock_sigma=0.03, max_dist=None, cam_corr=None):
        self.p = dict(floor=floor, depth_k=depth_k, side_k=side_k, yaw_floor=yaw_floor, yaw_k=yaw_k,
                      yaw_amb=yaw_amb, q_xy=q_xy, q_yaw=q_yaw, prior_xy=prior_xy, prior_yaw=prior_yaw,
                      use_prior=use_prior, gate=gate, yaw_gate=yaw_gate, lock_frames=lock_frames,
                      lock_sigma=lock_sigma, max_dist=max_dist)
        self.cam_corr = cam_corr or {}
        self.reset()

    def reset(self, prior=None):
        p = self.p
        self.est, self.Pxy, self.Pyaw, self.n, self.jumps = None, None, None, 0, 0
        if prior is not None and p["use_prior"]:
            self.est = tuple(prior)
            self.Pxy = np.eye(2) * p["prior_xy"] ** 2
            self.Pyaw = p["prior_yaw"] ** 2

    def locked(self):
        return (self.est is not None and self.n >= self.p["lock_frames"]
                and math.sqrt(max(self.Pxy[0, 0], self.Pxy[1, 1])) < self.p["lock_sigma"])

    def update(self, o):
        p = self.p
        if p["max_dist"] is not None and o.dist > p["max_dist"]:
            return
        tir = o.tag_in_robot
        if o.cam in self.cam_corr:
            tir = compose(self.cam_corr[o.cam], tir)
        r = o.robot_at_capture
        m = compose(r, tir)
        depth = p["floor"] + p["depth_k"] * o.dist ** 2
        side = p["floor"] + p["side_k"] * o.dist
        ray = math.atan2(m[1] - r[1], m[0] - r[0])
        rot = np.array([[math.cos(ray), -math.sin(ray)], [math.sin(ray), math.cos(ray)]])
        Rxy = rot @ np.diag([depth ** 2, side ** 2]) @ rot.T
        Ryaw = (p["yaw_floor"] + p["yaw_k"] * o.dist + p["yaw_amb"] * o.amb) ** 2
        if self.est is None:
            self.est, self.Pxy, self.Pyaw, self.n, self.jumps = m, Rxy, Ryaw, 1, 0
            return
        self.Pxy = self.Pxy + np.eye(2) * p["q_xy"] ** 2
        self.Pyaw += p["q_yaw"] ** 2
        innov = np.array([m[0] - self.est[0], m[1] - self.est[1]])
        dyaw = wrap(m[2] - self.est[2])
        if math.hypot(*innov) > p["gate"] and self.n >= p["lock_frames"]:
            self.jumps += 1
            if self.jumps >= 3:
                self.est, self.Pxy, self.Pyaw, self.n, self.jumps = m, Rxy, Ryaw, 1, 0
            return
        self.jumps = 0
        K = self.Pxy @ np.linalg.inv(self.Pxy + Rxy)
        step = K @ innov
        self.Pxy = (np.eye(2) - K) @ self.Pxy
        ystep = 0.0
        if abs(dyaw) < p["yaw_gate"] * math.sqrt(self.Pyaw + Ryaw):
            k = self.Pyaw / (self.Pyaw + Ryaw)
            ystep = k * dyaw
            self.Pyaw *= 1 - k
        self.est = (self.est[0] + step[0], self.est[1] + step[1], wrap(self.est[2] + ystep))
        self.n += 1


def variants(cam_corr=None):
    c = cam_corr or {}
    return {
        "TagTracker.java (as shipped)": lambda: ReplayKF(cam_corr=c),
        "no camera correction": lambda: ReplayKF(),
        "keep ambiguous frames": lambda: ReplayKF(cam_corr=c, max_amb=1.0),
        "keep ambiguous frames, inflate their yaw noise": lambda: ReplayKF(cam_corr=c, max_amb=1.0,
                                                                          yaw_amb=math.radians(9.0)),
        "near frames only (< 1.5 m)": lambda: ReplayKF(cam_corr=c, max_dist=1.5),
        "more forgetting (drift x4)": lambda: ReplayKF(cam_corr=c, drift=(0.008, math.radians(0.4))),
        "geo filter (sim alternative)": lambda: ReplayGeo(cam_corr=c),
    }


def replay(obs, resets=(), variant_makers=None):
    """-> {variant: list of (t, estimate)} after each logged loop. Resets at each (t, prior) in resets."""
    out = {}
    for name, make in (variant_makers or variants()).items():
        tr = make()
        hist = []
        last_t = None
        gi = 0
        for o in obs:
            while gi < len(resets) and resets[gi][0] <= o.t:
                tr.reset(resets[gi][1])
                gi += 1
            tr.update(o)
            if o.t != last_t and tr.est is not None:
                hist.append((o.t, tr.est))
                last_t = o.t
        out[name] = hist
    return out


def report(results, truth=None):
    lines = ["| variant | frames | final tag x, y (m), yaw (deg) | spread last 0.5 s (mm / deg) |"
             + (" error vs truth (mm / deg) |" if truth else ""),
             "|---|---|---|---|" + ("---|" if truth else "")]
    for name, hist in results.items():
        if not hist:
            lines.append(f"| {name} | 0 | - | - |" + (" - |" if truth else ""))
            continue
        t_end = hist[-1][0]
        last = np.array([e for t, e in hist if t >= t_end - 0.5])
        est = hist[-1][1]
        spread = (float(np.hypot(last[:, 0].std(), last[:, 1].std()) * 1000), float(np.degrees(last[:, 2].std())))
        row = (f"| {name} | {len(hist)} | {est[0]:.3f}, {est[1]:.3f}, {math.degrees(est[2]):.2f} | "
               f"{spread[0]:.1f} / {spread[1]:.2f} |")
        if truth:
            row += f" {math.hypot(est[0] - truth[0], est[1] - truth[1]) * 1000:.1f} / " \
                   f"{abs(math.degrees(wrap(est[2] - truth[2]))):.2f} |"
        lines.append(row)
    return "\n".join(lines)


# ---------------------------------------------------------------------------- synthetic logs
def write_synthetic(path, seed=3):
    """Run one sim approach and log exactly what TagTracker.java logs. Returns the true tag pose in the
    odometry frame at the end of the run (for scoring the replay)."""
    from .trackers import FrameTracker
    from .world import Scenario, Strategy, TrackerSpec, World

    log = WpilogWriter(path)
    truth = {}

    class LoggingTracker(FrameTracker):
        def update(self, t, snap, frames, est, field):
            log.double(f"/RealOutputs/TagTracker/{self.tag_id}/Approach", t, 1.0)
            tir, rac, age, amb, dist, cam = [], [], [], [], [], []
            names = sorted({fr.camera for fr in frames})
            for fr, f in self._sightings(t, frames):
                r = est.odom_buffer.sample(fr.t_capture_est)
                if r is None:
                    continue
                tir.append(f.tag_in_robot)
                rac.append(r)
                age.append(t - fr.t_capture_est)
                amb.append(f.ambiguity)
                dist.append(f.distance)
                cam.append(float(names.index(fr.camera)))
            pre = f"/RealOutputs/TagTracker/{self.tag_id}/"
            log.poses(pre + "TagInRobot", t, tir)
            log.poses(pre + "RobotAtCapture", t, rac)
            log.doubles(pre + "FrameAgeSeconds", t, age)
            log.doubles(pre + "Ambiguity", t, amb)
            log.doubles(pre + "Distance", t, dist)
            log.doubles(pre + "Camera", t, cam)
            log.pose("/RealOutputs/Drivetrain/OdometryPose", t, snap["odom"])
            log.pose("/RealOutputs/Drivetrain/Pose", t, snap["pose"])
            # truth: where the real tag is, expressed in the odometry frame right now
            truth["tag"] = compose(snap["odom"], relative(snap["true"], field.target.true_pose)) \
                if "true" in snap else None
            super().update(t, snap, frames, est, field)

    w = World(Scenario(), seed)
    orig_snap = w._snap

    def snap_with_truth(t, est, odo, plant):
        s = orig_snap(t, est, odo, plant)
        s["true"] = plant.pose()
        return s
    w._snap = snap_with_truth
    w.run(Strategy(tracker=TrackerSpec(LoggingTracker, max_amb=1.0)))
    log.close()
    return truth.get("tag")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", nargs="?")
    ap.add_argument("--tag", type=int, default=1)
    ap.add_argument("--synthetic", metavar="OUT")
    ap.add_argument("--corr", nargs="*", default=[], metavar="CAM:X,Y,DEG",
                    help="per-camera correction from CameraCorrections.java, by camera index")
    a = ap.parse_args(argv)
    truth = None
    path = a.log
    if a.synthetic:
        truth = write_synthetic(a.synthetic)
        path = a.synthetic
        print(f"wrote synthetic log {path}; true tag (odometry frame) = "
              f"({truth[0]:.3f}, {truth[1]:.3f}, {math.degrees(truth[2]):.2f} deg)")
    if not path:
        ap.error("give a log file or --synthetic OUT")
    corr = {}
    for item in a.corr:
        cam, vals = item.split(":")
        x, y, deg = (float(v) for v in vals.split(","))
        corr[int(cam)] = (x, y, math.radians(deg))
    obs, _, resets = load(path, a.tag)
    print(f"{len(obs)} frames of tag {a.tag}, {len(resets)} approaches")
    print(report(replay(obs, resets, variants(corr)), truth))


if __name__ == "__main__":
    sys.exit(main())
