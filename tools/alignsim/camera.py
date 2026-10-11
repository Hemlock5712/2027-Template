"""Limelight-like camera: projects true tag corners with the TRUE intrinsics/mount, adds pixel
noise + motion blur, then solves with the BELIEVED intrinsics/mount exactly like the camera would
(OpenCV IPPE_SQUARE for single tags, multi-tag PnP for MegaTag1, gyro-constrained MegaTag2)."""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .geom import rot3_rpy, tf3, tf3_from_pose2, tf3_inv, tf3_to_pose2, wrap
from .params import CameraParams

# camera NWU (x fwd, y left, z up) <-> OpenCV (x right, y down, z fwd)
CV_FROM_NWU = np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]], dtype=float)
NWU_FROM_CV = CV_FROM_NWU.T
# tag NWU (x out of face) <-> IPPE object frame (x right as seen by a viewer, y up)
P_FROM_TAG = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], dtype=float)
TAG_FROM_P = P_FROM_TAG.T


def mount_tf(m):
    return tf3(rot3_rpy(m[3], m[4], m[5]), m[:3])


@dataclass
class Fiducial:
    id: int
    tag_in_cam: np.ndarray  # 4x4, best solution, camera NWU
    alt_in_cam: np.ndarray  # 4x4, the other IPPE solution
    ambiguity: float  # best/second reprojection error ratio (0 = unambiguous)
    tx: float  # deg, + right (from believed principal point)
    ty: float  # deg, + up
    corners: np.ndarray  # 4x2 px
    tag_in_robot: tuple  # (x, y, yaw) via the BELIEVED mount
    distance: float  # camera -> tag (m), from the solve


@dataclass
class Frame:
    camera: str
    t_capture: float  # truth (mid-exposure)
    t_capture_est: float  # what the robot computes: receive time - reported latency
    t_arrive: float  # when the robot can read it
    fiducials: list = field(default_factory=list)
    mt1: tuple = None  # robot pose (x, y, yaw) or None
    mt1_tags: int = 0
    mt1_avg_dist: float = 0.0
    mt2: tuple = None
    mt2_avg_dist: float = 0.0
    pieces: list = field(default_factory=list)  # (tx, ty) deg per detected game piece


class Camera:
    def __init__(self, cp: CameraParams, rng):
        self.cp = cp
        self.rng = rng
        self.mount_believed = mount_tf(cp.mount)
        err = cp.mount_error
        self.mount_true = self.mount_believed @ tf3(rot3_rpy(err[3], err[4], err[5]), err[:3])
        cx, cy = cp.width / 2, cp.height / 2
        self.K_true = np.array([[cp.fx, 0, cx], [0, cp.fy, cy], [0, 0, 1]], dtype=float)
        bx = cp.fx * (1 + cp.fx_error)
        by = cp.fy * (1 + cp.fx_error)
        self.K_believed = np.array(
            [[bx, 0, cx + cp.center_error[0]], [0, by, cy + cp.center_error[1]], [0, 0, 1]], dtype=float)
        self.next_capture = rng.uniform(0, 1 / cp.fps)

    # ------------------------------------------------------------------ projection
    def _project(self, pts_cam_nwu):
        cv = (CV_FROM_NWU @ pts_cam_nwu.T).T
        if np.any(cv[:, 2] < 0.05):
            return None
        u = self.K_true[0, 0] * cv[:, 0] / cv[:, 2] + self.K_true[0, 2]
        v = self.K_true[1, 1] * cv[:, 1] / cv[:, 2] + self.K_true[1, 2]
        return np.stack([u, v], axis=1)

    def _solve_tag(self, tag, img):
        obj = (P_FROM_TAG @ tag.corners_in_tag().T).T
        ok, rvecs, tvecs, errs = cv2.solvePnPGeneric(
            obj, img.reshape(-1, 1, 2), self.K_believed, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok or len(rvecs) == 0:
            return None
        errs = np.asarray(errs).ravel()
        order = np.argsort(errs)
        sols = []
        for k in order:
            R_cv, _ = cv2.Rodrigues(rvecs[k])
            R = NWU_FROM_CV @ R_cv @ P_FROM_TAG
            t = NWU_FROM_CV @ tvecs[k].ravel()
            sols.append(tf3(R, t))
        amb = float(errs[order[0]] / errs[order[1]]) if len(order) > 1 and errs[order[1]] > 1e-9 else 0.0
        if self.cp.pnp_yaw_bias:
            sols[0] = sols[0] @ tf3(rot3_rpy(0, 0, self.cp.pnp_yaw_bias), (0, 0, 0))
        return sols[0], (sols[1] if len(sols) > 1 else sols[0]), amb

    # ------------------------------------------------------------------ capture
    def capture(self, t, robot_pose, robot_vel, field, mt2_heading, pieces=()):
        """One frame at mid-exposure time t from the TRUE robot state."""
        cp = self.cp
        T_field_cam = tf3_from_pose2(robot_pose) @ self.mount_true
        T_cam_field = tf3_inv(T_field_cam)
        speed = math.hypot(robot_vel[0], robot_vel[1])
        latency = cp.capture_latency + max(0.002, self.rng.normal(cp.pipeline_latency, cp.pipeline_jitter))
        t_arrive = t + latency + cp.network_latency
        frame = Frame(cp.name, t, t + cp.clock_bias + self.rng.normal(0, 0.0003), t_arrive)

        mt1_obj, mt1_img = [], []
        for tag in field.tags:
            T_cam_tag = T_cam_field @ tag.tf3(true=True)
            corners = (T_cam_tag[:3, :3] @ tag.corners_in_tag().T).T + T_cam_tag[:3, 3]
            img = self._project(corners)
            if img is None:
                continue
            if np.any(img[:, 0] < 2) or np.any(img[:, 0] > cp.width - 2) or np.any(img[:, 1] < 2) \
                    or np.any(img[:, 1] > cp.height - 2):
                continue
            side = np.mean(np.linalg.norm(img - np.roll(img, 1, axis=0), axis=1))
            if side < cp.min_tag_pixels:
                continue
            to_cam = -T_cam_tag[:3, 3] @ T_cam_tag[:3, :3]  # camera position in tag frame
            view = math.acos(max(-1.0, min(1.0, to_cam[0] / max(1e-9, np.linalg.norm(to_cam)))))
            if view > cp.max_view_angle:
                continue
            # motion blur (px) during the exposure, as extra corner noise
            z = max(0.1, T_cam_tag[0, 3])
            blur = cp.fx * (speed / z + abs(robot_vel[2])) * cp.exposure
            sigma = math.sqrt(cp.pixel_noise ** 2 + blur ** 2 / 12.0)
            noisy = img + self.rng.normal(0, sigma, img.shape)
            sol = self._solve_tag(tag, noisy)
            if sol is None:
                continue
            best, alt, amb = sol
            center = noisy.mean(axis=0)
            Kb = self.K_believed
            tx = math.degrees(math.atan((center[0] - Kb[0, 2]) / Kb[0, 0]))
            ty = math.degrees(math.atan((Kb[1, 2] - center[1]) / Kb[1, 1]))
            T_robot_tag = self.mount_believed @ best
            frame.fiducials.append(Fiducial(
                tag.id, best, alt, amb, tx, ty, noisy, tf3_to_pose2(T_robot_tag),
                float(np.linalg.norm(best[:3, 3]))))
            # MegaTag1 object points use the MAP tag pose
            Tm = tag.tf3(true=False)
            mt1_obj.append((Tm[:3, :3] @ tag.corners_in_tag().T).T + Tm[:3, 3])
            mt1_img.append(noisy)

        if frame.fiducials:
            self._megatag(frame, field, mt1_obj, mt1_img, mt2_heading)
        if pieces:
            self._detect_pieces(frame, T_cam_field, pieces)
        return frame

    def _megatag(self, frame, field, mt1_obj, mt1_img, heading):
        inv_mount = tf3_inv(self.mount_believed)
        if len(frame.fiducials) >= 2:
            obj = np.vstack(mt1_obj)
            img = np.vstack(mt1_img)
            # field NWU points -> solvePnP gives field->camera(cv)
            ok, rvec, tvec = cv2.solvePnP(obj, img, self.K_believed, None, flags=cv2.SOLVEPNP_SQPNP)
            if ok:
                R_cv, _ = cv2.Rodrigues(rvec)
                T_cv_field = tf3(R_cv, tvec.ravel())
                T_nwu_field = tf3(NWU_FROM_CV, (0, 0, 0)) @ T_cv_field
                T_field_cam = tf3_inv(T_nwu_field)
                frame.mt1 = tf3_to_pose2(T_field_cam @ inv_mount)
                frame.mt1_tags = len(frame.fiducials)
        else:
            f = frame.fiducials[0]
            tag = field.by_id[f.id]
            T_field_cam = tag.tf3(true=False) @ tf3_inv(f.tag_in_cam)
            frame.mt1 = tf3_to_pose2(T_field_cam @ inv_mount)
            frame.mt1_tags = 1
        frame.mt1_avg_dist = float(np.mean([f.distance for f in frame.fiducials]))
        # MegaTag2: robot yaw from the gyro feed, position from each tag's translation
        xs, ys, ws = [], [], []
        c, s = math.cos(heading), math.sin(heading)
        for f in frame.fiducials:
            tag = field.by_id[f.id]
            px, py = f.tag_in_robot[0], f.tag_in_robot[1]
            xs.append(tag.map_pose[0] - (c * px - s * py))
            ys.append(tag.map_pose[1] - (s * px + c * py))
            ws.append(1.0 / max(0.05, f.distance) ** 2)
        w = np.array(ws)
        frame.mt2 = (float(np.dot(w, xs) / w.sum()), float(np.dot(w, ys) / w.sum()), heading)
        frame.mt2_avg_dist = frame.mt1_avg_dist

    def _detect_pieces(self, frame, T_cam_field, pieces):
        cp = self.cp
        for (x, y, h) in pieces:
            p = T_cam_field @ np.array([x, y, h, 1.0])
            img = self._project(p[:3].reshape(1, 3))
            if img is None:
                continue
            u, v = img[0]
            if not (0 < u < cp.width and 0 < v < cp.height) or p[0] > 5.0:
                continue
            u += self.rng.normal(0, cp.detector_noise)
            v += self.rng.normal(0, cp.detector_noise)
            Kb = self.K_believed
            tx = math.degrees(math.atan((u - Kb[0, 2]) / Kb[0, 0]))
            ty = math.degrees(math.atan((Kb[1, 2] - v) / Kb[1, 1]))
            frame.pieces.append((tx, ty))

    def maybe_capture(self, t, *args, **kwargs):
        if t + 1e-9 < self.next_capture:
            return None
        self.next_capture += 1.0 / self.cp.fps
        return self.capture(t, *args, **kwargs)
