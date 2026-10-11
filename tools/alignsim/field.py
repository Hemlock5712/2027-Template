"""Field: AprilTags with a MAP pose (what the code believes) and a TRUE pose (where they are)."""

import math
from dataclasses import dataclass

import numpy as np

from .geom import compose, rot3_rpy, tf3
from .params import FieldParams


@dataclass
class Tag:
    id: int
    map_pose: tuple  # (x, y, yaw) - from the field layout
    true_pose: tuple  # (x, y, yaw) - real placement
    height: float
    size: float

    def tf3(self, true=True):
        p = self.true_pose if true else self.map_pose
        return tf3(rot3_rpy(0, 0, p[2]), (p[0], p[1], self.height))

    def corners_in_tag(self):
        """3D corners in the tag frame (+X out of the face) in OpenCV IPPE_SQUARE order."""
        h = self.size / 2
        # pnp object frame: x_p = +Y_tag (viewer's right), y_p = +Z_tag (up)
        return np.array([[0, -h, h], [0, h, h], [0, h, -h], [0, -h, -h]], dtype=float)


class Field:
    TARGET_ID = 1

    def __init__(self, fp: FieldParams, rng=None):
        self.fp = fp
        tags = []
        e = fp.element_pose
        # element center sits behind the target face; faces are spread around it
        center = (e[0] - fp.element_radius * math.cos(e[2]), e[1] - fp.element_radius * math.sin(e[2]))
        if rng is not None:
            shift = (rng.normal(0, fp.element_xy_error), rng.normal(0, fp.element_xy_error),
                     rng.normal(0, fp.element_yaw_error))
        else:
            shift = (0.0, 0.0, 0.0)
        for k, ang in enumerate((0.0,) + tuple(fp.neighbor_faces)):
            yaw = e[2] + ang
            m = (center[0] + fp.element_radius * math.cos(yaw), center[1] + fp.element_radius * math.sin(yaw), yaw)
            # rigid element error: rotate about the element center, then shift
            c, s_ = math.cos(shift[2]), math.sin(shift[2])
            rx, ry = m[0] - center[0], m[1] - center[1]
            t = (center[0] + c * rx - s_ * ry + shift[0], center[1] + s_ * rx + c * ry + shift[1], yaw + shift[2])
            if rng is not None:
                t = (t[0] + rng.normal(0, fp.tag_mount_xy_error), t[1] + rng.normal(0, fp.tag_mount_xy_error),
                     t[2] + rng.normal(0, fp.tag_mount_yaw_error))
            tags.append(Tag(self.TARGET_ID if k == 0 else 2 + k, m, t, fp.tag_height, fp.tag_size))
        for k, (x, y, yaw, hgt) in enumerate(fp.other_tags):
            m = (x, y, yaw)
            if rng is not None:
                t = (x + rng.normal(0, fp.other_tag_xy_error), y + rng.normal(0, fp.other_tag_xy_error),
                     yaw + rng.normal(0, fp.other_tag_yaw_error))
            else:
                t = m
            tags.append(Tag(10 + k, m, t, hgt, fp.tag_size))
        self.tags = tags
        self.by_id = {t.id: t for t in tags}

    @property
    def target(self):
        return self.by_id[self.TARGET_ID]

    def goal(self, true=True, goal_in_tag=None):
        g = goal_in_tag if goal_in_tag is not None else self.fp.goal_in_tag
        t = self.target
        return compose(t.true_pose if true else t.map_pose, g)
