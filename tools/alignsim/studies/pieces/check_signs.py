"""End-to-end tx/ty sign check: a piece on the robot's LEFT must give tx < 0 and a floor point with
y > 0; a nearer piece must give a lower ty. Also checks PieceCamera == core Camera when the box
model is off. Run: python3 check_signs.py (asserts, prints a small table)."""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from piece_world import PieceCamera, piece_camera  # noqa: E402
from strategies import floor_point  # noqa: E402

from alignsim.camera import Camera  # noqa: E402
from alignsim.field import Field  # noqa: E402
from alignsim.geom import compose  # noqa: E402
from alignsim.params import FieldParams, replace  # noqa: E402


def detect(cam, robot, piece_robot, h=0.10):
    f = Field(FieldParams())
    f.tags = []
    p = compose(robot, (piece_robot[0], piece_robot[1], 0.0))
    return cam.capture(0.0, robot, (0, 0, 0), f, robot[2], [(p[0], p[1], h)]).pieces


def main():
    rows = []
    for yaw_deg, cam_y in ((0.0, 0.0), (10.0, 0.15), (-10.0, -0.2)):
        cp = replace(piece_camera(y=cam_y, yaw_deg=yaw_deg), detector_noise=0.0)
        cam = PieceCamera(cp, np.random.default_rng(0), bbox=False)
        for robot in ((0.0, 0.0, 0.0), (3.0, 1.0, 2.0)):
            for pr in ((1.5, 0.5), (1.5, -0.5), (1.0, 0.0), (2.5, 0.0)):
                dets = detect(cam, robot, pr)
                assert len(dets) == 1, (pr, dets)
                tx, ty = dets[0]
                fp = floor_point(cam.mount_believed, tx, ty, 0.10)
                assert abs(fp[0] - pr[0]) < 1e-6 and abs(fp[1] - pr[1]) < 1e-6, (pr, fp)
                if robot == (0.0, 0.0, 0.0):
                    rows.append((yaw_deg, cam_y, pr, tx, ty, fp))
    # sign rules on the centered camera
    c = [r for r in rows if r[0] == 0.0]
    left, right, near, far = (next(r for r in c if r[2] == k) for k in ((1.5, 0.5), (1.5, -0.5), (1.0, 0.0), (2.5, 0.0)))
    assert left[3] < 0 < right[3], "tx must be negative for a piece on the left"
    assert left[5][1] > 0 > right[5][1], "a piece on the left must project to +y"
    assert near[4] < far[4], "a nearer floor piece must sit lower in the image (smaller ty)"
    # PieceCamera with the box model off must match the core detector draw for draw
    for seed in range(5):
        cp = replace(piece_camera(), detector_noise=2.0)
        a, b = Camera(cp, np.random.default_rng(seed)), PieceCamera(cp, np.random.default_rng(seed), bbox=False)
        assert detect(a, (0, 0, 0.3), (1.7, 0.3)) == detect(b, (0, 0, 0.3), (1.7, 0.3))
    print("| cam yaw | cam y | piece (robot x, y) | tx deg | ty deg | floor point (x, y) |")
    print("|---|---|---|---|---|---|")
    for yaw_deg, cam_y, pr, tx, ty, fp in rows:
        print(f"| {yaw_deg:+.0f} | {cam_y:+.2f} | ({pr[0]:.1f}, {pr[1]:+.1f}) | {tx:+6.2f} | {ty:+6.2f} | "
              f"({fp[0]:.3f}, {fp[1]:+.3f}) |")
    print("signs OK: left piece -> tx < 0 -> floor y > 0; nearer -> lower ty; PieceCamera == core")


if __name__ == "__main__":
    main()
