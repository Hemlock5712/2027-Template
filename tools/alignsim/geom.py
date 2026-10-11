"""2D/3D pose math. 2D poses are plain (x, y, theta) tuples for speed in the hot loop."""

import math

import numpy as np

TAU = 2.0 * math.pi


def wrap(a):
    """Angle to (-pi, pi]."""
    return (a + math.pi) % TAU - math.pi


def compose(a, b):
    """a (+) b : apply b in a's frame (WPILib Pose2d.transformBy)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def relative(a, b):
    """b expressed in a's frame (WPILib b.relativeTo(a))."""
    return compose(inverse(a), b)


def rot(v, th):
    c, s = math.cos(th), math.sin(th)
    return (c * v[0] - s * v[1], s * v[0] + c * v[1])


def interp_pose(a, b, u):
    return (a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1]), wrap(a[2] + u * wrap(b[2] - a[2])))


def twist_exp(dx, dy, dth):
    """WPILib Twist2d.exp(): pose change for a constant-curvature move."""
    if abs(dth) < 1e-9:
        s, c = 1.0 - dth * dth / 6.0, dth / 2.0
    else:
        s, c = math.sin(dth) / dth, (1.0 - math.cos(dth)) / dth
    return (dx * s - dy * c, dx * c + dy * s, dth)


def twist_log(t):
    """WPILib Transform2d.log(): inverse of twist_exp."""
    dth = t[2]
    half = dth / 2.0
    cos_minus_one = math.cos(dth) - 1.0
    if abs(cos_minus_one) < 1e-9:  # WPILib's guard: 1 - cos underflows long before dth is tiny
        hcot = 1.0 - dth * dth / 12.0
    else:
        hcot = -half * math.sin(dth) / cos_minus_one
    x, y = t[0], t[1]
    # rotate (x, y) by -half and scale by hypot(hcot, half)
    c, s = math.cos(-half), math.sin(-half)
    rx, ry = c * x - s * y, s * x + c * y
    k = math.hypot(hcot, half)
    return (rx * k, ry * k, dth)


# ---------------------------------------------------------------------------- 3D (numpy)


def rot3_rpy(roll, pitch, yaw):
    """Rotation matrix for extrinsic X(roll) then Y(pitch) then Z(yaw), WPILib Rotation3d order.

    NWU: +X forward, +Y left, +Z up. Positive pitch tilts +X DOWN (right-hand rule about +Y).
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def tf3(R, t):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def tf3_from_pose2(p, z=0.0):
    return tf3(rot3_rpy(0, 0, p[2]), (p[0], p[1], z))


def tf3_inv(T):
    R, t = T[:3, :3], T[:3, 3]
    return tf3(R.T, -R.T @ t)


def tf3_to_pose2(T):
    """Project a 3D transform to (x, y, yaw)."""
    return (float(T[0, 3]), float(T[1, 3]), math.atan2(T[1, 0], T[0, 0]))


def yaw_of(R):
    return math.atan2(R[1, 0], R[0, 0])
