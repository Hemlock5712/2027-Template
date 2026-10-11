"""Sanity tests: each model reduces to the right answer in the noise-free limit."""

import math

import numpy as np
import pytest

from alignsim.camera import Camera
from alignsim.controllers import CONTROLLERS, TrapezoidProfile
from alignsim.field import Field
from alignsim.geom import compose, inverse, relative, twist_exp, twist_log
from alignsim.odometry import PoseEstimator, SwerveOdometry, TimeBuffer
from alignsim.params import FieldParams, RobotParams, front_cameras, replace
from alignsim.plant import SwervePlant


def test_twist_roundtrip():
    for t in [(0.3, -0.2, 0.7), (1.0, 0.0, 0.0), (0.0, 0.5, -2.5)]:
        tw = twist_log(t)
        back = twist_exp(*tw)
        assert np.allclose(back, t, atol=1e-9)


def test_compose_inverse():
    a = (1.0, 2.0, 0.5)
    assert np.allclose(compose(a, inverse(a)), (0, 0, 0), atol=1e-12)
    b = (3.0, -1.0, -1.2)
    assert np.allclose(compose(a, relative(a, b)), b, atol=1e-12)


def test_trapezoid_reaches_goal():
    tp = TrapezoidProfile(2.0, 3.0)
    assert np.allclose(tp.calculate(10.0, (0, 0), (1, 0)), (1, 0))
    p, v = tp.calculate(0.2, (0, 0), (1, 0))
    assert v == pytest.approx(0.6) and p == pytest.approx(0.06)


def test_plant_traction_and_top_speed():
    rp = RobotParams()
    p = SwervePlant(rp, (0, 0, 0))
    p.set_chassis_robot(10, 0, 0)
    for _ in range(100):  # 0.2 s
        p.step(0.002)
    accel = p.vx / 0.2
    assert accel < rp.mu_static * 9.81 + 0.5  # traction limited
    for _ in range(1500):
        p.step(0.002)
    assert p.vx == pytest.approx(rp.free_speed, rel=0.02)


def test_odometry_matches_truth_without_slip():
    rp = replace(RobotParams(), steer_noise=0.0, gyro_noise=0.0, gyro_drift_rate=0.0)
    p = SwervePlant(rp, (0, 0, 0))
    odo = SwerveOdometry(rp.module_positions, p.read_gyro(), p.read_modules(), (0, 0, 0))
    # gentle (ramped) commands: no slip. A step command really does spin the wheels.
    for k in range(1500):
        t = k * 0.002
        if k % 2 == 0:
            ramp = min(1.0, t / 0.5)
            p.set_chassis_robot(0.8 * math.cos(t) * ramp, 0.5 * math.sin(t) * ramp, 0.6 * ramp)
            odo.update(p.read_gyro(), p.read_modules())
        p.step(0.002)
    assert p.slip_time < 0.05
    assert math.hypot(odo.pose[0] - p.x, odo.pose[1] - p.y) < 0.01


def test_camera_noise_free_recovers_truth():
    f = Field(FieldParams())
    rng = np.random.default_rng(0)
    for cp in front_cameras():
        cam = Camera(replace(cp, pixel_noise=0.0), rng)
        robot = (f.target.map_pose[0] + 1.3, f.target.map_pose[1] - 0.2, math.pi - 0.1)
        fr = cam.capture(0.0, robot, (0, 0, 0), f, robot[2])
        tgt = [x for x in fr.fiducials if x.id == f.TARGET_ID][0]
        truth = relative(robot, f.target.true_pose)
        assert np.allclose(tgt.tag_in_robot[:2], truth[:2], atol=1e-6)
        assert abs(math.remainder(tgt.tag_in_robot[2] - truth[2], 2 * math.pi)) < 1e-6
        assert np.allclose(fr.mt2[:2], robot[:2], atol=1e-6)


def test_estimator_pulls_toward_vision():
    rp = RobotParams()
    p = SwervePlant(rp, (0, 0, 0))
    odo = SwerveOdometry(rp.module_positions, p.read_gyro(), p.read_modules(), (0.2, 0.0, 0.0))
    est = PoseEstimator(odo)
    for k in range(50):
        est.update(k * 0.02, p.read_gyro(), p.read_modules())
        est.add_vision((0.0, 0.0, 0.0), k * 0.02, (0.05, 0.05, 1e6))
    assert abs(est.estimate[0]) < 0.01


def test_estimator_drops_newer_on_older_insert():
    rp = RobotParams()
    p = SwervePlant(rp, (0, 0, 0))
    odo = SwerveOdometry(rp.module_positions, p.read_gyro(), p.read_modules(), (0, 0, 0))
    est = PoseEstimator(odo)
    for k in range(10):
        est.update(k * 0.02, p.read_gyro(), p.read_modules())
    est.add_vision((0.5, 0, 0), 0.15, (0.1, 0.1, 1e6))
    est.add_vision((0.5, 0, 0), 0.10, (0.1, 0.1, 1e6))
    assert est.vision_t == [0.10]


@pytest.mark.parametrize("name", list(CONTROLLERS))
def test_controllers_converge_point_mass(name):
    c = CONTROLLERS[name]()
    pose = [0.0, 0.0, 0.0]
    c.reset(tuple(pose), (0, 0, 0))
    goal = (2.0, 0.5, math.pi / 2)
    for _ in range(300):
        v = c.step(tuple(pose), goal, (0, 0, 0), 0.02)
        pose = [pose[0] + v[0] * 0.02, pose[1] + v[1] * 0.02, pose[2] + v[2] * 0.02]
    assert math.hypot(goal[0] - pose[0], goal[1] - pose[1]) < 0.025


def test_time_buffer_interpolates():
    b = TimeBuffer(1.0)
    b.add(0.0, (0, 0, 0))
    b.add(1.0, (1, 0, 0))
    assert np.allclose(b.sample(0.25), (0.25, 0, 0))
