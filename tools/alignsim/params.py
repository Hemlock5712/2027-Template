"""Every tunable number of the simulated robot, cameras and field, with its source or rationale.

Numbers marked ASSUMED are guesses to be replaced from robot logs (see docs/alignment test plan).
"""

import math
from dataclasses import dataclass, field, replace


@dataclass
class RobotParams:
    # Chassis
    mass: float = 60.0  # kg incl. battery + bumpers (2026 code used 60)
    inertia: float = 6.0  # kg m^2 (~ m * (0.8^2 + 0.8^2) / 12)
    module_xy: float = 0.29  # m, module centers at (+-x, +-y)
    # Traction - ASSUMED, tread on carpet
    mu_static: float = 1.15
    mu_kinetic: float = 1.0
    lateral_stiffness: float = 4000.0  # N per m/s of sideways tread creep, before sliding (brush model)
    # Drive: Kraken X60 FOC
    wheel_radius: float = 0.0508  # m (2 in)
    drive_gear: float = 6.75
    motor_kt: float = 0.0194  # Nm/A  (9.37 Nm / 483 A)
    motor_r: float = 0.0248  # ohm   (12 V / 483 A)
    motor_kv: float = 50.6  # rad/s per V (5800 rpm / 12 V)
    stator_limit: float = 120.0  # A (template "slip current")
    reflected_wheel_mass: float = 1.2  # kg, rotor + wheel inertia seen at the tread
    drive_kp: float = 3.0  # V per m/s of wheel speed error (onboard velocity loop)
    drive_ks: float = 0.15  # V static friction feedforward
    cosine_compensation: bool = True
    # Steering
    steer_gain: float = 60.0  # 1/s
    steer_max_rate: float = 25.0  # rad/s
    steer_noise: float = math.radians(0.15)  # encoder noise
    # Sensors - ASSUMED
    wheel_radius_error: float = 0.0  # fractional, set per trial (wear, tread compression)
    gyro_noise: float = math.radians(0.02)
    gyro_drift_rate: float = math.radians(0.5) / 60.0  # rad/s
    gyro_scale_error: float = 0.0  # fractional, set per trial

    @property
    def free_speed(self):
        return self.motor_kv * 12.0 / self.drive_gear * self.wheel_radius

    @property
    def module_positions(self):
        a = self.module_xy
        return ((a, a), (a, -a), (-a, a), (-a, -a))


@dataclass
class CameraParams:
    name: str = "cam"
    # Mount on robot (robot frame, NWU): x, y, z (m), roll, pitch, yaw (rad). Pitch < 0 = looking up.
    mount: tuple = (0.25, 0.20, 0.30, 0.0, math.radians(-15), math.radians(10))
    # Limelight 4: OV9281 1280x800 global shutter. ~82 deg HFOV -> fx ~ 740 px.
    width: int = 1280
    height: int = 800
    fx: float = 740.0
    fy: float = 740.0
    fps: float = 60.0
    exposure: float = 0.0015  # s
    pixel_noise: float = 0.35  # px, corner noise (ASSUMED)
    min_tag_pixels: float = 18.0  # tag side must be at least this many px to decode
    max_view_angle: float = math.radians(70)
    # Timing (ASSUMED until logged): mid-exposure -> pipeline start, pipeline, network
    capture_latency: float = 0.008
    pipeline_latency: float = 0.012
    pipeline_jitter: float = 0.003
    network_latency: float = 0.001
    # The camera clock is synced to the robot once per NT connection; bias of that sync (s).
    clock_bias: float = 0.0
    # Mount calibration error (what the robot BELIEVES vs truth), set per trial
    mount_error: tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    # Calibration errors, per trial: believed fx vs true (fraction), principal point (px)
    fx_error: float = 0.0
    center_error: tuple = (0.0, 0.0)
    # Extra systematic yaw error in the tag's solved orientation (lens model error), per trial
    pnp_yaw_bias: float = 0.0
    # Game-piece detector: bounding-box center noise (px)
    detector_noise: float = 2.0


@dataclass
class FieldParams:
    # Scoring element: the tag on it and where the robot should end up (tag frame, robot facing tag)
    element_pose: tuple = (5.0, 4.0, 0.0)  # map pose of the tag (x, y, yaw); +x out of the face
    tag_height: float = 0.30
    tag_size: float = 0.1651  # 6.5 in
    goal_in_tag: tuple = (0.55, 0.0, math.pi)  # robot center 0.55 m out, facing the tag
    # How far the real element is from the map (1 sigma), drawn per trial - ASSUMED.
    # The whole element moves rigidly; each tag on it also has its own small mounting error.
    element_xy_error: float = 0.025
    element_yaw_error: float = math.radians(1.0)
    tag_mount_xy_error: float = 0.004
    tag_mount_yaw_error: float = math.radians(0.3)
    # Reef-like neighbor faces on the same element: (angle around the element center, radius).
    # Target face is at angle 0. Set to () for a lone tag.
    element_radius: float = 0.8
    neighbor_faces: tuple = (math.radians(60), math.radians(-60))
    # Other field tags (for the global pose): (x, y, yaw, height); each misplaced by this sigma
    other_tags: tuple = (
        (8.27, 2.0, math.pi, 1.2),
        (8.27, 6.0, math.pi, 1.2),
        (2.0, 8.0, -math.pi / 2, 1.2),
        (2.0, 0.0, math.pi / 2, 1.2),
        (0.0, 4.0, 0.0, 1.2),
    )
    other_tag_xy_error: float = 0.015
    other_tag_yaw_error: float = math.radians(0.5)


@dataclass
class LoopParams:
    sim_dt: float = 0.002  # 500 Hz physics
    odom_period: float = 0.004  # 250 Hz CTRE odometry thread (CAN FD)
    main_period: float = 0.020  # 50 Hz robot loop
    main_jitter: float = 0.001
    actuation_delay: float = 0.002  # request -> motors (applied on the next odometry tick anyway)


def front_cameras():
    """Two Limelights on the front corners, angled slightly inward, looking up 15 deg."""
    return (
        CameraParams(name="ll-left", mount=(0.25, 0.20, 0.30, 0.0, math.radians(-15), math.radians(-10))),
        CameraParams(name="ll-right", mount=(0.25, -0.20, 0.30, 0.0, math.radians(-15), math.radians(10))),
    )


__all__ = ["RobotParams", "CameraParams", "FieldParams", "LoopParams", "front_cameras", "replace", "field"]
