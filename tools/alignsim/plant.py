"""Swerve drivetrain physics: 4 modules with steering lag, motor torque-speed curve + current
limit, and stick/slip wheel contact (friction circle), so hard maneuvers really slip and
odometry really drifts. Plus a gyro and wheel/steer encoders with errors."""

import math

from .geom import rot, wrap
from .params import RobotParams

G = 9.81
V_REG = 0.05  # m/s: sliding friction ramps to full below this slip speed
SUBSTEPS = 8


class SwervePlant:
    def __init__(self, rp: RobotParams, pose, field_vel=(0.0, 0.0, 0.0), rng=None):
        self.rp = rp
        self.x, self.y, self.th = pose
        self.th_unwrapped = pose[2]
        self.vx, self.vy, self.w = field_vel
        self.mods = rp.module_positions
        # Module state: start rolling in the direction of motion
        vr = rot((self.vx, self.vy), -self.th)
        self.steer = []
        self.wheel = []
        for mx, my in self.mods:
            cx, cy = vr[0] - self.w * my, vr[1] + self.w * mx
            ang = math.atan2(cy, cx) if math.hypot(cx, cy) > 1e-3 else 0.0
            self.steer.append(ang)
            self.wheel.append(math.hypot(cx, cy))
        self.slipping = [False] * 4
        self.target_angle = list(self.steer)
        self.target_speed = list(self.wheel)
        self.neutral = False  # Idle request: motors in brake
        self.drive_dist = [0.0] * 4  # what the encoders report (m)
        self.ext = (0.0, 0.0, 0.0)  # external field-frame force (N) and torque (Nm)
        self.ext_until = -1.0
        self.slip_time = 0.0
        # Gyro error terms (per trial)
        self.rng = rng
        self.gyro_bias = 0.0
        self.t = 0.0
        self.mass_per_wheel = rp.mass / 4.0
        self.normal = rp.mass * G / 4.0
        self.kv_volts = 12.0 / rp.free_speed

    # ------------------------------------------------------------------ commands
    def set_chassis_robot(self, vx, vy, w):
        """Robot-relative ChassisVelocities -> module targets (like CTRE's request at apply time)."""
        rp = self.rp
        speeds, angles = [], []
        for mx, my in self.mods:
            cx, cy = vx - w * my, vy + w * mx
            speeds.append(math.hypot(cx, cy))
            angles.append(math.atan2(cy, cx))
        top = max(speeds)
        if top > rp.free_speed:  # desaturate
            speeds = [s * rp.free_speed / top for s in speeds]
        for i in range(4):
            s, a = speeds[i], angles[i]
            if s < 1e-4:  # hold angle when stopped
                a = self.target_angle[i]
            err = wrap(a - self.steer[i])
            if abs(err) > math.pi / 2:  # optimize: flip
                a = wrap(a + math.pi)
                s = -s
            self.target_angle[i] = a
            self.target_speed[i] = s
        self.neutral = False

    def set_neutral(self):
        self.neutral = True

    def push(self, fx, fy, torque, duration):
        """External shove (collision / defense), field frame."""
        self.ext = (fx, fy, torque)
        self.ext_until = self.t + duration

    # ------------------------------------------------------------------ physics
    def step(self, dt):
        rp = self.rp
        c, s = math.cos(self.th), math.sin(self.th)
        vxr, vyr = c * self.vx + s * self.vy, -s * self.vx + c * self.vy  # robot frame
        fxr = fyr = tau = 0.0
        any_slip = False
        for i in range(4):
            mx, my = self.mods[i]
            # steering (onboard position loop, rate limited)
            err = wrap(self.target_angle[i] - self.steer[i])
            rate = max(-rp.steer_max_rate, min(rp.steer_max_rate, rp.steer_gain * err))
            self.steer[i] = wrap(self.steer[i] + rate * dt)
            ch, sh = math.cos(self.steer[i]), math.sin(self.steer[i])

            # drive motor (onboard velocity loop + torque curve + stator limit)
            sw = self.wheel[i]
            if self.neutral:
                volts = 0.0
            else:
                sp = self.target_speed[i]
                if rp.cosine_compensation:
                    sp *= math.cos(wrap(self.target_angle[i] - self.steer[i]))
                volts = self.kv_volts * sp + rp.drive_kp * (sp - sw)
                if abs(sp) > 1e-3:
                    volts += math.copysign(rp.drive_ks, sp)
                volts = max(-12.0, min(12.0, volts))
            rotor = sw / rp.wheel_radius * rp.drive_gear
            amps = (volts - rotor / rp.motor_kv) / rp.motor_r
            amps = max(-rp.stator_limit, min(rp.stator_limit, amps))
            f_motor = rp.motor_kt * amps * rp.drive_gear / rp.wheel_radius

            # contact point velocity, along / across the wheel
            cvx, cvy = vxr - self.w * my, vyr + self.w * mx
            u_long = cvx * ch + cvy * sh
            u_lat = -cvx * sh + cvy * ch

            f_static_long = f_motor
            f_static_lat = -rp.lateral_stiffness * u_lat
            fmax_s = rp.mu_static * self.normal
            if self.slipping[i]:
                slip_l, slip_t = sw - u_long, -u_lat
                if math.hypot(slip_l, slip_t) < 0.03 and math.hypot(f_static_long, f_static_lat) <= fmax_s:
                    self.slipping[i] = False
            elif math.hypot(f_static_long, f_static_lat) > fmax_s:
                self.slipping[i] = True

            if self.slipping[i]:
                any_slip = True
                # kinetic friction, regularized below V_REG and sub-stepped so the light wheel
                # can't chatter around zero slip
                fk = rp.mu_kinetic * self.normal
                slip_t = -u_lat
                fl_sum = ft_sum = 0.0
                h = dt / SUBSTEPS
                for _ in range(SUBSTEPS):
                    slip_l = sw - u_long
                    k = fk / max(math.hypot(slip_l, slip_t), V_REG)
                    fl_k, ft_k = k * slip_l, k * slip_t
                    sw += (f_motor - fl_k) / rp.reflected_wheel_mass * h
                    fl_sum += fl_k
                    ft_sum += ft_k
                fl, ft = fl_sum / SUBSTEPS, ft_sum / SUBSTEPS
                self.wheel[i] = sw
            else:
                fl, ft = f_static_long, f_static_lat
                self.wheel[i] = u_long
            # force on chassis, robot frame
            fx = fl * ch - ft * sh
            fy = fl * sh + ft * ch
            fxr += fx
            fyr += fy
            tau += mx * fy - my * fx
            self.drive_dist[i] += self.wheel[i] * (1.0 + rp.wheel_radius_error) * dt

        # rolling resistance
        fxr -= 2.0 * vxr
        fyr -= 2.0 * vyr
        ffx, ffy = rot((fxr, fyr), self.th)
        if self.t < self.ext_until:
            ffx += self.ext[0]
            ffy += self.ext[1]
            tau += self.ext[2]
        # move with the velocity the encoders just integrated, so a rolling wheel reads true travel
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.th = wrap(self.th + self.w * dt)
        self.th_unwrapped += self.w * dt
        self.vx += ffx / rp.mass * dt
        self.vy += ffy / rp.mass * dt
        self.w += tau / rp.inertia * dt
        self.t += dt
        self.gyro_bias += rp.gyro_drift_rate * dt
        if any_slip:
            self.slip_time += dt

    # ------------------------------------------------------------------ sensors
    def pose(self):
        return (self.x, self.y, self.th)

    def field_velocity(self):
        return (self.vx, self.vy, self.w)

    def read_gyro(self):
        rp = self.rp
        noise = self.rng.normal(0, rp.gyro_noise) if self.rng is not None else 0.0
        return wrap(self.th_unwrapped * (1.0 + rp.gyro_scale_error) + self.gyro_bias + noise)

    def read_modules(self):
        """(distance, angle) per module as the encoders report them."""
        rp = self.rp
        out = []
        for i in range(4):
            noise = self.rng.normal(0, rp.steer_noise) if self.rng is not None else 0.0
            out.append((self.drive_dist[i], wrap(self.steer[i] + noise)))
        return out

    def module_velocities(self):
        """(speed, angle) per module - what CTRE reports as ModuleVelocities."""
        return [(self.wheel[i], self.steer[i]) for i in range(4)]
