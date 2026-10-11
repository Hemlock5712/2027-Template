"""Drive-to-point control laws. Each takes the measured pose, the goal (both in the SAME frame),
the measured field velocity and dt, and returns a field-frame velocity command (vx, vy, w).

Implemented as faithfully as public code allows:
  VectorBrake     - this repo's DriveToPose (braking curve + linear zone + vector accel limit)
  Trailblazer581  - 581 offseason-2026 PidPathFollower + ConstraintsCalculator
  PerAxis2026     - 5712's 2026 DriveToPoint (per-axis braking, time-synced omega, friction circle)
  Rolling6328     - 6328 2025/2026 DriveToPose (distance profile re-seeded from the last setpoint)
  ProfiledPerAxis - WPILib ProfiledPIDController per x/y/theta (the old template DriveToTag style)
  LinearPathPID   - CTRE LinearPath (time-based, fixed at start) + PID trim (old template DriveToPose)
"""

import math

from .geom import rot, wrap


# ---------------------------------------------------------------------------- WPILib pieces
class TrapezoidProfile:
    """WPILib 2025/2026 TrapezoidProfile.calculate(t, current, goal)."""

    def __init__(self, max_v, max_a):
        self.max_v, self.max_a = max_v, max_a

    def calculate(self, t, cur, goal):
        cp, cv = cur
        gp, gv = goal
        direction = -1.0 if cp > gp else 1.0
        cp, cv, gp, gv = cp * direction, cv * direction, gp * direction, gv * direction
        if abs(cv) > self.max_v:
            cv = math.copysign(self.max_v, cv)
        a, vmax = self.max_a, self.max_v
        cutoff_begin = cv / a
        cutoff_dist_begin = cutoff_begin * cutoff_begin * a / 2.0
        cutoff_end = gv / a
        cutoff_dist_end = cutoff_end * cutoff_end * a / 2.0
        full_trap = cutoff_dist_begin + (gp - cp) + cutoff_dist_end
        accel_time = vmax / a
        full_speed_dist = full_trap - accel_time * accel_time * a
        if full_speed_dist < 0:
            accel_time = math.sqrt(max(0.0, full_trap / a))
            full_speed_dist = 0.0
        end_accel = accel_time - cutoff_begin
        end_full = end_accel + full_speed_dist / vmax
        end_decel = end_full + accel_time - cutoff_end
        p, v = cp, cv
        if t < end_accel:
            v += t * a
            p += (cv + t * a / 2.0) * t
        elif t < end_full:
            v = vmax
            p += (cv + end_accel * a / 2.0) * end_accel + vmax * (t - end_accel)
        elif t <= end_decel:
            tl = end_decel - t
            v = gv + tl * a
            p = gp - (gv + tl * a / 2.0) * tl
        else:
            p, v = gp, gv
        return p * direction, v * direction

    def total_time(self, cur, goal):
        # time until the profile reaches the goal (search, cheap enough)
        lo, hi = 0.0, 30.0
        for _ in range(40):
            mid = (lo + hi) / 2
            p, v = self.calculate(mid, cur, goal)
            if abs(p - goal[0]) < 1e-6 and abs(v - goal[1]) < 1e-6:
                hi = mid
            else:
                lo = mid
        return hi


class ProfiledPID:
    """WPILib ProfiledPIDController (P only + optional continuous input)."""

    def __init__(self, kp, max_v, max_a, continuous=False, kd=0.0):
        self.kp, self.kd = kp, kd
        self.profile = TrapezoidProfile(max_v, max_a)
        self.continuous = continuous
        self.sp = (0.0, 0.0)
        self.prev_err = None

    def reset(self, pos, vel):
        self.sp = (pos, vel)
        self.prev_err = None

    def calculate(self, meas, goal, dt):
        gp = goal
        sp_p, sp_v = self.sp
        if self.continuous:
            gp = meas + wrap(goal - meas)
            sp_p = meas + wrap(sp_p - meas)
        self.sp = self.profile.calculate(dt, (sp_p, sp_v), (gp, 0.0))
        err = self.sp[0] - meas
        d = 0.0
        if self.kd and self.prev_err is not None:
            d = self.kd * (err - self.prev_err) / dt
        self.prev_err = err
        return self.kp * err + d


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


# ---------------------------------------------------------------------------- controllers
class VectorBrake:
    """This repo's DriveToPose."""

    name = "VectorBrake"

    def __init__(self, vmax=3.5, decel=3.0, accel=6.0, kp=4.0, react=0.05,
                 wmax=6.0, wdecel=8.0, waccel=20.0, kp_w=5.0, end_speed=0.0, rotation_cap=False):
        self.__dict__.update(locals())
        del self.__dict__["self"]
        self.cmd = (0.0, 0.0, 0.0)

    def reset(self, pose, meas_vel):
        self.cmd = tuple(meas_vel)

    def step(self, pose, goal, meas_vel, dt):
        ex, ey = goal[0] - pose[0], goal[1] - pose[1]
        d = math.hypot(ex, ey)
        eth = wrap(goal[2] - pose[2])
        speed_now = math.hypot(self.cmd[0], self.cmd[1])
        brake_d = max(0.0, d - speed_now * self.react)
        v = min(self.vmax, math.sqrt(self.end_speed ** 2 + 2 * self.decel * brake_d))
        if self.end_speed == 0.0:
            v = min(v, self.kp * d)
        if self.rotation_cap and abs(eth) > math.radians(2):
            t_rot = abs(eth) / self.wmax
            v = min(v, max(d / t_rot, 0.4))
        vx, vy = (ex / d * v, ey / d * v) if d > 1e-6 else (0.0, 0.0)
        w = math.copysign(min(self.wmax, math.sqrt(2 * self.wdecel * abs(eth)), self.kp_w * abs(eth)), eth)
        dvx, dvy = vx - self.cmd[0], vy - self.cmd[1]
        dv = math.hypot(dvx, dvy)
        if dv > self.accel * dt:
            dvx *= self.accel * dt / dv
            dvy *= self.accel * dt / dv
        dw = _clamp(w - self.cmd[2], -self.waccel * dt, self.waccel * dt)
        self.cmd = (self.cmd[0] + dvx, self.cmd[1] + dvy, self.cmd[2] + dw)
        return self.cmd


class Trailblazer581:
    """581 offseason-2026: v = min(P*d, vmax, max(vPrev + a*dt, 0.5), sqrt(2*a*d)) toward the
    target + rotation-aware cap; heading = ProfiledPID(kP 15, 4 rot/s, 4 rot/s^2)."""

    name = "Trailblazer581"

    def __init__(self, kp=3.5, vmax=4.75, amax=4.0, kp_w=15.0, wmax=4 * 2 * math.pi,
                 walpha=4 * 2 * math.pi, lin_tol=0.05, ang_tol=math.radians(2), rotation_cap=True):
        self.kp, self.vmax, self.amax = kp, vmax, amax
        self.lin_tol, self.ang_tol, self.rotation_cap = lin_tol, ang_tol, rotation_cap
        self.wmax = wmax
        self.theta = ProfiledPID(kp_w, wmax, walpha, continuous=True)
        self.last_cmd_v = 0.0

    def reset(self, pose, meas_vel):
        self.last_cmd_v = math.hypot(meas_vel[0], meas_vel[1])
        self.theta.reset(pose[2], meas_vel[2])

    def step(self, pose, goal, meas_vel, dt):
        ex, ey = goal[0] - pose[0], goal[1] - pose[1]
        d = math.hypot(ex, ey)
        v = abs(self.kp * d)
        eth = abs(wrap(goal[2] - pose[2]))
        if self.rotation_cap:
            work = max(0.0, eth - self.ang_tol)
            if work > 0:
                cap = max(d / (work / self.wmax), 0.4)
                v = min(v, cap)
                if d <= self.lin_tol:
                    v = 0.0
        v = min(v, self.vmax)
        v = min(v, max(self.last_cmd_v + self.amax * min(dt, 0.04), 0.5))
        v = min(v, math.sqrt(2 * self.amax * d))
        self.last_cmd_v = v
        w = self.theta.calculate(pose[2], goal[2], dt)
        vx, vy = (ex / d * v, ey / d * v) if d > 1e-6 else (0.0, 0.0)
        return (vx, vy, w)


class PerAxis2026:
    """5712 2026 DriveToPoint: per-axis braking speeds with a demand-ratio friction budget,
    time-synchronized omega, then a friction-circle acceleration limit on the commanded velocity."""

    name = "PerAxis2026"

    def __init__(self, vmax=4.5, friction_accel=1.1 * 9.81, base_radius=0.41, react=0.1,
                 end_speed=0.0, pos_tol=0.02, rot_tol=math.radians(2)):
        self.vmax, self.fa, self.r, self.react = vmax, friction_accel, base_radius, react
        self.end_speed, self.pos_tol, self.rot_tol = end_speed, pos_tol, rot_tol
        self.wmax_hw = vmax / base_radius
        self.max_ang_decel = friction_accel / base_radius
        self.cmd = (0.0, 0.0, 0.0)

    def reset(self, pose, meas_vel):
        self.cmd = tuple(meas_vel)

    def _avail(self, omega, eth):
        a = abs(eth)
        ang = min(omega * omega / (2 * a), self.max_ang_decel) if a > 1e-9 else 0.0
        c = ang * self.r
        return math.sqrt(max(0.0, self.fa ** 2 - c * c))

    def _axis(self, dist, cur, react, acc, end):
        buffered = max(0.0, dist - cur * react)
        tgt = math.sqrt(end * end + 2 * acc * buffered)
        if tgt < cur:
            return min(tgt, self.vmax)
        return min(math.sqrt(end * end + 2 * acc * dist), self.vmax)

    def step(self, pose, goal, meas_vel, dt):
        tx, ty = goal[0] - pose[0], goal[1] - pose[1]
        d = math.hypot(tx, ty)
        eth = wrap(goal[2] - pose[2])
        cvx, cvy, cw = self.cmd
        speed = math.hypot(cvx, cvy)
        w_t = 0.0
        if abs(eth) >= self.rot_tol:
            buffered = max(0.0, abs(eth) - abs(cw) * self.react)
            stop_w = math.sqrt(2 * self.fa / self.r * buffered)
            eff_d = max(0.0, d - speed * self.react)
            acc = self._avail(cw, eth)
            brake_t = math.sqrt(2 * eff_d / acc) if acc > 1e-9 else float("inf")
            cruise_t = eff_d / speed if speed > 0 else brake_t
            drive_t = max(brake_t, cruise_t)
            time_w = 2 * abs(eth) / drive_t if drive_t > 0 else self.wmax_hw
            w_t = math.copysign(min(stop_w, time_w, self.wmax_hw), eth)
        vx = vy = 0.0
        if d >= self.pos_tol:
            acc = self._avail(w_t, eth)
            dx, dy = abs(tx), abs(ty)
            ux, uy = tx / d, ty / d
            ex, ey = self.end_speed * abs(ux), self.end_speed * abs(uy)
            sx, sy = abs(cvx), abs(cvy)
            ox = self._axis(dx, sx, self.react, acc, ex)
            oy = self._axis(dy, sy, self.react, acc, ey)
            demx, demy = abs(ox - sx), abs(oy - sy)
            tot = math.hypot(demx, demy)
            if tot < 1e-9:
                ax, ay = acc * abs(ux), acc * abs(uy)
            else:
                ax, ay = acc * demx / tot, acc * demy / tot
            vx = math.copysign(self._axis(dx, sx, self.react, ax, ex), tx)
            vy = math.copysign(self._axis(dy, sy, self.react, ay, ey), ty)
            s = math.hypot(vx, vy)
            if s > self.vmax:
                vx, vy = vx * self.vmax / s, vy * self.vmax / s
        # friction circle limit on the change of the commanded velocity
        ax_, ay_, aw_ = (vx - cvx) / dt, (vy - cvy) / dt, (w_t - cw) / dt
        lin = math.hypot(ax_, ay_)
        tot = math.hypot(lin, aw_ * self.r)
        if tot > self.fa:
            k = self.fa / tot
            ax_, ay_, aw_ = ax_ * k, ay_ * k, aw_ * k
        self.cmd = (cvx + ax_ * dt, cvy + ay_ * dt, cw + aw_ * dt)
        return self.cmd


class Rolling6328:
    """6328 DriveToPose: a 1-D distance trapezoid re-solved every loop from the LAST SETPOINT
    (not the measurement), feedforward fading out between 5 cm and 1 cm, P on distance."""

    name = "Rolling6328"

    def __init__(self, kp=1.8, vmax=4.0, amax=4.0, kp_w=5.0, kd_w=0.5, wmax=math.radians(500),
                 walpha=8.0, ff_min=0.01, ff_max=0.05, tol=0.01, min_sp_vel=-0.5):
        self.kp, self.tol, self.ff_min, self.ff_max, self.min_sp_vel = kp, tol, ff_min, ff_max, min_sp_vel
        self.profile = TrapezoidProfile(vmax, amax)
        self.theta = ProfiledPID(kp_w, wmax, walpha, continuous=True, kd=kd_w)
        self.sp_xy = (0.0, 0.0)
        self.sp_vel = (0.0, 0.0)

    def reset(self, pose, meas_vel):
        self.sp_xy = (pose[0], pose[1])
        self.sp_vel = (meas_vel[0], meas_vel[1])
        self.theta.reset(pose[2], meas_vel[2])

    def step(self, pose, goal, meas_vel, dt):
        cd = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        ff = _clamp((cd - self.ff_min) / (self.ff_max - self.ff_min), 0.0, 1.0)
        dx, dy = goal[0] - self.sp_xy[0], goal[1] - self.sp_xy[1]
        dn = math.hypot(dx, dy)
        if dn > 1e-9:
            spv = (self.sp_vel[0] * dx + self.sp_vel[1] * dy) / dn
        else:
            spv = 0.0
        spv = max(spv, self.min_sp_vel)
        p, v = self.profile.calculate(dt, (dn, -spv), (0.0, 0.0))
        scalar = self.kp * (p - cd) + v * ff
        if cd < self.tol:
            scalar = 0.0
        # direction from target toward robot; scalar is negative when approaching
        if cd > 1e-9:
            ux, uy = (pose[0] - goal[0]) / cd, (pose[1] - goal[1]) / cd
        else:
            ux = uy = 0.0
        vx, vy = ux * scalar, uy * scalar
        # new setpoint: distance p from the target, on the line toward the old setpoint
        if dn > 1e-9:
            self.sp_xy = (goal[0] - dx / dn * p, goal[1] - dy / dn * p)
            self.sp_vel = (dx / dn * -v, dy / dn * -v)  # v < 0 while approaching
        else:
            self.sp_xy = (goal[0], goal[1])
            self.sp_vel = (0.0, 0.0)
        w = self.theta.calculate(pose[2], goal[2], dt) + self.theta.sp[1] * ff
        return (vx, vy, w)


class ProfiledPerAxis:
    """WPILib ProfiledPIDController on x, y, theta + setpoint-velocity feedforward."""

    name = "ProfiledPerAxis"

    def __init__(self, kp=3.0, vmax=2.5, amax=3.0, kp_w=4.0, wmax=math.pi, walpha=2 * math.pi):
        self.x = ProfiledPID(kp, vmax, amax)
        self.y = ProfiledPID(kp, vmax, amax)
        self.t = ProfiledPID(kp_w, wmax, walpha, continuous=True)

    def reset(self, pose, meas_vel):
        self.x.reset(pose[0], meas_vel[0])
        self.y.reset(pose[1], meas_vel[1])
        self.t.reset(pose[2], meas_vel[2])

    def step(self, pose, goal, meas_vel, dt):
        vx = self.x.calculate(pose[0], goal[0], dt) + self.x.sp[1]
        vy = self.y.calculate(pose[1], goal[1], dt) + self.y.sp[1]
        w = self.t.calculate(pose[2], goal[2], dt) + self.t.sp[1]
        return (vx, vy, w)


class LinearPathPID:
    """CTRE LinearPath (straight-line trapezoid fixed at the start + heading trapezoid) + PID trim.
    The goal is frozen at reset() - a moving goal is ignored (that's the point of comparing)."""

    name = "LinearPathPID"

    def __init__(self, vmax=2.5, amax=3.0, wmax=math.pi, walpha=2 * math.pi, kp=3.0, kp_w=4.0):
        self.lin = TrapezoidProfile(vmax, amax)
        self.ang = TrapezoidProfile(wmax, walpha)
        self.kp, self.kp_w = kp, kp_w
        self.t = 0.0
        self.start = None
        self.goal = None

    def reset(self, pose, meas_vel):
        self.start = pose
        self.v0 = meas_vel
        self.t = 0.0
        self.goal = None

    def step(self, pose, goal, meas_vel, dt):
        if self.goal is None:
            self.goal = goal
        self.t += dt
        s, g = self.start, self.goal
        L = math.hypot(g[0] - s[0], g[1] - s[1])
        ux, uy = ((g[0] - s[0]) / L, (g[1] - s[1]) / L) if L > 1e-9 else (0.0, 0.0)
        v0 = self.v0[0] * ux + self.v0[1] * uy  # velocity off the line is dropped (as in CTRE)
        p, v = self.lin.calculate(self.t, (0.0, v0), (L, 0.0))
        dth = wrap(g[2] - s[2])
        pa, va = self.ang.calculate(self.t, (0.0, self.v0[2]), (dth, 0.0))
        spx, spy, spt = s[0] + ux * p, s[1] + uy * p, wrap(s[2] + pa)
        vx = ux * v + self.kp * (spx - pose[0])
        vy = uy * v + self.kp * (spy - pose[1])
        w = va + self.kp_w * wrap(spt - pose[2])
        return (vx, vy, w)


CONTROLLERS = {c.name: c for c in (VectorBrake, Trailblazer581, PerAxis2026, Rolling6328, ProfiledPerAxis, LinearPathPID)}
