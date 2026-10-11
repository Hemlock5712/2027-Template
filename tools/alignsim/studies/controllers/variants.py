"""Study-local controller variants. The core laws live in alignsim/controllers.py and are not
changed; these add the knobs the study needs (pass-through end speed, fixes for failure modes)."""

import math

from alignsim.controllers import (LinearPathPID, ProfiledPerAxis, ProfiledPID, Rolling6328,
                                  Trailblazer581, TrapezoidProfile, VectorBrake, _clamp)
from alignsim.geom import wrap


def _vec_limit(cmd, target, a, dt):
    dvx, dvy = target[0] - cmd[0], target[1] - cmd[1]
    dv = math.hypot(dvx, dvy)
    if dv > a * dt:
        dvx, dvy = dvx * a * dt / dv, dvy * a * dt / dv
    return cmd[0] + dvx, cmd[1] + dvy


# ---------------------------------------------------------------------------- VectorBrake family
class VectorBrakeX(VectorBrake):
    """VectorBrake + options:
    budget:  share one acceleration budget (m/s^2 at the module) between translation and rotation,
             |a_lin| + |alpha| * r <= budget, so combined steps can't break traction.
    lat_brake: plan braking on the speed ALONG the goal direction only after the sideways part has
             been removed (time to cancel it is added to the reaction distance).
    """

    name = "VectorBrakeX"

    def __init__(self, budget=None, radius=0.41, lat_brake=False, switch_radius=0.0, perp_first=False,
                 wreact=0.0, **kw):
        super().__init__(**kw)
        self.wreact = wreact  # heading: brake this many seconds early, like react for translation
        self.perp_first = perp_first  # spend the accel limit on sideways speed before speeding up
        self.budget, self.radius, self.lat_brake = budget, radius, lat_brake
        self.switch_radius = switch_radius  # pass-through: reach end_speed this far before the waypoint

    def step(self, pose, goal, meas_vel, dt):
        ex, ey = goal[0] - pose[0], goal[1] - pose[1]
        d = math.hypot(ex, ey)
        eth = wrap(goal[2] - pose[2])
        cvx, cvy, cw = self.cmd
        speed_now = math.hypot(cvx, cvy)
        react_d = speed_now * self.react
        if self.lat_brake and d > 1e-6:
            # sideways speed must be cancelled before the braking curve applies
            lat = abs(-cvx * ey / d + cvy * ex / d)
            t_lat = lat / self.accel
            along = max(0.0, (cvx * ex + cvy * ey) / d)
            react_d += along * t_lat * 0.5
        if self.end_speed > 0:
            react_d += self.switch_radius
        brake_d = max(0.0, d - react_d)
        v = min(self.vmax, math.sqrt(self.end_speed ** 2 + 2 * self.decel * brake_d))
        if self.end_speed == 0.0:
            v = min(v, self.kp * d)
        if self.rotation_cap and abs(eth) > math.radians(2):
            t_rot = abs(eth) / self.wmax
            v = min(v, max(d / t_rot, 0.4))
        vx, vy = (ex / d * v, ey / d * v) if d > 1e-6 else (0.0, 0.0)
        brake_th = max(0.0, abs(eth) - abs(cw) * self.wreact)
        w = math.copysign(min(self.wmax, math.sqrt(2 * self.wdecel * brake_th), self.kp_w * abs(eth)), eth)
        dvx, dvy, dw = vx - cvx, vy - cvy, w - cw
        if self.perp_first and d > 1e-6:
            ux, uy = ex / d, ey / d
            a_perp = -dvx * uy + dvy * ux  # change needed across the goal direction
            a_along = dvx * ux + dvy * uy
            lim = self.accel * dt
            p = _clamp(a_perp, -lim, lim)
            room = math.sqrt(max(0.0, lim * lim - p * p))
            q = _clamp(a_along, -room, room)
            dvx, dvy = q * ux - p * uy, q * uy + p * ux
            dw = _clamp(dw, -self.waccel * dt, self.waccel * dt)
        elif self.budget is None:
            dv = math.hypot(dvx, dvy)
            if dv > self.accel * dt:
                dvx, dvy = dvx * self.accel * dt / dv, dvy * self.accel * dt / dv
            dw = _clamp(dw, -self.waccel * dt, self.waccel * dt)
        else:
            # individual caps first, then the shared budget
            dv = math.hypot(dvx, dvy)
            if dv > self.accel * dt:
                dvx, dvy = dvx * self.accel * dt / dv, dvy * self.accel * dt / dv
            dw = _clamp(dw, -self.waccel * dt, self.waccel * dt)
            use = math.hypot(dvx, dvy) + abs(dw) * self.radius
            cap = self.budget * dt
            if use > cap:
                k = cap / use
                dvx, dvy, dw = dvx * k, dvy * k, dw * k
        self.cmd = (cvx + dvx, cvy + dvy, cw + dw)
        return self.cmd


# ---------------------------------------------------------------------------- 581
class Trailblazer581X(Trailblazer581):
    """581 law + knobs: floor (their 0.5 m/s ramp floor), end_speed (pass-through), vec_accel
    (add a vector acceleration limit on the command; None = as published)."""

    name = "Trailblazer581X"

    def __init__(self, floor=0.5, end_speed=0.0, vec_accel=None, **kw):
        super().__init__(**kw)
        self.floor, self.end_speed, self.vec_accel = floor, end_speed, vec_accel
        self.cmd = (0.0, 0.0)

    def reset(self, pose, meas_vel):
        super().reset(pose, meas_vel)
        self.cmd = (meas_vel[0], meas_vel[1])

    def step(self, pose, goal, meas_vel, dt):
        ex, ey = goal[0] - pose[0], goal[1] - pose[1]
        d = math.hypot(ex, ey)
        ve = self.end_speed
        v = abs(self.kp * d) if ve == 0.0 else max(self.kp * d, ve)
        eth = abs(wrap(goal[2] - pose[2]))
        if self.rotation_cap:
            work = max(0.0, eth - self.ang_tol)
            if work > 0:
                cap = max(d / (work / self.wmax), 0.4)
                v = min(v, cap)
                if d <= self.lin_tol:
                    v = 0.0
        v = min(v, self.vmax)
        v = min(v, max(self.last_cmd_v + self.amax * min(dt, 0.04), self.floor))
        v = min(v, math.sqrt(ve * ve + 2 * self.amax * d))
        self.last_cmd_v = v
        w = self.theta.calculate(pose[2], goal[2], dt)
        vx, vy = (ex / d * v, ey / d * v) if d > 1e-6 else (0.0, 0.0)
        if self.vec_accel is not None:
            vx, vy = _vec_limit(self.cmd, (vx, vy), self.vec_accel, dt)
            self.last_cmd_v = math.hypot(vx, vy)
        self.cmd = (vx, vy)
        return (vx, vy, w)


# ---------------------------------------------------------------------------- 6328
class Rolling6328PT(Rolling6328):
    """6328 rolling profile with a goal velocity (pass-through). end_speed = 0 is the original."""

    name = "Rolling6328PT"

    def __init__(self, end_speed=0.0, **kw):
        super().__init__(**kw)
        self.end_speed = end_speed

    def step(self, pose, goal, meas_vel, dt):
        if self.end_speed == 0.0:
            return super().step(pose, goal, meas_vel, dt)
        cd = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
        dx, dy = goal[0] - self.sp_xy[0], goal[1] - self.sp_xy[1]
        dn = math.hypot(dx, dy)
        spv = (self.sp_vel[0] * dx + self.sp_vel[1] * dy) / dn if dn > 1e-9 else 0.0
        spv = max(spv, self.min_sp_vel)
        p, v = self.profile.calculate(dt, (dn, -spv), (0.0, -self.end_speed))
        scalar = self.kp * (p - cd) + v
        ux, uy = ((pose[0] - goal[0]) / cd, (pose[1] - goal[1]) / cd) if cd > 1e-9 else (0.0, 0.0)
        if dn > 1e-9:
            self.sp_xy = (goal[0] - dx / dn * p, goal[1] - dy / dn * p)
            self.sp_vel = (dx / dn * -v, dy / dn * -v)
        w = self.theta.calculate(pose[2], goal[2], dt) + self.theta.sp[1]
        return (ux * scalar, uy * scalar, w)


# ---------------------------------------------------------------------------- WPILib per axis
class _ProfiledPIDGoalVel(ProfiledPID):
    def calculate_v(self, meas, goal, goal_v, dt):
        sp_p, sp_v = self.sp
        self.sp = self.profile.calculate(dt, (sp_p, sp_v), (goal, goal_v))
        return self.kp * (self.sp[0] - meas)


class ProfiledPerAxisPT(ProfiledPerAxis):
    """Per-axis profiles with a goal velocity along the line to the goal (pass-through)."""

    name = "ProfiledPerAxisPT"

    def __init__(self, end_speed=0.0, kp=3.0, vmax=2.5, amax=3.0, kp_w=4.0, wmax=math.pi, walpha=2 * math.pi):
        super().__init__(kp, vmax, amax, kp_w, wmax, walpha)
        self.x = _ProfiledPIDGoalVel(kp, vmax, amax)
        self.y = _ProfiledPIDGoalVel(kp, vmax, amax)
        self.end_speed = end_speed

    def step(self, pose, goal, meas_vel, dt):
        if self.end_speed == 0.0:
            return super().step(pose, goal, meas_vel, dt)
        ex, ey = goal[0] - pose[0], goal[1] - pose[1]
        d = math.hypot(ex, ey)
        ux, uy = (ex / d, ey / d) if d > 1e-6 else (0.0, 0.0)
        vx = self.x.calculate_v(pose[0], goal[0], self.end_speed * ux, dt) + self.x.sp[1]
        vy = self.y.calculate_v(pose[1], goal[1], self.end_speed * uy, dt) + self.y.sp[1]
        w = self.t.calculate(pose[2], goal[2], dt) + self.t.sp[1]
        return (vx, vy, w)


class LinearPathPT(LinearPathPID):
    """CTRE LinearPath with an end velocity (pass-through)."""

    name = "LinearPathPT"

    def __init__(self, end_speed=0.0, **kw):
        super().__init__(**kw)
        self.end_speed = end_speed

    def step(self, pose, goal, meas_vel, dt):
        if self.goal is None:
            self.goal = goal
        self.t += dt
        s, g = self.start, self.goal
        L = math.hypot(g[0] - s[0], g[1] - s[1])
        ux, uy = ((g[0] - s[0]) / L, (g[1] - s[1]) / L) if L > 1e-9 else (0.0, 0.0)
        v0 = self.v0[0] * ux + self.v0[1] * uy
        p, v = self.lin.calculate(self.t, (0.0, v0), (L, self.end_speed))
        if self.end_speed > 0 and p >= L - 1e-9:  # past the end: keep going at end speed
            p, v = L + self.end_speed * (self.t - self.lin.total_time((0.0, v0), (L, self.end_speed))), self.end_speed
        dth = wrap(g[2] - s[2])
        pa, va = self.ang.calculate(self.t, (0.0, self.v0[2]), (dth, 0.0))
        spx, spy, spt = s[0] + ux * p, s[1] + uy * p, wrap(s[2] + pa)
        vx = ux * v + self.kp * (spx - pose[0])
        vy = uy * v + self.kp * (spy - pose[1])
        w = va + self.kp_w * wrap(spt - pose[2])
        return (vx, vy, w)


# ---------------------------------------------------------------------------- generic fix
class VecLimited:
    """Wrap any law: vector acceleration limit on its translation output (seeded from the measured
    velocity) and a rate limit on omega. Shows whether a law's failure is just its missing limiter."""

    def __init__(self, base, accel=5.0, alpha=12.0):
        self.inner = base()
        self.accel, self.alpha = accel, alpha
        self.name = getattr(self.inner, "name", "law") + "+limit"
        self.cmd = (0.0, 0.0, 0.0)

    @property
    def end_speed(self):
        return getattr(self.inner, "end_speed", 0.0)

    @end_speed.setter
    def end_speed(self, v):
        if hasattr(self.inner, "end_speed"):
            self.inner.end_speed = v

    def reset(self, pose, meas_vel):
        self.inner.reset(pose, meas_vel)
        self.cmd = tuple(meas_vel)

    def step(self, pose, goal, meas_vel, dt):
        vx, vy, w = self.inner.step(pose, goal, meas_vel, dt)
        vx, vy = _vec_limit(self.cmd, (vx, vy), self.accel, dt)
        w = self.cmd[2] + _clamp(w - self.cmd[2], -self.alpha * dt, self.alpha * dt)
        self.cmd = (vx, vy, w)
        return self.cmd
