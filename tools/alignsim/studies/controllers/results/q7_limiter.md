# Q7: the 2026-style physics limiter

The team's 2026 `AccelerationLimiter`: the motor current limit is only a hardware ceiling (200 A then,
360 A here), and the command decides how hard to push from physics:
- **traction:** a friction circle, `sqrt(a_lin^2 + (alpha*r)^2) <= TRACTION_ACCEL`, so driving and
  turning share the grip;
- **motors:** when speeding up, at most what four drive motors can add at the current wheel speed,
  from the torque-speed curve at a lower "model current" (150 A; braking isn't limited by it).

`DriveToPose` keeps its separate heading cap (`ANGULAR_ACCEL` 8): heading steps were the main source of
slip in the controller study. `python3 alignsim/studies/controllers/q7_limiter.py 60 4`. The plant's
hardware current limit is 360 A in every row; CG 0.2 m rows include weight transfer.

**Q7 limiter, hardware limit 360 A, from rest, n=60**

| | t 5cm | t settle | over p95 cm | yaw over p95° | slip ms | odo p95 cm | peak cmd accel |
|---|---:|---:|---:|---:|---:|---:|---:|
| CG 0.0 m: fixed accel 10 (current DriveToPose) | 1.39 | 1.48 | 0.0 | 0.0 | 372 | 7.0 | 10.0 |
| CG 0.0 m: physics: traction 10, motor model 150 A | 1.49 | 1.57 | 0.0 | 0.0 | 383 | 5.3 | 10.0 |
| CG 0.0 m: physics, vmax 4.4 | 1.49 | 1.57 | 0.0 | 0.0 | 386 | 5.4 | 10.0 |
| CG 0.0 m: physics, traction 10.8 (1.1 g, 2026) | 1.47 | 1.56 | 0.0 | 0.0 | 395 | 9.4 | 10.8 |
| CG 0.2 m: fixed accel 10 (current DriveToPose) | 1.38 | 1.48 | 0.0 | 0.0 | 797 | 8.8 | 10.0 |
| CG 0.2 m: physics: traction 10, motor model 150 A | 1.48 | 1.57 | 0.0 | 0.0 | 821 | 7.3 | 10.0 |
| CG 0.2 m: physics, vmax 4.4 | 1.48 | 1.57 | 0.0 | 0.0 | 841 | 7.6 | 10.0 |
| CG 0.2 m: physics, traction 10.8 (1.1 g, 2026) | 1.47 | 1.55 | 0.0 | 0.0 | 815 | 10.7 | 10.8 |
**Q7 limiter, hardware limit 360 A, moving start 0.5-3 m/s, n=60**

| | t 5cm | t settle | over p95 cm | yaw over p95° | slip ms | odo p95 cm | peak cmd accel |
|---|---:|---:|---:|---:|---:|---:|---:|
| CG 0.0 m: fixed accel 10 (current DriveToPose) | 1.61 | 1.70 | 0.0 | 0.0 | 437 | 14.2 | 10.0 |
| CG 0.0 m: physics: traction 10, motor model 150 A | 1.61 | 1.71 | 0.0 | 0.0 | 443 | 9.6 | 10.0 |
| CG 0.0 m: physics, vmax 4.4 | 1.60 | 1.70 | 0.0 | 0.0 | 457 | 10.0 | 10.0 |
| CG 0.0 m: physics, traction 10.8 (1.1 g, 2026) | 1.61 | 1.70 | 0.0 | 0.0 | 451 | 19.8 | 10.8 |
| CG 0.2 m: fixed accel 10 (current DriveToPose) | 1.59 | 1.69 | 0.0 | 1.2 | 883 | 17.7 | 10.0 |
| CG 0.2 m: physics: traction 10, motor model 150 A | 1.60 | 1.69 | 0.0 | 0.0 | 897 | 13.8 | 10.0 |
| CG 0.2 m: physics, vmax 4.4 | 1.60 | 1.69 | 0.0 | 0.6 | 937 | 14.1 | 10.0 |
| CG 0.2 m: physics, traction 10.8 (1.1 g, 2026) | 1.60 | 1.69 | 0.0 | 0.0 | 888 | 22.0 | 10.8 |
**Final tag approach, CG 0.2 m, 360 A, hard world, n=100**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| fixed accel 10 (current DriveToPose) |  99.0 |   1.0 | 100.0 |  1.38 |   0.71 |   1.28 |   1.13 |   0.68 |  0.00 |  639.6 |  10.0 |
| physics: traction 10, motor model 150 A |  99.0 |   1.0 | 100.0 |  1.38 |   0.69 |   1.25 |   1.18 |   0.68 |  0.00 |  648.3 |  10.0 |
| physics, vmax 4.4 | 100.0 |   0.0 | 100.0 |  1.37 |   0.70 |   1.28 |   1.14 |   0.72 |  0.01 |  699.8 |  10.0 |
| physics, traction 10.8 (1.1 g, 2026) |  99.0 |   1.0 | 100.0 |  1.38 |   0.69 |   1.28 |   1.14 |   0.68 |  0.00 |  640.7 |  10.8 |
**Final tag approach, CG 0.2 m, 360 A, very hard world, n=100**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| fixed accel 10 (current DriveToPose) |  96.0 |   4.0 | 100.0 |  1.32 |   0.88 |   1.97 |   1.66 |   0.69 |  0.00 |  534.5 |  10.0 |
| physics: traction 10, motor model 150 A |  95.0 |   5.0 | 100.0 |  1.32 |   0.86 |   1.99 |   1.69 |   0.65 |  0.00 |  541.5 |  10.0 |
| physics, vmax 4.4 |  93.0 |   7.0 | 100.0 |  1.31 |   0.88 |   2.09 |   1.70 |   0.67 |  0.00 |  604.3 |  10.0 |
| physics, traction 10.8 (1.1 g, 2026) |  95.0 |   5.0 | 100.0 |  1.33 |   0.90 |   1.91 |   1.68 |   0.66 |  0.00 |  537.1 |  10.8 |

**Reading it:**
- **The physics limiter leaves 15-30% less odometry error** than a fixed accel 10 (from rest 7.0 -> 5.3
  cm; moving starts 14.2 -> 9.6 cm without weight transfer, 17.7 -> 13.8 cm with it). The fixed limit
  let translation use the full 10 m/s^2 with turning on top; the friction circle shares one budget.
- **It costs about 0.09 s on moves from rest that also turn** (same reason), and nothing on moving starts
  or on the final tag approach (99% hard, 95-96% very hard, same time).
- **The motor limit only matters above ~3.5 m/s** (25.8 m/s^2 available below that, 10.4 at 4 m/s,
  4.9 at 4.3). It keeps the command from running ahead of what the motors can deliver near free speed,
  but raising MAX_SPEED to 4.4 gains nothing on 1-6 m moves.
- **1.1 g (the 2026 value) is no faster than 1.0 g and leaves more odometry error** (9.4-22 cm), so
  `TRACTION_ACCEL` is 10.
- **Counting spin-up in the motor check too, as the 2026 code did, was slower.** With the worst-case wheel
  speed `|v| + |omega|*r`, driving fast while turning reads past free speed, so spin-up is blocked until the
  robot slows: moves took 0.08-0.12 s longer (from rest 1.65 vs 1.57 s, moving starts 1.73 vs 1.61 s)
  with the same odometry error. `AccelerationLimiter` counts only the driving part.
- **Motor model from CTRE's dyno data** (later): `AccelerationLimiter` builds its Kraken X60 FOC from the
  dyno numbers (9.362 N·m and 476.1 A at stall, 5785 rpm free; WPILib's built-in is 9.37 / 483 / 5800).
  Trap: leave the free current at 0. `DCMotor` treats all current as torque, so the dyno's 10.4 A free
  current would overstate the push near top speed (11.4 instead of 9.7 m/s^2 at 4 m/s). With 0, the model
  matches the dyno torque line: 26 / 19 / 10 / 4 / 0 m/s^2 at 3 / 3.5 / 4 / 4.3 / 4.53 m/s.
