# Q6: physics and timing (weight transfer, slip current, loop rate, force feedforward)

Prompted by two robot-side points from the team: (1) a low drive slip current starves the wheels that
weight transfer loads up, so 5712 keeps it at about 2x the real grip and limits acceleration in the
command instead; (2) faster control loops or onboard (1 kHz) acceleration control may allow more
aggressive, more accurate driving.

**New plant physics** (default off; on only in this study):
- `cg_height`: weight transfer. Each wheel's load follows the chassis acceleration with a 30 ms lag.
- `drive_ramp`: an onboard per-motor setpoint ramp with acceleration feedforward (Motion Magic velocity).
- per-module force feedforward on the request (CTRE wheel force feedforwards), either split equally or
  in proportion to each wheel's expected load.

`python3 alignsim/studies/controllers/q6_physics.py 4`: five fixed 3 m moves with 0-180 deg turns, from
rest, VectorBrake with decel = accel - 1.

**CG height 0.0 m** - mean slip ms / mean time to 2 cm s / max odometry error cm (decel = accel - 1)

| setup | accel 6 | accel 8 | accel 10 |
|---|---|---|---|
| 50 Hz, 120 A | 53 / 1.66 / 0.3 | 183 / 1.50 / 0.9 | 567 / 1.39 / 5.0 |
| 50 Hz, 60 A (low slip current) | 4 / 1.66 / 0.2 | 10 / 1.51 / 0.2 | 127 / 1.41 / 2.5 |
| 250 Hz, 120 A | 0 / 1.70 / 0.1 | 12 / 1.55 / 0.2 | 318 / 1.47 / 4.3 |
| 50 Hz + onboard ramp = accel (Motion Magic) | 498 / 1.84 / 6.5 | 463 / 1.63 / 5.6 | 953 / 1.47 / 8.8 |
| 250 Hz + equal force FF | 49 / 1.78 / 0.4 | 113 / 1.63 / 0.7 | 428 / 1.58 / 11.4 |
| 50 Hz + load-split force FF | 156 / 1.75 / 0.6 | 290 / 1.61 / 1.7 | 580 / 1.54 / 12.8 |
| 250 Hz + load-split force FF | 49 / 1.78 / 0.4 | 113 / 1.63 / 0.7 | 428 / 1.58 / 11.4 |

**CG height 0.2 m** - mean slip ms / mean time to 2 cm s / max odometry error cm (decel = accel - 1)

| setup | accel 6 | accel 8 | accel 10 |
|---|---|---|---|
| 50 Hz, 120 A | 568 / 1.65 / 1.4 | 913 / 1.47 / 2.3 | 833 / 1.26 / 4.4 |
| 50 Hz, 60 A (low slip current) | 582 / 1.65 / 1.4 | 956 / 1.45 / 4.1 | 1022 / 1.70 / 7.4 |
| 250 Hz, 120 A | 376 / 1.70 / 1.2 | 890 / 1.52 / 2.1 | 794 / 1.41 / 4.1 |
| 50 Hz + onboard ramp = accel (Motion Magic) | 886 / 1.86 / 17.0 | 1094 / 1.63 / 9.4 | 1007 / 1.49 / 7.2 |
| 250 Hz + equal force FF | 446 / 1.78 / 1.6 | 896 / 1.63 / 3.2 | 796 / 1.55 / 8.3 |
| 50 Hz + load-split force FF | 189 / 1.75 / 0.6 | 485 / 1.61 / 1.9 | 714 / 1.54 / 12.4 |
| 250 Hz + load-split force FF | 98 / 1.78 / 0.4 | 295 / 1.64 / 1.1 | 599 / 1.58 / 11.3 |

**CG height 0.3 m** - mean slip ms / mean time to 2 cm s / max odometry error cm (decel = accel - 1)

| setup | accel 6 | accel 8 | accel 10 |
|---|---|---|---|
| 50 Hz, 120 A | 1124 / 1.63 / 2.2 | 1011 / 1.37 / 4.6 | 895 / 1.25 / 7.1 |
| 50 Hz, 60 A (low slip current) | 1125 / 1.64 / 2.2 | 1142 / 1.58 / 8.0 | 1232 / 1.92 / 19.8 |
| 250 Hz, 120 A | 964 / 1.69 / 2.9 | 1002 / 1.47 / 4.4 | 888 / 1.30 / 6.4 |
| 50 Hz + onboard ramp = accel (Motion Magic) | 1332 / 1.84 / 21.9 | 1161 / 1.60 / 14.7 | 1071 / 1.45 / 11.6 |
| 250 Hz + equal force FF | 872 / 1.78 / 3.5 | 1006 / 1.62 / 5.4 | 876 / 1.53 / 9.7 |
| 50 Hz + load-split force FF | 444 / 1.75 / 0.9 | 636 / 1.61 / 3.5 | 818 / 1.54 / 12.5 |
| 250 Hz + load-split force FF | 234 / 1.78 / 0.7 | 528 / 1.64 / 2.3 | 784 / 1.57 / 11.4 |

**Final tag approach with the shipped tune, 120 A vs 60 A, with and without weight transfer**
(recommended tag pipeline, n=100; `python3 alignsim/studies/controllers/q6_physics.py 4 world`):

**Final tag approach, hard world, n=100**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| CG 0.0 m, 120 A: accel 5 / decel 4 |  99.0 |   1.0 | 100.0 |  1.58 |   0.70 |   1.28 |   1.11 |   0.69 |  0.02 |    0.0 |   5.0 |
| CG 0.0 m, 120 A: accel 8 / decel 6 | 100.0 |   0.0 | 100.0 |  1.41 |   0.71 |   1.22 |   1.12 |   0.69 |  0.01 |   83.5 |   8.0 |
| CG 0.0 m, 120 A: accel 10 / decel 6 |  99.0 |   1.0 | 100.0 |  1.40 |   0.68 |   1.31 |   1.07 |   0.59 |  0.00 |  209.3 |  10.0 |
| CG 0.0 m, 60 A: accel 10 / decel 6 | 100.0 |   0.0 | 100.0 |  1.40 |   0.68 |   1.26 |   1.09 |   0.59 |  0.00 |    4.7 |  10.0 |
| CG 0.2 m, 120 A: accel 5 / decel 4 |  99.0 |   1.0 | 100.0 |  1.58 |   0.69 |   1.25 |   1.10 |   0.69 |  0.00 |   76.2 |   5.0 |
| CG 0.2 m, 120 A: accel 8 / decel 6 |  98.0 |   2.0 | 100.0 |  1.39 |   0.68 |   1.23 |   1.16 |   0.62 |  0.02 |  670.1 |   8.0 |
| CG 0.2 m, 120 A: accel 10 / decel 6 |  99.0 |   1.0 | 100.0 |  1.38 |   0.71 |   1.28 |   1.13 |   0.67 |  0.00 |  639.5 |  10.0 |
| CG 0.2 m, 60 A: accel 10 / decel 6 |  98.0 |   2.0 | 100.0 |  1.39 |   0.69 |   1.31 |   1.12 |   0.67 |  0.00 |  677.8 |  10.0 |
**Final tag approach, very hard world, n=100**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| CG 0.0 m, 120 A: accel 5 / decel 4 |  97.0 |   3.0 | 100.0 |  1.49 |   0.87 |   1.86 |   1.78 |   0.60 |  0.00 |    0.0 |   5.0 |
| CG 0.0 m, 120 A: accel 8 / decel 6 |  97.0 |   3.0 | 100.0 |  1.34 |   0.84 |   1.97 |   1.69 |   0.67 |  0.00 |   50.1 |   8.0 |
| CG 0.0 m, 120 A: accel 10 / decel 6 |  98.0 |   2.0 | 100.0 |  1.34 |   0.88 |   1.93 |   1.76 |   0.67 |  0.00 |  109.3 |  10.0 |
| CG 0.0 m, 60 A: accel 10 / decel 6 |  98.0 |   2.0 | 100.0 |  1.34 |   0.85 |   1.94 |   1.77 |   0.69 |  0.03 |    6.6 |  10.0 |
| CG 0.2 m, 120 A: accel 5 / decel 4 |  96.0 |   4.0 | 100.0 |  1.49 |   0.87 |   1.86 |   1.78 |   0.59 |  0.00 |   48.4 |   5.0 |
| CG 0.2 m, 120 A: accel 8 / decel 6 |  95.0 |   5.0 | 100.0 |  1.32 |   0.85 |   1.98 |   1.66 |   0.62 |  0.00 |  547.3 |   8.0 |
| CG 0.2 m, 120 A: accel 10 / decel 6 |  96.0 |   4.0 | 100.0 |  1.32 |   0.88 |   1.97 |   1.66 |   0.69 |  0.00 |  534.5 |  10.0 |
| CG 0.2 m, 60 A: accel 10 / decel 6 |  95.0 |   5.0 | 100.0 |  1.32 |   0.88 |   2.00 |   1.62 |   0.67 |  0.00 |  550.4 |  10.0 |

**Reading it:**
- **Weight transfer is the big physics effect.** With a 0.2 m CG at 6 m/s^2 the front wheels carry ~90 N
  and the rears ~210 N. Velocity-controlled modules push about equally, so the light wheels slip while the
  loaded ones have grip to spare. Equal-force traction limit = mu*g / (1 + mu*h/x): ~6 m/s^2 at h = 0.2 m.
- **A low slip current makes it worse, as the team said.** With a 0.2-0.3 m CG and accel 10, 60 A is
  0.4-0.7 s slower and doubles to triples the odometry error: the loaded wheels can't deliver the force.
  Keep the slip current high (~2x grip) and limit acceleration in the command. (Without weight transfer
  the low current looked best; that was a sim artifact.)
- **120 A vs 360 A (about the Kraken's stall current, i.e. no cap) changes nothing while driving**
  (added later; same moves, CG 0 and 0.2 m, accel 8 and 10, at 50 Hz, 250 Hz and with feedforward: slip,
  time and odometry match to within 2 ms / 0.2 cm). Holding 10 m/s^2 needs ~65 A per wheel and the
  setpoint-step peaks stay under 120 A, so above ~2x grip the cap never acts; the command does the
  limiting. What a higher cap does change is not modeled here: battery draw and brownouts when the drive
  motors push against something.
- **250 Hz control removes the setpoint-step slip.** At 50 Hz each new setpoint makes the stiff velocity
  loop push ~1.4x the average force for a moment. With no weight transfer, 250 Hz is clean up to
  8 m/s^2 (12 ms vs 183 ms of slip).
- **Load-split force feedforward is the per-module win.** With a 0.2 m CG, 250 Hz + load-split
  feedforward cuts slip 4x at 6 m/s^2 (376 -> 98 ms) and 3x at 8 (890 -> 295), and halves odometry error.
  The robot also follows its command closely (lag 0.25 -> 0.05 m/s), so it looks slower only because the
  plain robot brakes later than planned; retune the limits up to get the time back. An equal split
  (what CTRE's WheelForceCalculator does) doesn't help.
- **At 10 m/s^2 with a realistic CG, nothing keeps every wheel gripping:** the light wheels are at their
  limit. Feedforward that close to the limit makes odometry worse (11 cm). Plan around ~8 m/s^2 with
  feedforward, or accept the slip.
- **Don't use Motion Magic velocity on swerve drive motors.** When the robot turns while driving, each
  wheel's speed swings by +-omega*r (about +-2.5 m/s at 6 rad/s), so a wheel must change speed faster than
  the chassis accelerates. A per-motor ramp at the chassis limit can't keep up and the wheels scrub
  (5-10x more slip on turning moves).
- **Alignment doesn't care.** At the tag approach, accel 10 / decel 6 at 120 A gets 99% (hard) and 96-98%
  (very hard) with or without weight transfer, 0.15-0.2 s faster than accel 5. The tracker re-anchors
  after slip early in the approach.
