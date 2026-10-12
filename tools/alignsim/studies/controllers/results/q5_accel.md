# Q5: how hard can VectorBrake accelerate?

`python3 alignsim/studies/controllers/q5_accel.py 60 4` (move bench, perfect localization; 1-6 m moves,
0-180 deg heading change). Heading limits stay at the ship values (6 / 8 rad/s^2) unless noted.
`odo p95` is how far wheel odometry ends up from the truth after the move.

**Q5 accel sweep, from rest, n=60**

| | t 5cm | t settle | over p95 cm | yaw over p95° | slip ms | odo p95 cm |
|---|---:|---:|---:|---:|---:|---:|
| ship: accel 5 / decel 4 | 1.63 | 1.72 | 0.0 | 0.1 | 21 | 0.4 |
| accel 7 / decel 6 | 1.44 | 1.53 | 0.0 | 0.1 | 145 | 0.7 |
| accel 8.5 / decel 7.5 | 1.36 | 1.45 | 0.0 | 0.1 | 337 | 2.0 |
| accel 10 / decel 9 | 1.32 | 1.40 | 0.0 | 0.1 | 582 | 5.2 |
| accel 12 / decel 11 (past grip) | 1.29 | 1.46 | 5.3 | 0.2 | 730 | 5.9 |
| accel 10 / decel 9, slip current 55 A | 1.31 | 1.39 | 0.0 | 0.1 | 64 | 1.5 |
| accel 10 / decel 6, slip current 55 A | 1.39 | 1.49 | 0.0 | 0.1 | 31 | 0.5 |
| ship: accel 5 / decel 4, slip current 55 A | 1.63 | 1.72 | 0.0 | 0.1 | 13 | 0.4 |
| SHIPPED: accel 10 / decel 6, slip current 60 A | 1.38 | 1.48 | 0.0 | 0.1 | 99 | 2.2 |
| accel 10 / decel 6, 60 A, 68 kg robot | 1.39 | 1.48 | 0.0 | 0.1 | 17 | 0.5 |
| accel 10 / decel 9, heading 10/12, wmax 9 | 1.27 | 1.33 | 0.0 | 0.1 | 587 | 8.6 |
| accel 10 / decel 9, vmax 4.4 | 1.33 | 1.40 | 0.0 | 0.1 | 603 | 5.1 |
**Q5 accel sweep, moving start 0.5-3 m/s, n=60**

| | t 5cm | t settle | over p95 cm | yaw over p95° | slip ms | odo p95 cm |
|---|---:|---:|---:|---:|---:|---:|
| ship: accel 5 / decel 4 | 1.89 | 1.98 | 0.0 | 0.1 | 22 | 1.9 |
| accel 7 / decel 6 | 1.66 | 1.76 | 0.0 | 0.1 | 236 | 2.3 |
| accel 8.5 / decel 7.5 | 1.55 | 1.65 | 0.0 | 0.1 | 449 | 3.8 |
| accel 10 / decel 9 | 1.49 | 1.56 | 0.0 | 0.1 | 640 | 12.6 |
| accel 12 / decel 11 (past grip) | 1.43 | 1.70 | 5.3 | 1.7 | 803 | 26.7 |
| accel 10 / decel 9, slip current 55 A | 1.50 | 1.57 | 0.0 | 0.1 | 410 | 11.2 |
| accel 10 / decel 6, slip current 55 A | 1.60 | 1.70 | 0.0 | 0.1 | 388 | 11.5 |
| ship: accel 5 / decel 4, slip current 55 A | 1.89 | 1.98 | 0.0 | 0.1 | 21 | 1.9 |
| SHIPPED: accel 10 / decel 6, slip current 60 A | 1.60 | 1.70 | 0.0 | 0.1 | 398 | 11.6 |
| accel 10 / decel 6, 60 A, 68 kg robot | 1.60 | 1.69 | 0.0 | 0.1 | 342 | 10.7 |
| accel 10 / decel 9, heading 10/12, wmax 9 | 1.50 | 1.57 | 0.0 | 0.1 | 650 | 15.4 |
| accel 10 / decel 9, vmax 4.4 | 1.47 | 1.53 | 0.0 | 0.1 | 676 | 12.6 |

**Final tag approach** (recommended tag pipeline, per-camera calibration, n=100 each; `t_done` s):

| controller, slip current | hard: succ % / t_done / pos p95 cm | very hard: succ % / t_done / pos p95 cm |
|---|---|---|
| accel 5 / decel 4, 120 A (old ship) | 99 / 1.58 / 1.28 | 97 / 1.49 / 1.86 |
| accel 10 / decel 9, 120 A | 97 / 1.26 / 1.56 | 85 / 1.21 / 2.87 |
| accel 10 / decel 9, 55 A | 98 / 1.27 / 1.37 | 89 / 1.23 / 2.25 |
| accel 10 / decel 7.5, 55 A | 99 / 1.34 / 1.22 | 92 / 1.30 / 2.09 |
| **accel 10 / decel 6, 55 A (ships)** | **99 / 1.40 / 1.31** | **98 / 1.34 / 1.93** |
| accel 10 / decel 4, 55 A | 100 / 1.52 / 1.27 | 95 / 1.47 / 1.99 |

**Reading it:**
- **ACCEL 10 is fine if the slip current matches the tires' grip.** At 120 A the motors can push about
  1.8x harder than the tread holds, so 10 m/s^2 slips the wheels (582 ms per move, 5 cm odometry error).
  At 55-60 A it is as clean as the old accel 5, and moves are 0.23-0.28 s faster.
- **Keep DECEL near 6 for precise alignment.** Braking at 9 brings the robot into the last stretch fast;
  in the very hard world (15 ms clock error, 2.5 m/s handoff) depth error grows and success drops from
  97% to 85-89%. Decel 6 keeps 98-99% and is still 0.15-0.2 s faster than the old tune.
- **Hard reversals from a moving start still slip** (~400 ms, ~11 cm odometry error at accel 10 with
  any slip current): the modules have to re-steer. Vision corrects the global pose and the tag tracker
  re-anchors, but a wheel-only auto that starts moves at speed is better with ACCEL ~7.
- **Past grip (12 / 11) is slower to settle and overshoots 5 cm.**
- Faster heading (10 / 12, wmax 9) saves 0.07 s from rest and adds odometry error; max speed 4.4 vs 4.0
  changes nothing on 1-6 m moves.
- The slip current that is clean depends on mass and tread (60 A: 99 ms of slip at 60 kg, 17 ms at
  68 kg). Measure it.
