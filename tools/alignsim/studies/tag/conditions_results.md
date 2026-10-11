# Operating-conditions sweep

`python3 alignsim/studies/tag/conditions.py 60 2 [sweep]`, default world, tracker = current `odom KF`
(the first `TagTracker.java`) and `geo` (the shipped one, without per-camera correction). Old controller
tune (accel 6). Run before the core plant fix (a speed x 2 ms encoder offset while moving); the skid study
showed it doesn't change alignment success.

Columns: `succ%` = declared done AND truly < 2 cm and < 1 deg; `false%` = declared done but truly outside;
`pos/lat/yaw` are true errors at done (cm, deg); `jitter` cm/s at the end; `slip_ms` total wheel-slip time;
`pk_acc` peak commanded accel (m/s^2).

**Reading it:**
- **Two corner cameras are the big lever.** One center camera gets 73-78%: it sees the tag head-on, where
  the single-tag yaw solve is ambiguous. One corner camera already gets 98-100%. Toe-in angle doesn't matter.
- **Frame rate (20-120 fps), pixel noise (0.2-1.5 px) and handoff speed (0-3 m/s) barely matter.**
- **100 Hz odometry (SystemCore without a CANivore) doesn't hurt alignment.**
- **Timestamps:** a + bias (capture time computed too late) beyond the real latency clamps to "no latency
  compensation at all", so the 50 and 100 ms rows are identical. Even that only grows the median from
  0.4 to 0.8 cm. Timestamps too early by 100 ms cost 7%. Errors land along the approach (depth), because
  the last frames come in slowly. Rotating while viewing (not tested here) is where timing matters more;
  the robot spin test covers that.

**camera layout, n=60**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 2 corners, toe-in 10deg (default) / odom KF | 100.0 |   0.0 | 100.0 |  1.85 |   0.39 |   0.79 |   0.59 |   0.64 |  0.04 |   23.4 |   6.0 |
| 2 corners, toe-in 10deg (default) / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.39 |   0.82 |   0.63 |   0.69 |  0.03 |   15.0 |   6.0 |
| 2 corners, toe-in 25deg / odom KF | 100.0 |   0.0 | 100.0 |  1.86 |   0.41 |   0.81 |   0.68 |   0.67 |  0.05 |   31.6 |   6.0 |
| 2 corners, toe-in 25deg / geo | 100.0 |   0.0 | 100.0 |  1.85 |   0.39 |   0.84 |   0.67 |   0.64 |  0.08 |   15.2 |   6.0 |
| 2 corners, parallel / odom KF | 100.0 |   0.0 | 100.0 |  1.85 |   0.38 |   0.75 |   0.58 |   0.58 |  0.00 |   24.5 |   6.0 |
| 2 corners, parallel / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.37 |   0.75 |   0.51 |   0.61 |  0.00 |   15.3 |   6.0 |
| 1 center camera / odom KF |  73.3 |  26.7 | 100.0 |  1.87 |   0.74 |   1.69 |   1.65 |   1.70 |  0.00 |   37.8 |   6.0 |
| 1 center camera / geo |  78.3 |  21.7 | 100.0 |  1.86 |   0.66 |   1.63 |   1.59 |   1.67 |  0.00 |   17.9 |   6.0 |
| 1 corner camera / odom KF |  98.3 |   1.7 | 100.0 |  1.86 |   0.44 |   0.96 |   0.82 |   0.83 |  0.02 |   35.1 |   6.0 |
| 1 corner camera / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.46 |   0.93 |   0.89 |   0.80 |  0.01 |   18.0 |   6.0 |

**frame rate, n=60**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 20 fps / odom KF | 100.0 |   0.0 | 100.0 |  1.86 |   0.39 |   0.73 |   0.54 |   0.68 |  0.03 |   32.4 |   6.0 |
| 20 fps / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.40 |   0.71 |   0.51 |   0.56 |  0.03 |   16.0 |   6.0 |
| 30 fps / odom KF | 100.0 |   0.0 | 100.0 |  1.86 |   0.37 |   0.73 |   0.56 |   0.61 |  0.01 |   33.2 |   6.0 |
| 30 fps / geo | 100.0 |   0.0 | 100.0 |  1.85 |   0.36 |   0.76 |   0.59 |   0.65 |  0.04 |   15.6 |   6.0 |
| 60 fps / odom KF | 100.0 |   0.0 | 100.0 |  1.85 |   0.39 |   0.79 |   0.59 |   0.64 |  0.04 |   23.4 |   6.0 |
| 60 fps / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.39 |   0.82 |   0.63 |   0.69 |  0.03 |   15.0 |   6.0 |
| 120 fps / odom KF | 100.0 |   0.0 | 100.0 |  1.85 |   0.43 |   0.88 |   0.65 |   0.70 |  0.02 |   22.1 |   6.0 |
| 120 fps / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.42 |   0.89 |   0.66 |   0.71 |  0.01 |   15.1 |   6.0 |

**pixel noise, n=60**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.2 px / odom KF | 100.0 |   0.0 | 100.0 |  1.86 |   0.40 |   0.79 |   0.58 |   0.64 |  0.02 |   21.1 |   6.0 |
| 0.2 px / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.39 |   0.82 |   0.61 |   0.65 |  0.02 |   15.4 |   6.0 |
| 0.35 px / odom KF | 100.0 |   0.0 | 100.0 |  1.85 |   0.39 |   0.79 |   0.59 |   0.64 |  0.04 |   23.4 |   6.0 |
| 0.35 px / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.39 |   0.82 |   0.63 |   0.69 |  0.03 |   15.0 |   6.0 |
| 0.7 px / odom KF | 100.0 |   0.0 | 100.0 |  1.86 |   0.40 |   0.80 |   0.64 |   0.63 |  0.04 |   27.0 |   6.0 |
| 0.7 px / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.43 |   0.83 |   0.66 |   0.68 |  0.03 |   15.2 |   6.0 |
| 1.5 px / odom KF | 100.0 |   0.0 | 100.0 |  1.87 |   0.37 |   0.77 |   0.68 |   0.55 |  0.09 |   33.1 |   6.0 |
| 1.5 px / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.44 |   0.85 |   0.57 |   0.55 |  0.04 |   15.5 |   6.0 |

**handoff speed, n=60**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.0 m/s / odom KF | 100.0 |   0.0 | 100.0 |  2.07 |   0.38 |   0.85 |   0.63 |   0.66 |  0.04 |   52.9 |   6.0 |
| 0.0 m/s / geo | 100.0 |   0.0 | 100.0 |  2.08 |   0.37 |   0.79 |   0.62 |   0.65 |  0.02 |   50.9 |   6.0 |
| 1.5 m/s / odom KF | 100.0 |   0.0 | 100.0 |  1.85 |   0.39 |   0.79 |   0.59 |   0.64 |  0.04 |   23.4 |   6.0 |
| 1.5 m/s / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.39 |   0.82 |   0.63 |   0.69 |  0.03 |   15.0 |   6.0 |
| 3.0 m/s / odom KF | 100.0 |   0.0 | 100.0 |  1.75 |   0.39 |   0.83 |   0.58 |   0.62 |  0.02 |   16.3 |   6.0 |
| 3.0 m/s / geo | 100.0 |   0.0 | 100.0 |  1.75 |   0.41 |   0.77 |   0.58 |   0.64 |  0.01 |   12.9 |   6.0 |

**odometry rate, n=60**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 250 Hz (CANivore) / odom KF | 100.0 |   0.0 | 100.0 |  1.85 |   0.39 |   0.79 |   0.59 |   0.64 |  0.04 |   23.4 |   6.0 |
| 250 Hz (CANivore) / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.39 |   0.82 |   0.63 |   0.69 |  0.03 |   15.0 |   6.0 |
| 100 Hz (SystemCore CAN 2.0) / odom KF | 100.0 |   0.0 | 100.0 |  1.84 |   0.39 |   0.81 |   0.57 |   0.64 |  0.04 |   30.6 |   6.0 |
| 100 Hz (SystemCore CAN 2.0) / geo | 100.0 |   0.0 | 100.0 |  1.83 |   0.42 |   0.81 |   0.59 |   0.63 |  0.04 |   18.1 |   6.0 |

**clock bias, + = computed capture time too late, n=60**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| 0 ms @ 1.5 m/s / odom KF | 100.0 |   0.0 | 100.0 |  1.85 |   0.39 |   0.80 |   0.61 |   0.67 |  0.02 |   23.4 |   6.0 |
| 0 ms @ 1.5 m/s / geo | 100.0 |   0.0 | 100.0 |  1.86 |   0.40 |   0.84 |   0.62 |   0.63 |  0.04 |   15.0 |   6.0 |
| 20 ms @ 1.5 m/s / odom KF | 100.0 |   0.0 | 100.0 |  1.80 |   0.57 |   1.08 |   0.60 |   0.57 |  0.00 |   24.4 |   6.0 |
| 20 ms @ 1.5 m/s / geo | 100.0 |   0.0 | 100.0 |  1.80 |   0.53 |   1.06 |   0.65 |   0.64 |  0.03 |   14.7 |   6.0 |
| 50 ms @ 1.5 m/s / odom KF |  98.3 |   1.7 | 100.0 |  1.77 |   0.84 |   1.36 |   0.68 |   0.60 |  0.00 |   24.9 |   6.0 |
| 50 ms @ 1.5 m/s / geo |  98.3 |   1.7 | 100.0 |  1.78 |   0.75 |   1.45 |   0.64 |   0.60 |  0.00 |   15.0 |   6.0 |
| 100 ms @ 1.5 m/s / odom KF |  98.3 |   1.7 | 100.0 |  1.77 |   0.84 |   1.36 |   0.68 |   0.60 |  0.00 |   24.9 |   6.0 |
| 100 ms @ 1.5 m/s / geo |  98.3 |   1.7 | 100.0 |  1.78 |   0.75 |   1.45 |   0.64 |   0.60 |  0.00 |   15.0 |   6.0 |
| 0 ms @ 3.0 m/s / odom KF | 100.0 |   0.0 | 100.0 |  1.75 |   0.38 |   0.83 |   0.60 |   0.62 |  0.02 |   16.3 |   6.0 |
| 0 ms @ 3.0 m/s / geo | 100.0 |   0.0 | 100.0 |  1.75 |   0.39 |   0.78 |   0.60 |   0.64 |  0.02 |   12.9 |   6.0 |
| 20 ms @ 3.0 m/s / odom KF | 100.0 |   0.0 | 100.0 |  1.69 |   0.55 |   1.05 |   0.68 |   0.68 |  0.03 |   17.2 |   6.0 |
| 20 ms @ 3.0 m/s / geo | 100.0 |   0.0 | 100.0 |  1.70 |   0.59 |   1.07 |   0.69 |   0.64 |  0.01 |   13.2 |   6.0 |
| 50 ms @ 3.0 m/s / odom KF | 100.0 |   0.0 | 100.0 |  1.67 |   0.76 |   1.41 |   0.69 |   0.63 |  0.02 |   17.4 |   6.0 |
| 50 ms @ 3.0 m/s / geo |  98.3 |   1.7 | 100.0 |  1.68 |   0.81 |   1.41 |   0.67 |   0.66 |  0.00 |   13.9 |   6.0 |
| 100 ms @ 3.0 m/s / odom KF | 100.0 |   0.0 | 100.0 |  1.67 |   0.76 |   1.41 |   0.69 |   0.63 |  0.02 |   17.4 |   6.0 |
| 100 ms @ 3.0 m/s / geo | 100.0 |   0.0 | 100.0 |  1.68 |   0.81 |   1.41 |   0.67 |   0.66 |  0.00 |   13.9 |   6.0 |

**clock bias, timestamps too EARLY (over-compensated), geo tracker, n=60**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| -20 ms (too early) @ 1.5 m/s / geo | 100.0 |   0.0 | 100.0 |  1.92 |   0.55 |   0.98 |   0.55 |   0.66 |  0.02 |   15.4 |   6.0 |
| -50 ms (too early) @ 1.5 m/s / geo | 100.0 |   0.0 | 100.0 |  2.04 |   0.82 |   1.38 |   0.56 |   0.69 |  0.12 |   15.7 |   6.0 |
| -100 ms (too early) @ 1.5 m/s / geo |  93.3 |   6.7 | 100.0 |  2.24 |   1.22 |   2.08 |   0.71 |   0.67 |  0.15 |   15.8 |   6.0 |
| -20 ms (too early) @ 3.0 m/s / geo | 100.0 |   0.0 | 100.0 |  1.81 |   0.55 |   0.98 |   0.54 |   0.67 |  0.05 |   12.9 |   6.0 |
| -50 ms (too early) @ 3.0 m/s / geo | 100.0 |   0.0 | 100.0 |  1.93 |   0.81 |   1.46 |   0.55 |   0.67 |  0.10 |   13.1 |   6.0 |
| -100 ms (too early) @ 3.0 m/s / geo |  93.3 |   6.7 | 100.0 |  2.16 |   1.24 |   2.18 |   0.65 |   0.65 |  0.09 |   20.0 |   6.0 |
