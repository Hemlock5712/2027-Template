# Headline comparison and approach shaping

`python3 alignsim/studies/tag/final.py 100 2 [approach|headline|gate]`. Every row uses the same 100 seeds.

**Worlds** (errors are 1 sigma per trial unless fixed):
- **default:** element off the map 2.5 cm / 1 deg, camera mount yaw and pitch 0.4 deg, no PnP yaw bias,
  focal length 0.5%, principal point 2 px, clock bias 2 ms, global pose 3 cm off at the handoff, wheel
  radius 1%, start 2.8 m out at 1.5 m/s.
- **hard:** element 4 cm / 2 deg, camera mount yaw and pitch 0.8 deg, PnP yaw bias 1 deg, clock 5 ms,
  global pose 8 cm off (the rest as default).
- **very hard:** element 8 cm / 4 deg, camera yaw 1.5 deg, pitch 1 deg, PnP yaw 1.5 deg, focal 1%,
  principal point 4 px, clock 15 ms, pixel noise 0.6 px, global pose 15 cm off, 2.5 m/s handoff.

**RECOMMENDED** = geo tracker (now `TagTracker.java`) + per-camera correction from the park-and-back-off
calibration + the retuned `VectorBrake` (now `DriveToPose.java`).

**Reading it:**
- Only the per-camera-calibrated geo tracker holds up as the world gets worse: 99-100 / 99 / 96-97% (two runs), against
  100 / 72 / 46% for the first `TagTracker.java` and 57 / 31 / 15% for any global-pose approach.
- Fusing only the target tag into the global pose (581 / 254) is no better than all tags here: the error
  is in the element and the camera, not in which tags are fused.
- The retuned controller finishes 0.28 s sooner with zero slip and the same accuracy.
- Approach shaping (staged point on the normal, or 6328's sliding target) gives 100% for +0.10-0.16 s.
  Direct handoff is 99%. Worth it only if the element blocks a diagonal approach.
- The slow-zone row clamps the speed after the controller, which defeats its accel limit (51 m/s^2
  steps, 87 ms of slip). If you want a slow zone, lower the controller's max speed instead.
- The done gate doesn't matter: the controller parks within 3 mm of where it thinks the goal is, so the
  remaining 1-3% are estimate errors, not gate errors.
- The shaped row was first run with a sign error in the port of 6328's side shift (37% success, a false
  resting point 0.4 m out and 0.56 m to the side). The table shows the rerun after the fix.

**Approach shaping, hard world, recommended tracker, n=100**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| handoff (direct) |  99.0 |   1.0 | 100.0 |  1.58 |   0.71 |   1.27 |   1.09 |   0.68 |  0.00 |    0.0 |   5.0 |
| staged 0.6 m (Nav2) | 100.0 |   0.0 | 100.0 |  1.74 |   0.74 |   1.25 |   1.07 |   0.58 |  0.00 |    0.0 |   5.0 |
| shaped (6328 sliding target, sign fixed, rerun) | 100.0 |   0.0 | 100.0 |  1.68 |   0.70 |   1.30 |   1.11 |   0.70 |  0.00 |    0.0 |   5.0 |
| slow zone 0.4 m @ 0.6 m/s |  99.0 |   1.0 | 100.0 |  1.73 |   0.73 |   1.32 |   1.10 |   0.72 |  0.00 |   86.6 |  51.6 |

**Headline: default world, n=100**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| global (all tags) |  57.0 |  43.0 | 100.0 |  1.85 |   0.94 |   2.57 |   2.56 |   2.58 |  0.04 |   16.7 |   6.0 |
| global (target tag only, 581/254) |  57.0 |  43.0 | 100.0 |  1.86 |   0.94 |   2.52 |   2.51 |   2.57 |  0.02 |   16.7 |   6.0 |
| raw tag pose (old DriveToTag) |  10.0 |   0.0 |  10.0 |  1.73 |   0.52 |   1.01 |   0.72 |   0.75 | 16.54 |  168.0 |   6.0 |
| 6328 txTy (map yaw) |  56.0 |  44.0 | 100.0 |  1.86 |   1.05 |   2.60 |   2.57 |   2.55 |  0.01 |   16.7 |   6.0 |
| fused history (2026 samplePoseAt style) | 100.0 |   0.0 | 100.0 |  1.86 |   0.42 |   0.81 |   0.68 |   0.61 |  0.04 |   26.4 |   6.0 |
| odom KF (current TagTracker.java) | 100.0 |   0.0 | 100.0 |  1.86 |   0.37 |   0.73 |   0.59 |   0.63 |  0.03 |   26.5 |   6.0 |
| geo KF + per-camera cal, old controller tune | 100.0 |   0.0 | 100.0 |  1.86 |   0.48 |   0.97 |   0.68 |   0.70 |  0.03 |   21.0 |   6.0 |
| RECOMMENDED: + retuned VectorBrake |  99.0 |   1.0 | 100.0 |  1.58 |   0.47 |   0.97 |   0.69 |   0.78 |  0.00 |    0.1 |   5.0 |

**Headline: hard world, n=100**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| global (all tags) |  31.0 |  69.0 | 100.0 |  1.86 |   1.46 |   4.26 |   4.25 |   4.41 |  0.12 |   19.4 |   6.0 |
| global (target tag only, 581/254) |  31.0 |  69.0 | 100.0 |  1.86 |   1.47 |   4.28 |   4.27 |   4.41 |  0.18 |   19.4 |   6.0 |
| raw tag pose (old DriveToTag) |   9.0 |  10.0 |  19.0 |  1.72 |   1.48 |   2.35 |   2.23 |   2.57 | 17.18 |  175.3 |   6.0 |
| 6328 txTy (map yaw) |  31.0 |  69.0 | 100.0 |  1.86 |   1.54 |   4.53 |   4.52 |   4.42 |  0.11 |   19.4 |   6.0 |
| fused history (2026 samplePoseAt style) |  71.0 |  29.0 | 100.0 |  1.86 |   0.80 |   1.66 |   1.58 |   1.84 |  0.15 |   30.3 |   6.0 |
| odom KF (current TagTracker.java) |  72.0 |  28.0 | 100.0 |  1.86 |   0.75 |   1.65 |   1.54 |   1.89 |  0.06 |   30.5 |   6.0 |
| geo KF + per-camera cal, old controller tune | 100.0 |   0.0 | 100.0 |  1.86 |   0.72 |   1.32 |   1.18 |   0.52 |  0.09 |   19.1 |   6.0 |
| RECOMMENDED: + retuned VectorBrake |  99.0 |   1.0 | 100.0 |  1.58 |   0.71 |   1.27 |   1.09 |   0.68 |  0.00 |    0.0 |   5.0 |

**Headline: very hard world, n=100**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| global (all tags) |  15.0 |  85.0 | 100.0 |  1.78 |   2.93 |   8.91 |   8.89 |   8.88 |  0.26 |   19.0 |   6.0 |
| global (target tag only, 581/254) |  16.0 |  84.0 | 100.0 |  1.78 |   2.91 |   8.99 |   8.88 |   8.87 |  0.32 |   19.0 |   6.0 |
| raw tag pose (old DriveToTag) |   6.0 |  24.0 |  30.0 |  1.65 |   1.83 |   3.40 |   3.23 |   4.08 | 16.09 |  175.8 |   6.0 |
| 6328 txTy (map yaw) |  15.0 |  85.0 | 100.0 |  1.80 |   2.92 |   9.35 |   9.21 |   8.88 |  0.32 |   19.0 |   6.0 |
| fused history (2026 samplePoseAt style) |  45.0 |  55.0 | 100.0 |  1.78 |   1.22 |   2.70 |   2.64 |   3.02 |  0.31 |   28.6 |   6.0 |
| odom KF (current TagTracker.java) |  46.0 |  54.0 | 100.0 |  1.79 |   1.14 |   2.68 |   2.56 |   3.01 |  0.20 |   28.5 |   6.0 |
| geo KF + per-camera cal, old controller tune |  95.0 |   5.0 | 100.0 |  1.79 |   0.81 |   1.91 |   1.84 |   0.59 |  0.19 |   19.6 |   6.0 |
| RECOMMENDED: + retuned VectorBrake |  96.0 |   4.0 | 100.0 |  1.49 |   0.88 |   1.92 |   1.85 |   0.64 |  0.00 |    0.0 |   5.0 |

**Done gate, recommended pipeline, n=100** (after the core plant fix)

| world / gate | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| default / 2 cm, 1.5 deg | 100.0 |   0.0 | 100.0 |  1.58 |   0.46 |   0.94 |   0.68 |   0.73 |  0.00 |    0.1 |   5.0 |
| default / 1 cm, 0.75 deg | 100.0 |   0.0 | 100.0 |  1.58 |   0.46 |   0.94 |   0.68 |   0.73 |  0.00 |    0.1 |   5.0 |
| hard / 2 cm, 1.5 deg |  99.0 |   1.0 | 100.0 |  1.58 |   0.70 |   1.28 |   1.11 |   0.69 |  0.02 |    0.0 |   5.0 |
| hard / 1 cm, 0.75 deg |  99.0 |   1.0 | 100.0 |  1.58 |   0.70 |   1.28 |   1.11 |   0.69 |  0.02 |    0.0 |   5.0 |
| very hard / 2 cm, 1.5 deg |  97.0 |   3.0 | 100.0 |  1.49 |   0.87 |   1.86 |   1.78 |   0.60 |  0.00 |    0.0 |   5.0 |
| very hard / 1 cm, 0.75 deg |  97.0 |   3.0 | 100.0 |  1.49 |   0.87 |   1.86 |   1.78 |   0.60 |  0.00 |    0.0 |   5.0 |
