# Study: driving to and picking up a game piece

**Status:** simulation only, Oct 2026. Untested on a robot. Code is in
[`tools/alignsim/studies/pieces/`](../../../tools/alignsim/studies/pieces/). Every number below comes from
`run_study.py` with fixed seeds.

**Question:** a Limelight neural detector reports each game piece as `tx`/`ty` (degrees, `tx` + = right,
`ty` + = up). What is the best way for our swerve robot to drive to the piece and get it into an intake
on its front?

## The answer

1. **Recommended: steer on the bearing and remember the piece in the odom frame** (strategy **H**).
   - While the piece is in view, the heading target is the *gyro yaw at capture time − tx*. The robot
     drives along that bearing. Speed comes from `ty`: 3.0 m/s far away, down to 1.2 m/s about 0.6 m
     out.
   - It never stops on a measured range. It drives into the piece at about 1.2 m/s.
   - When the piece leaves the image, the robot drives at 1.2 m/s toward the piece's last position
     in the odom frame, from `PieceTracker` upgraded to a Kalman filter, for up to 1 s.
   - **Results:** 99.0 % mean success over 18 stresses, 85 % in the worst one, and **1.05 s** median
     time to intake. The current Java takes 1.34 s; the best `DriveToPose`-based version takes 1.26 s.
2. **Fix the current Java now, whatever else you do** (A → A\*, two constants):
   - Raise `MAX_DISTANCE_METERS` from 4 to 8.
   - Set `INTAKE_REACH_METERS` so the goal is about 12 cm past the point where the intake first
     touches the piece: 0.40 here, instead of 0.45.
   - Result: static-piece success goes from 98.0 % to 99.8 % mean, and the worst static stress
     goes from 84 % to 97 %.
3. **Sideways error is not the problem; range is.** At contact, the piece's estimated position is
   about 1–1.5 cm off sideways (p95) but 3–15 cm off in range. So:
   - never stop on a measured range, and never gate on it;
   - let the intake's depth absorb the range error.
4. **Mount the detector camera so it sees the piece all the way into the intake.** This is the
   biggest lever for rolling pieces: the Java tracker scores 89 % with that camera and 47 % with one
   that goes blind 0.5 m out.
5. **The tx/ty signs are right end to end in the sim.** On the robot, check them with a piece placed
   off to one side (§2). A flipped `tx` sign drops success to 2 %.
6. **No core simulator bugs found** (details in §7).

---

## 1. Setup

### Simulator

The study uses `alignsim`: swerve physics with wheel slip at 500 Hz, the CTRE 250 Hz odometry thread,
a wheel-only odometry buffer (the "odom frame"), a port of WPILib's `PoseEstimator`, and cameras with
latency, clock bias and calibration errors.

Everything specific to pieces lives in `studies/pieces/piece_world.py`; the core modules were not
changed.

- **`PieceCamera`** extends the core `Camera`. The core detector reports the projected piece
  center for as long as that center is inside the image. `PieceCamera` adds a bounding box of the
  piece's apparent size:
  - near the image edge the box is clipped, so its center shifts inward;
  - a box less than 40 % visible is dropped;
  - frames can miss detections at random.

  With the box model off, it matches the core detector draw for draw (tested in `check_signs.py`).
- **`PieceWorld`** extends `World`. It adds rolling pieces with friction, an optional second piece,
  a sideways shove (odometry slip), the intake contact check, and a 50 Hz strategy hook. Thread
  timing is the same as in `World.run`.
- **No AprilTags.** Without tags, the fused pose equals the odom pose. The Java code re-anchors its
  goal onto the fused pose (`getPose() ⊕ (odom⁻¹ ⊕ piece)`), which cancels vision jumps, so this
  changes nothing.

### Robot, camera, piece and intake

| Item | Value |
| --- | --- |
| Robot | 60 kg swerve, square bumper ±0.45 m, Kraken X60, μ 1.15 (0.98 in the slip stresses) |
| Detector camera (default) | Limelight 4 optics (1280×800, fx 740 px), on the center line at x 0.25 m, **0.50 m high, pitched 25° down**, 30 fps, 34 ms mean latency (capture 8 + pipeline 25 ± 4 + network 1) |
| Per-trial calibration errors | Pitch: σ 0.3° plus the stress bias (random sign). Yaw 0.4°, roll 0.2°, position 3 mm, focal length 0.5 %, principal point 2 px, clock bias ±2 ms |
| Detector noise | 2 px on the box center (stress: 1–5 px) |
| Piece | center height 0.10 m as the code assumes (true height ±1 cm per trial; stress: 0.05–0.15 m), horizontal radius 0.07 m |
| Intake | mouth on the front bumper face, width **W = 0.4 m** (piece center within ±W/2). A narrower W is scored from the same runs |
| Start | piece 1.5–3.5 m away, bearing ±20°. The robot is already moving at 0–1.5 m/s, in a direction within ±60° of the piece |

**Outcome of each trial.** The piece center enters the bumper square grown by the piece radius. Then:

- **capture:** it entered through the mouth (|y| ≤ W/2) with a relative speed ≤ 2.5 m/s;
- **too fast:** it entered through the mouth faster than 2.5 m/s;
- **bump:** it hit the bumper anywhere else, so it gets pushed away;
- **timeout:** none of these happened within 4 s.

Success means capture. Narrower intakes are rescored from the same runs (capture with |y| ≤ W/2).
**Miss** is |y| of the piece in the robot frame at contact. **Time** is from the start of the
command to the capture.

**Statistics.** n = 150 trials per cell, seeds 0–149. Every strategy sees the same 150 starts and
errors. Near 90 %, a difference smaller than about 5 points is noise.

### Strategies (all in `strategies.py`)

| Code | What it is |
| --- | --- |
| **A** | `PieceTracker.java` + `DriveToPiece.java` as written: floor projection, odom pose at capture, newest frame only, EMA 0.3, 0.4 m association gate, 4 m range cut, lock after 3 frames. `DriveToPose` (`VectorBrake`) to a goal 0.45 m behind the piece, facing it; holds still until locked |
| A' | A, but the floor point is placed with the odometry pose *now* (no latency compensation) |
| A+ | A with reach 0.35 |
| **A\*** | **Minimal Java fix:** A with an 8 m range cut and reach 0.40 |
| **B0** | 2026-AutoTesting: the "driver" holds heading and drives forward at 2 m/s; code adds `−kP·tx` strafe (kP = 0.04 (m/s)/deg) |
| **B** | Bearing pursuit, as described in "The answer" (IBVS-like, no metric position). When the piece is lost low in the image, it keeps the last bearing at 1.2 m/s for 0.8 s |
| B' | B, using the yaw *now* instead of the yaw at capture |
| **C** | Constant-velocity Kalman filter in the odom frame (every frame, range-shaped noise, Mahalanobis gate), plus lead and velocity feedforward. `DriveToPose` to a goal 0.45 m behind the piece, 4 m cut |
| **D** | The task's hybrid: C's filter, but box-edge detections are rejected so the estimate commits to the last clean fix near the intake. Heading is frozen in the last 0.3 m, and the goal is past the mouth (reach 0.30) |
| **H** | **Recommended:** B while the piece is in view. Once it is lost, drive to C's odom-frame estimate (8 m cut) |
| **R** | Best `DriveToPose`-based version: C with an 8 m cut and reach 0.40 |

### Stresses

| Stress | What changes from base |
| --- | --- |
| pitch 1° / 2° | camera pitch calibration bias of ±1° / ±2° (random sign) |
| noise 1 px / 5 px | detector box-center noise |
| noise 5 px + 30 % dropouts | and 30 % of detections are missed |
| latency 70 ms + clock 20 ms | pipeline 60 ms and a ±20 ms clock-sync bias |
| late | piece 0.9–1.8 m away, robot already at 2–3 m/s in any direction within ±90° |
| blind last 0.5 m | camera 0.6 m high, pitched 5° down: the piece leaves the image about 0.45 m before contact |
| blind + pitch 2° / blind + height | the above, plus a ±2° pitch bias, or a true piece height of 0.05–0.15 m while the code assumes 0.10 |
| height 0.05–0.15 | wrong piece height with the default camera |
| rolling | piece rolls at 0.3–1 m/s in a random direction (0.15 m/s² friction) |
| two pieces | a second piece 0.4–0.8 m from the first (wider than the intake, so aiming between them hits neither) |
| shove / blind + shove | a 900 N sideways push for 0.15 s once the bumper is 0.1–0.6 m from the piece (0.1–0.5 m with the blind camera), at μ × 0.85. The robot slides about 5–15 cm and the odometry doesn't see it |
| ALL (static piece) | blind camera with 40 ms pipeline, pitch ±1°, 3 px, 15 % dropouts, clock ±10 ms, start 1.0–3.5 m at 0–3 m/s within ±90°, height 0.07–0.13 m, a 700 N shove |
| ALL + rolling | ALL, with the piece rolling at 0.3–1 m/s |

Compute: about 26 CPU-minutes, one process.

---

## 2. Sign convention, end to end

`check_signs.py` places a piece at known robot-relative spots, captures a noise-free frame with
`PieceCamera`, and floor-projects it with the believed mount, as `PieceTracker.floorPoint` does.

| Camera yaw | Camera y | Piece (robot x, y) | tx | ty | Floor point (x, y) |
|---|---|---|---:|---:|---|
| 0 | 0 | (1.5, **+0.5**, left) | **−21.0°** | +7.3° | (1.500, **+0.500**) |
| 0 | 0 | (1.5, −0.5, right) | +21.0° | +7.3° | (1.500, −0.500) |
| 0 | 0 | (1.0, 0.0) | 0.0° | **−3.1°** | (1.000, 0.000) |
| 0 | 0 | (2.5, 0.0) | 0.0° | **+14.9°** | (2.500, 0.000) |

What this confirms:

- **A piece on the left gives tx < 0 and a floor point with y > 0.** A nearer piece sits lower in
  the image.
- The ray `(1, −tan tx, tan ty)`, rotated by the mount (WPILib NWU: pitch positive = nose *down*),
  recovers the true point to 1e-6 m. This also holds for yawed and offset cameras and for a rotated
  robot pose.
- In closed loop, every strategy steers *toward* the piece. H's heading target `yaw − tx` turns left
  (positive ω) for a negative tx, which is correct.

What the two possible sign bugs look like (`check_signs.py --closed-loop`, strategy A, base stress):

| Case | Success | What you'd see on the robot |
|---|---:|---|
| Signs correct | 100 % | — |
| `tx` sign flipped | **2 %** | The robot curves away to the mirror-image spot, then chases its own error |
| `tx` flipped, piece within ±2° of straight ahead | 15 % | Looks *almost* fine with a centered piece. **Always test with the piece off to one side** |
| Pitch sign flipped (25° down entered as 25° up) | — | No floor point for any piece 1–3 m away, so the tracker **never locks** and the robot holds still |

**How to check on the real robot** (15 minutes, robot disabled, log `PieceTracker/PieceInOdometry`
and the odometry pose):

1. **Left/right.** Put a piece 1.5 m ahead and 0.5 m to the **left** of the robot center (tape
   measure).
   - The Limelight `tx` must be **negative** (about −18° to −22° for a centered camera).
   - The logged piece, relative to the robot, must be (1.5, **+0.5**) to within a few cm.
   - Move it to the right: both signs must flip.
2. **Near/far.** Put the piece straight ahead at 1.0 m and at 2.5 m. The range error tells you the
   pitch error: **Δpitch ≈ Δrange · H / (H² + d²)**, with H = camera height − piece height and d the
   floor distance from the camera.
   - Example: a +15 cm error at 2.5 m with H = 0.4 m means the pitch is off by about 0.5°.
   - Fix the mount pitch; don't add a fudge factor.
   - If nothing locks at all, the pitch sign is flipped.
3. **Spin.** Rotate in place at about 1 rad/s with the piece in view. Its odom-frame position should
   stay within a few cm. If it smears in the direction of rotation, the capture timestamp is wrong.
4. **Don't trust the camera's own mount convention.**
   - `PieceTracker` reads `getCameraPose_RobotSpace()`.
   - LimelightLib 2.0's javadoc says every space is NWU (y left).
   - 2026 team code using the older helpers treats the Limelight "side" as positive to the **right**
     (Spectrum negates Y before sending), and pitch as positive **up** (581's tag cameras use +15° and
     +20°).
   - Define the mount once in Java as a WPILib `Transform3d`, use it in `PieceTracker`, and send the
     same values to the camera. Steps 1–2 then prove the whole chain.

---

## 3. Results

### Robustness summary

| | A | A' | A+ | A\* | B0 | B | B' | C | D | **H** | R |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| Mean success, 18 stresses, W 0.4 m (%) | 94.8 | 96.6 | 96.8 | 97.1 | 90.1 | 98.8 | 99.0 | 98.5 | 97.5 | **99.0** | 98.9 |
| Worst stress, W 0.4 m (%) | 49 | 59 | 56 | 53 | 41 | 81 | 85 | **87** | 67 | 85 | 84 |
| Mean success, 18 stresses, if W were 0.2 m (%) | 85.3 | 88.2 | 88.4 | 88.5 | 75.9 | 91.9 | 92.9 | 89.5 | 89.0 | 92.2 | 90.5 |
| Mean success, 16 static-piece stresses (%) | 98.0 | 99.2 | 99.2 | **99.8** | 94.2 | **99.8** | **99.8** | 99.1 | 99.4 | **99.8** | **99.8** |
| Median time to intake, base (s) | 1.34 | 1.23 | 1.21 | 1.27 | 1.08 | **1.05** | **1.05** | 1.34 | 1.17 | **1.05** | 1.26 |

### Success % (W = 0.4 m)

| Stress | A | A' | A+ | A\* | B0 | B | B' | C | D | H | R |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| base | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 |
| pitch 1° | 99 | 99 | 99 | 100 | 100 | 100 | 100 | 99 | 100 | 100 | 100 |
| pitch 2° | 96 | 96 | 96 | 100 | 100 | 100 | 100 | 96 | 100 | 100 | 100 |
| noise 1 px | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 |
| noise 5 px | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 |
| noise 5 px + 30 % dropouts | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 |
| latency 70 ms + clock 20 ms | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 |
| late: 0.9–1.8 m, 2–3 m/s | 98 | 97 | 97 | 97 | **41** | 98 | 99 | 98 | 96 | 98 | 97 |
| blind last 0.5 m | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 |
| blind + pitch 2° | **84** | 96 | 96 | 100 | 100 | 100 | 100 | 95 | 99 | 100 | 100 |
| blind + height 0.05–0.15 | 96 | 100 | 100 | 100 | 100 | 100 | 100 | 99 | 100 | 100 | 100 |
| height 0.05–0.15 (code 0.10) | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 |
| rolling 0.3–1 m/s | 89 | 92 | 99 | 99 | **67** | 100 | 100 | 100 | 97 | 100 | 100 |
| two pieces 0.4–0.8 m apart | 100 | 100 | 100 | 100 | 95 | 99 | 99 | 100 | 100 | 99 | 100 |
| shove in last 0.6 m | 100 | 100 | 100 | 100 | 93 | 100 | 100 | 100 | 100 | 100 | 100 |
| blind + shove | 100 | 100 | 100 | 100 | 93 | 100 | 100 | 100 | 100 | 100 | 100 |
| ALL (static piece) | 95 | 99 | 99 | 99 | 87 | 99 | 99 | 98 | 96 | 100 | 99 |
| ALL + rolling | **49** | 59 | 56 | 53 | **49** | 81 | 85 | 87 | 67 | 85 | 84 |

### Success % if the intake were only W = 0.2 m (±10 cm)

| Stress | A | A' | A+ | A\* | B0 | B | B' | C | D | H | R |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| base | 100 | 100 | 100 | 100 | 93 | 100 | 100 | 100 | 100 | 100 | 100 |
| pitch 2° | 96 | 96 | 96 | 100 | 93 | 100 | 100 | 96 | 100 | 100 | 100 |
| noise 5 px + 30 % dropouts | 100 | 100 | 100 | 100 | 91 | 100 | 100 | 100 | 100 | 100 | 100 |
| late: 0.9–1.8 m, 2–3 m/s | 91 | 87 | 88 | 89 | 25 | 88 | 91 | 91 | 85 | 88 | 89 |
| blind + pitch 2° | 84 | 96 | 96 | 100 | 85 | 100 | 100 | 95 | 99 | 100 | 100 |
| rolling 0.3–1 m/s | 75 | 76 | 76 | 79 | 37 | 91 | 98 | 98 | 69 | 91 | 89 |
| two pieces 0.4–0.8 m apart | 100 | 100 | 100 | 100 | 73 | 99 | 99 | 100 | 100 | 99 | 100 |
| shove in last 0.6 m | 30 | 44 | 47 | 39 | 66 | 47 | 48 | 32 | 51 | 47 | 39 |
| blind + shove | 42 | 57 | 58 | 54 | 72 | 80 | 80 | 44 | 61 | 80 | 53 |
| ALL (static piece) | 94 | 95 | 99 | 98 | 68 | 97 | 96 | 95 | 93 | 97 | 97 |
| ALL + rolling | 29 | 37 | 33 | 33 | 26 | 52 | 60 | 61 | 44 | 58 | 61 |

Rows that are 100 % for every strategy except B0 are left out. All rows are in the `run_study.py`
output.

### Median time to intake (s)

| Stress | A | A\* | B0 | B | C | D | H | R |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| base | 1.34 | 1.27 | 1.08 | 1.05 | 1.34 | 1.17 | 1.05 | 1.26 |
| late | 0.80 | 0.73 | 0.44 | 0.62 | 0.79 | 0.66 | 0.62 | 0.72 |
| blind + pitch 2° | 1.31 | 1.25 | 1.08 | 1.00 | 1.23 | 1.18 | 1.00 | 1.17 |
| rolling | 1.45 | 1.41 | 1.19 | 1.13 | 1.37 | 1.21 | 1.13 | 1.29 |
| ALL (static piece) | 1.12 | 1.09 | 1.01 | 0.87 | 1.14 | 1.08 | 0.87 | 1.10 |

B and H hit the piece at about 1.2–1.7 m/s. The `DriveToPose` versions hit it at 0.45 m/s (A),
0.65 m/s (A\*, R) or about 1.1 m/s (D). For rolling pieces, the 90th-percentile time is 3.2 s for A
(it parks at a stale estimate) and 1.6 s for B, H and R.

### Where the failures come from (three key stresses)

| Stress | Strategy | Success | Bump | Timeout | Too fast | Never locked | Miss p95 (cm) |
|---|---|---:|---:|---:|---:|---:|---:|
| blind + pitch 2° | A | 84 | 0 | **16** | 0 | 4 | 1.5 |
| | A\* | 100 | 0 | 0 | 0 | 0 | 1.6 |
| | C | 95 | 0 | 5 | 0 | 4 | 1.4 |
| | H | 100 | 0 | 0 | 0 | 0 | 1.6 |
| rolling | A | 89 | 0 | **11** | 0 | 0 | 14.8 |
| | C | 100 | 0 | 0 | 0 | 0 | 8.9 |
| | H | 100 | 0 | 0 | 0 | 0 | 10.9 |
| | R | 100 | 0 | 0 | 0 | 0 | 11.6 |
| ALL + rolling | A | 49 | 9 | **41** | 0 | 1 | 32.1 |
| | B0 | 49 | **35** | 5 | 12 | 0 | 37.4 |
| | C | 87 | 8 | 4 | 1 | 1 | 24.4 |
| | H | 85 | 3 | 0 | **11** | 0 | 19.2 |
| | R | 84 | 13 | 1 | 2 | 1 | 34.3 |

H's "too fast" failures are pieces rolling *toward* the robot while it drives in at 1.2–1.7 m/s, so
the closing speed exceeds 2.5 m/s. If the intake can't take that, lower the near speed to 0.8–1.0 m/s.

### Estimated piece position vs truth at contact (p95, robot frame)

| Stress | Range error, A / C / R (cm) | Sideways error, A / C / R (cm) |
|---|---|---|
| base | 3.9 / 4.2 / 3.4 | 1.0 / 1.0 / 1.0 |
| pitch 2° | 6.4 / 5.0 / 4.5 | 1.0 / 0.8 / 0.9 |
| blind + pitch 2° | 14.6 / 12.4 / 12.8 | 1.5 / 1.5 / 1.5 |
| blind + height 0.05–0.15 | 13.0 / 11.5 / 11.6 | 1.4 / 1.3 / 1.3 |
| rolling | 10.8 / 5.6 / 5.1 | 7.2 / 1.7 / 1.7 |

This is the floor-projection physics: range error ≈ (H² + d²)/H per radian of pitch error. With
H = 0.4 m, that is 1.8 cm/° at 0.5 m, 5.1 cm/° at 1 m, 18 cm/° at 2 m and 40 cm/° at 3 m. The bearing
error stays near d × (angle error), which is millimeters close in.

### Ablations (the R design; success % at W 0.4 / W 0.2, median time)

| Stress | reach 0.45 | **0.40** | 0.35 | 0.30 | KF q = 1.0 | static KF | + box-edge reject | C (4 m cut, reach 0.45) | decel 4.5 m/s² |
|---|---|---|---|---|---|---|---|---|---|
| base | 100/100, 1.34 s | 100/100, 1.26 s | 100/100, 1.21 s | 100/100, 1.17 s | 100/100 | 100/100 | 100/100 | 100/100 | 100/100, 1.17 s |
| pitch 2° | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 | **96**/96 | 95/95 |
| blind + pitch 2° | 99/99 | 100/100 | 100/100 | 99/99 | 100/100 | 100/100 | 97/97 | 95/95 | 93/93 |
| rolling | 100/**98** | 100/89 | 100/83 | 100/75 | 99/91 | 100/90 | 99/90 | 100/98 | 100/87 |
| ALL (static) | 98/95 | 99/97 | 99/98 | 99/97 | 99/98 | 99/99 | 95/93 | 98/95 | 99/95 |
| ALL + rolling | **87**/61 | 84/61 | 82/60 | 79/59 | 74/51 | **60**/39 | 76/49 | 87/61 | 71/57 |

### Camera mount (success % at W 0.4 / W 0.2)

| Mount | A | A\* | B | H | C | R |
|---|---|---|---|---|---|---|
| 0.5 m high, 25° down (sees into the intake), static | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 |
| 0.6 m high, 5° down (blind 0.45 m), static | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 |
| 0.5 m, 25° down, **rolling** | 89/75 | 99/79 | 100/91 | 100/91 | 100/98 | 100/89 |
| 0.6 m, 5° down, **rolling** | **47/28** | 55/38 | 90/63 | 92/74 | 95/81 | 99/79 |
| 0.6 m, 5° down, static, 15 fps instead of 30 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 | 100/100 |

Halving the frame rate costs no success. It adds about 0.13 s for the `DriveToPose` versions and
nothing for B and H.

---

## 4. Findings

1. **Sideways accuracy is easy; range is hard.**
   - Every strategy that keeps re-aiming at the piece misses by about 1–2 cm sideways (p95) with a
     static piece. That holds even with 2° of pitch error, 5 px of noise, 30 % dropouts or 70 ms of
     latency.
   - The ±20 cm intake has a lot of margin. Even ±10 cm works for static pieces.
   - The range estimate is 3–15 cm off at contact: about 10× the sideways error.
2. **The current Java fails in two ways, and both come from trusting range.**
   - **(a) The 4 m range cut.** A 2–2.5° pitch error pushes a piece that is really 3.1 m away out to
     4.1 m, so it is filtered out. The robot never locks and holds still (4 % of trials at 2°).
   - **(b) Stopping short.** The goal is the piece at the bumper face, only 7 cm past first contact.
     With a camera that goes blind 0.5 m out and a 2° pitch error, the last estimate is about 8 cm
     too close, and the robot parks 0–3 cm short of touching the piece (16 % timeouts).
   - A\* fixes both with two constants: an 8 m cut, and reach 0.40 (12 cm past contact).
   - Bearing pursuit has neither problem, because it never uses range to stop.
3. **Pitch calibration, piece height, detector noise, dropouts, latency and clock bias barely
   matter for pickup**, as long as (2) is fixed.
   - Placing the floor point with the odometry pose at capture time (A vs A') changed nothing
     measurable. During a pickup the robot drives roughly *at* the piece, so latency only shifts
     the range.
   - A' even looks slightly better in the blind cases. That is an accident: no compensation pushes
     the estimate about v × latency farther away, which acts as extra reach margin.
   - Keep capture-time placement anyway; it costs nothing and matters when the robot turns.
4. **Rolling pieces need either continuous re-aiming or a velocity estimate.**
   - The 0.3 EMA lags a rolling piece. When the piece leaves the image, the robot parks on a stale
     point: A gets 89 % with 11 % timeouts.
   - The constant-velocity Kalman filter (C, R) and bearing pursuit (B, H) all score 100 % with the
     default camera.
   - With a narrow intake, C's velocity feedforward with a stop on the piece is the most accurate
     (98 % at W 0.2).
   - Kalman tuning matters. With q = 1.0 instead of 0.3, ALL + rolling falls from 84 % to 74 %. A
     filter without velocity falls to 60 %.
5. **"Commit near the intake" needs no special logic.**
   - An odom-frame tracker already commits: it keeps the last estimate when the piece disappears.
   - Explicit commit tricks *hurt*. Rejecting box-edge detections and freezing the heading (D)
     gives 67 % on ALL + rolling vs 84 %, and 69 % vs 89 % on rolling at W 0.2. They throw away
     the closest, most accurate looks.
   - The box-clipping bias near the image edge is mostly a range bias, which the intake depth absorbs.
6. **Seeing the piece into the intake is worth more than any algorithm change for rolling pieces.**
   With the default camera the good strategies score 100 %; with the blind mount they score
   90–99 % and A scores 47 %.
7. **Late, fast starts are limited by physics, not sensing.** All good strategies score 96–99 %.
   The failures are bumps where the robot can't shed 2–3 m/s of sideways speed in under a meter.
8. **Odometry slip in the last 0.5 m can't be corrected by anyone.** After a 5–15 cm slide, every
   strategy misses by up to about 15 cm (p95), so a ±10 cm intake would fail 20–70 % of the time.
   Faster final approaches (B, H) give the shove less time to act. Intake width is the defense.
9. **Two pieces close together are not a problem.** Nearest-to-estimate association (the Java 0.4 m
   gate), the Mahalanobis gate, and nearest-bearing all get 99–100 %.
10. **The 2026 strafe-only approach is the worst.** It averages 90 %, with 41 % on late starts and
    67 % on rolling pieces, and misses 11 cm sideways (p95) even in base. Holding heading and
    strafing on raw `tx` doesn't converge fast enough from a real approach. Don't reuse it.
11. **Bearing pursuit is about 0.2 s faster than the best `DriveToPose` version, at equal
    robustness.** Making the `DriveToPose` version faster with a harder braking curve (4.5 m/s²)
    gains 0.09 s but drops ALL + rolling from 84 % to 71 %. The speed comes from never braking
    for a measured goal.

---

## 5. Recommendation and parameters

### Pickup law (H): steer on the bearing, remember in odom

Each loop (50 Hz), for each new detector frame:

1. **Bearing.** `bearing = odometryYaw(captureTime) − tx`.
   - `captureTime = now − (frame age + capture + targeting latency)`.
   - With two or more detections, keep the one whose bearing is closest to last loop's. For the
     first pick, use the lowest `ty` (the nearest piece).
2. **Speed** (thresholds are computed once from the mount; no range is used in the loop):
   - `v = 1.2 + 1.8 · clamp((ty − tyNear) / (tyFar − tyNear), 0, 1)` m/s.
   - `tyFar = pitchDown − atan(H / 1.6 m)` and `tyNear = pitchDown − atan(H / 0.6 m)`, with
     H = camera height − piece height (1.6 m and 0.6 m are floor distances from the camera).
   - For the default mount (25° down, H = 0.4 m): tyFar = +11.0°, tyNear = −8.7°.
3. **Command.**
   - Field velocity: `v · (cos bearing, sin bearing)`.
   - ω from the `DriveToPose` heading law on `bearing − yaw` (6 rad/s max, 8 rad/s² braking, kP 5).
   - Then `DriveToPose`'s vector acceleration limit of 6 m/s² and ω slew of 20 rad/s², seeded with
     the measured velocity.
4. **Lost (no detection for 0.12 s).** Drive at 1.2 m/s toward the `PieceTracker` odom-frame
   estimate for up to 1.0 s, then stop.
   - Without a tracker: if the piece was last seen low (`ty` < bottom of image + 12°), hold the last
     bearing at 1.2 m/s for 0.8 s.
5. **End** on the intake sensor, with a timeout of about 2.5 s.

### Piece tracker (memory, autos, rolling pieces)

- **Placement.** Floor projection placed with the odometry pose at capture, using **every** frame.
- **Filter.** A constant-velocity Kalman filter with state (x, y, vx, vy) in the odom frame.
  - Process noise q = 0.3 m²/s³.
  - Initial velocity σ = 0.8 m/s.
- **Measurement noise.** Shaped along the line of sight, then rotated to the odom frame:
  - sideways σ = d·√((px/fx)² + 0.25°²);
  - range σ = (H² + d²)/H · √((px/fy)² + 0.7°²) + d·0.015/H.
- **Association.**
  - Nearest detection by Mahalanobis distance, gate χ² < 11.8.
  - Re-start on the nearest detection after 4 rejects in a row.
  - Lock after 3 frames.
  - Frame age ≤ 0.25 s; range cut 8 m.
- **Velocity.** Report it only when |v| > max(0.15 m/s, 1.5 σv); otherwise report zero.

### If you keep the `DriveToPose`-based `DriveToPiece` (R)

- Same tracker.
- **Reach 0.40 m.** The general rule: reach = bumper + piece radius − 0.12 m.
- Velocity feedforward, plus a lead of τ = (d − reach) / 2 m/s, capped at 1 s, when the piece moves.
- Expect 98.9 % mean and 84 % worst case, at 1.26 s.
- For a narrow intake and rolling pieces, reach 0.45 is more accurate (98 % vs 89 % at W 0.2), at
  the cost of 0.08 s.

### Hardware

- **Mount the detector over the intake center line, about 0.5 m high and about 25° down**, so the
  piece stays in view until it touches the intake.
- 15–30 fps is enough.
- Measure `PIECE_HEIGHT` as the height of the **box center** the detector reports, using the
  near/far test in §2.

---

## 6. What `PieceTracker.java` and `DriveToPiece.java` should change

In priority order.

1. **`MAX_DISTANCE_METERS` 4.0 → 8.0.** Floor range from a slightly wrong pitch can be meters too
   long, and a hard cut silently never locks.
2. **`INTAKE_REACH_METERS` 0.45 → bumper + piece radius − 0.12** (0.40 for a 0.07 m piece). The
   goal must sit well past first contact, or the robot parks short.
3. **Camera mount from code, not `getCameraPose_RobotSpace()`.** Use a WPILib `Transform3d` constant
   (NWU, pitch + = down), push it to the camera, and run the §2 checks. Remove the "TODO: confirm both
   signs" only after the left/right test passes.
4. **Before lock, don't drive back to `holdPose`.** A robot entering at 3 m/s would brake and
   reverse to where the button was pressed, and with no lock it sits there forever. Keep the current
   velocity, or hand control back, and end if there is no lock within about 0.5 s.
5. **`DriveToPiece`: add a bearing-pursuit final approach (H).** Drive along `yaw(capture) − tx` at
   the `ty`-scheduled speed and use the tracker only when the piece is out of view. Also expose the
   bearing from `PieceTracker` (`getBearing()`: yaw at capture − tx).
6. **`PieceTracker`: replace the 0.3 EMA with the constant-velocity Kalman filter above** if 2027
   pieces roll or get bumped. For pieces that stay put, items 1–2 are enough (A\* = 99.8 % on static
   stresses). Expose `getVelocity()`.
7. **Use every queued frame.** `getLatestResults()` drops the occasional frame when two arrive in
   one loop. This is a small effect at 30 fps and wasn't measured separately.
8. **Add `.withTimeout(~2.5 s)` at the call site**, next to `.until(intake::hasPiece)`.

Keep as is:

- the floor projection math (the signs are right);
- odometry placement at capture time;
- the facing-the-piece goal geometry;
- the nearest-to-estimate association, which handled two pieces 0.4–0.8 m apart perfectly.

---

## 7. Core simulator notes

**No bugs found in `alignsim/*.py`.** The coordinator's `twist_log` fix landed before any of these
runs, and no run hit a division error.

Modeling limits worked around in `piece_world.py` (not bugs):

- `Camera._detect_pieces` reports the projected piece *center* until that point leaves the image.
  It has no box size, so no edge clipping, and no motion blur. `PieceCamera` adds box clipping and
  dropouts.
- `Frame.pieces` carries only (tx, ty). A box area or corners would let a strategy detect edge
  clipping the way `DetectorTarget.corners` and `targetAreaPercent` would on the robot.

## 8. Limits of this study

- **No false positives.** The detector never reports a piece where none exists, and gives no
  confidence or class. Real detectors flicker; the Mahalanobis gate (tracker) and the
  nearest-bearing rule (H) are the defenses, but they are untested against clutter.
- **Simple piece model.** The piece is a point with a radius: it doesn't bounce off the bumper or
  get pushed, and the intake is a perfect window (center within ±W/2, ≤ 2.5 m/s relative). Real
  intakes have funnels, and a minimum speed to pull the piece in.
- **No defender, walls or field elements.** The only disturbance is the scripted shove.
- **Starting assumptions.** The piece always starts in view (bearing ±20°). Searching for a piece
  is not studied.
- **Strafe-only stand-in.** B0 models the 2026 driver as constant forward speed with held heading.
  A real driver also steers, so B0 is a pessimistic stand-in for the 2026 code.
- **Assumed numbers.** Camera latency, detector noise, piece size and friction are guesses. Replace
  them from robot logs.

## 9. Reproduce

```bash
cd tools/alignsim/studies/pieces
python3 check_signs.py --closed-loop   # sign checks + what a flipped tx looks like
python3 run_study.py 150               # ~25 CPU-min, one process; results/*.json + all tables
```

Files:

- `piece_world.py`: `PieceCamera`, `PieceScenario`, `PieceWorld`;
- `strategies.py`: A–D, B0, H, R;
- `configs.py`: the strategy and stress tables;
- `results/`: summaries per cell (JSON).
