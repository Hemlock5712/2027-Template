# Drive-to-point and precise alignment for 2027

**Status: research + simulation studies + prototype code, Oct 2026. Untested on a robot.**

This document covers how the 2027 robot should (1) drive to a point and (2) line up precisely with an
AprilTag or a game piece. It includes:
- what top FRC teams and industry do;
- what SystemCore and the 2027 libraries change;
- what a Python simulator says works best;
- the decisions we made, and how to test them on a real robot.

| Where | What |
| --- | --- |
| [research/](research/) | Raw notes from each research thread (teams, libraries, industry, SystemCore) |
| [video/](video/brief.md) | A narrated explainer brief for the animate video tool, with its voice-over script |
| [studies/](studies/) | The four simulation studies: [tag tracking](studies/tag-tracking.md), [drive-to-point controller](studies/controllers.md), [game pieces](studies/game-pieces.md), [skids and collisions](studies/skid-odometry.md) |
| [tools/alignsim](../../tools/alignsim/README.md) | The simulator, and `replay.py` for re-running trackers on robot logs |

---

## Decisions at a glance

| # | Decision | Confidence | Revisit when |
| --- | --- | --- | --- |
| 1 | **Two poses, one controller.** The *global* (vision-fused field) pose gets close and follows paths. A *local* pose (the target measured from the robot) does the final approach. Both feed the same `DriveToPose`; only the goal changes. | High | — |
| 2 | **The local pose lives in the wheel-only odometry frame.** Place each frame with the odometry pose *at capture time*, then filter. Map-based approaches fall to 15–57% when the element is off the map; with decision 3 this stays at 98–100%. | High (sim) | Logs show odometry drift > 1 cm per approach |
| 3 | **Calibrate each camera once** with the "Calibrate Cameras" OpMode (park on a real scoring spot, back away 0.6 m). Take `tagToGoal` from CAD. In the hard sim world this takes alignment from 66% to 100%; a taught goal gets 64%. | Medium-high (sim) | Logs show a camera's correction changes with distance or angle |
| 4 | **Use the tag's measured yaw**, not the map's: the map's yaw inherits the element's rotation error (31% in the hard world). Camera yaw bias is handled by decision 3. | Medium-high (sim) | Test plan step 5 |
| 5 | **Keep the simple per-axis Kalman filter** in `TagTracker`, dropping frames with ambiguity > 0.3. Once cameras are calibrated it ties a geometry-aware filter (100/99/98% vs 100/99/97%). | Medium-high (sim) | Replay on real logs prefers the geo filter |
| 6 | **Two cameras at the front corners.** One center camera sees the tag head-on, where the yaw solve is ambiguous: 73–78%. | High (sim) | — |
| 7 | **Drive-to-point law: keep `VectorBrake`, retuned** (`DriveToPose`): speed = `min(vmax, √(v_end² + 2·decel·d'), kP·d)` with a vector accel limit, plus a heading reaction time and stop deadbands. **Acceleration limited from physics each loop** (as in 2026): a friction circle of 10 m/s² shared by driving and turning, plus the motors' torque-speed limit at a 150 A model current; **DECEL 6 m/s²** so the last frames of a tag approach are taken slowly. Pass-through waypoints use the **corner rule** (`passingThrough(next)`). | High (sim) | Real drivetrain limits differ |
| 8 | **Rules for any limits you pick:** `ACCEL > DECEL`; **keep the drive slip current high** (360 A, a hardware ceiling) and limit acceleration in the command, because weight transfer loads some wheels far more than others; brake early by the actuation delay. | High (sim + 2026 experience) | — |
| 9 | **Timestamps:** capture time = now − (frame age + Limelight latency), on the Phoenix clock. A straight approach is forgiving (no compensation at all still gets 98%); turning while viewing is not. | High | Spin test shows the tag pose swinging |
| 10 | **Global fusion: add frames oldest-first.** The estimator throws away stored updates newer than the one being added. | High (read in source) | — |
| 11 | **Collisions are the open risk** (32% false "done" after a 300–800 N push near the target). Next: an IMU-vs-wheels skid detector, and treat the lock as suspect after a skid (back off 30 cm, re-lock). Not in code yet. | Medium (sim) | IMU logs from pushes |
| 12 | **Game pieces: steer on the bearing** (`yaw at capture − tx`), speed from `ty`, never stop on a measured range. The two-constant fixes are in; the full law is next. | Medium-high (sim) | 2027 game piece |
| 13 | **2027 hardware:** SystemCore **plus a CANivore** (250 Hz odometry), Pigeon 2, Limelight 4s. 100 Hz odometry is fine for alignment; set `REACTION_TIME` to 0.10 without a CANivore. Next physics/timing step: run `DriveToPose` at 250 Hz in CTRE's odometry thread and send wheel force feedforward split by each wheel's load (controllers study Q6). Don't use Motion Magic on swerve drive motors. | Medium (beta) | SystemCore CAN FD ships |
| 14 | **Paths:** Commands v3 coroutines with `DriveToPose(...).passingThrough(next)` waypoints and `fork` for mechanisms. For curved paths use BLine (supports v3) or Choreo behind a small v3 follower. | Medium | 2027 game reveal |
| 15 | **Upgrade the template next** to WPILib alpha-7, Phoenix 26.70.0-alpha-2, LimelightLib beta9-alpha7, AdvantageKit 27 alpha-5/6 and SystemCore image ≥ 14. Do it as its own PR. | High | — |

---

## 1. The problem

The field map says where every AprilTag is. Real fields don't match the map:

- **The manual's tolerances are loose.** Arena drawings carry ±½ in and 1°.
- **FIRST expects worse.** 2025 Team Update 12 warned that event variation "will exceed" the
  difference between the welded and AndyMark tag maps, and those differ by 0.2–2.9 in.
- **Events have been worse still.** Trusses rotated up to 8° and bowed tag sheets have been
  reported.

Teams that scored precisely in 2025 kept per-event offset tables (Spectrum) or aligned relative to
the target itself (6328, 604, 2910).

So a robot that knows its field pose perfectly still ends up a few cm off, because it is driving to
where the element *should* be. The fix is to measure the target directly in the final stretch:

- **Global pose:** the robot on the field, from odometry plus every tag. It is good for paths,
  shooting and getting close. It jumps a little whenever vision corrects it.
- **Local pose:** the target relative to the robot, measured from that target's own tag (or the
  game piece). It is good for the last 1–3 m. It must stay correct while the robot moves, which
  means latency compensation.

## 2. What FRC teams do

### Your 2026 code (`2026-AutoTesting`)

- **`DriveToPoint`** commands velocity directly. Each axis has a braking speed of
  √(v_end² + 2·a·d) with a reaction buffer. Grip is shared between rotation and translation, and a
  motor-torque and friction acceleration limiter follows. The goal is a `Supplier<Pose2d>`. This is
  a strong base, and `DriveToPose` keeps its core idea.
- **In the controller study** it was the fastest on the final approach and had the least slip, but:
  - per-axis limits bend diagonal paths (57 cm p95 off the straight line);
  - it overshoots about 2 cm and is more latency-sensitive (12 cm at +40 ms);
  - with no gentle zone near the goal it chatters under pose noise;
  - its 2 cm / 2° deadbands gave 38% false "done" on the vision approach.
- Vision fusion is global only, so there is no local pose, and there is no skid or collision
  handling.

### How top teams split up

The strategies they use, with the teams in each:

| Approach | Teams | Takeaway |
| --- | --- | --- |
| Global pose → map goal | 1678, 971, 4481 | Simple, but inherits field error |
| Global + per-event offsets or field mapping | Spectrum (offset tables), 971 (mapping tool), WPIcal | Works, but costs calibration time and discipline |
| Fuse **only the target's tag** while aligning | 581 (`CLOSEST_REEF_TAG`), 254, 604, 1678 | In the sim, no better than all tags: the error is in the element, not in which tags are fused |
| **Local pose: target tag + odometry since capture** | **6328 2025** ("txTy" poses, blended in at 24–36 in), **604 2025** (reset a separate odometry at capture time and replay), 2056 (chassis-relative offset) | Map error cancels. This is what we chose |
| Reset odometry to the target tag | 2910 | Map error cancels, but they had no latency compensation (their own TODO) |
| Physical registration | 3476 (beam breaks), 1678 (drive into the tower) | Best last-cm precision, but needs mechanism design |

**Other details worth copying:**

- **6328:**
  - A rolling `DriveToPose` that re-seeds its profile from the last setpoint every loop, with a
    `Supplier<Pose2d>` goal. (In the sim, profile-based laws step the command after a moving start
    and slip the wheels; see the controller study.)
  - Approach shaping by sliding the target back along the element's normal, so the robot finishes
    straight in. (In the sim: 100% vs 99% for driving straight in, +0.10 s.)
  - Scoring only when both the current pose *and* a pose predicted 0.3 s ahead are in tolerance.
- **254:**
  - Pathfind, then a straight "join path" along the normal.
  - Hold nearly still at the join point until the mechanism, heading and fresh vision are all ready.
  - Control the *claw* point, not the robot center.
- **604:** finish tolerances measured *in the target's frame*: 1.5 in depth, 1.0 in sideways, 1°,
  held for 50 ms.
- **581:**
  - "Trailblazer": `v = min(P·d, vmax, vPrev + a·dt, √(2·a·d))`, pointed at the target.
  - In offseason 2026 they added a rotation-aware cap so the robot doesn't arrive before it has
    finished turning.
  - A progress parameter that only moves forward, so vision jumps can't drag the target backward.
- **Spectrum's October 2026 SystemCore repo** (alpha-6, Phoenix 26.50, like ours):
  - Copies 250 Hz odometry samples off CTRE's thread.
  - Logs raw frames before rejection, with a named rejection chain per camera.
  - Rejects frames taken within 0.3 s of a fast spin.
- **Orbit (1690)** publishes no code, so their formulas could not be checked. The public part is
  their **skid ratio**: `max|tᵢ| / min|tᵢ|` with `tᵢ = vᵢ − ω×rᵢ`, about 1.0 on a rigid robot and
  1.1–1.2 when driving and turning together. In the skid study it caught only ~30% of harmful skids:
  pushes, launches and hard braking slide all four wheels together, and the ratio can't see that.

## 3. What industry does

The plan matches standard practice almost exactly:

- **ROS REP-105** keeps two frames. `map` is globally correct but *jumps*; `odom` is continuous but
  drifts. Controllers run in `odom`.
- **Nav2 docking, Fetch, Spot, ArduPilot and PX4** all track the target in the odom (or
  vehicle-relative) frame, transforming each detection *at its own timestamp*, then filtering.
  - Nav2's docs: the fixed frame is *"recommended to be a smooth odometry frame, not map."*
  - They coast through dropouts using odometry, and abort and retry when off-corridor (Fetch:
    inside 0.4 m, more than 2.5 cm or 5° off).
- **Hand-eye calibration:** industry calibrates each camera's mount against a known target instead
  of trusting the CAD mount. That is what "Calibrate Cameras" does, in 2D, with the robot parked on a
  real scoring spot.
- **Error budget:**
  - A 1° camera-yaw error shifts the robot 1.7 cm sideways at 1 m.
  - Single-tag yaw from the camera's pose solve is weak at range and ambiguous head-on.
  - Depth error grows with distance squared.
  - So: finish close, use two cameras off to the sides, and use a time-of-flight sensor (CANrange)
    for depth if needed.
- **The last centimetre is mechanical.** Spot docks with indexing cones, and AGVs use fork
  lead-ins. Design a funnel that forgives ±3–5 cm and ±3°, and confirm with a beam break.
- **Skid detection:** vehicles compare the IMU with wheel odometry. In the skid study this caught
  95% of harmful skids with no false alarms, where wheel-only checks caught ~30%.

## 4. What 2027 changes

**Versions (newest public, Oct 11 2026):**

| Piece | Template pins | Newest |
| --- | --- | --- |
| WPILib | alpha-6 | alpha-7 (`main` is newer; no beta yet) |
| Phoenix 6 | 26.50.0-alpha-1 | 26.70.0-alpha-2 |
| LimelightLib 2 | ~~beta2~~ → **beta9-alpha5-6** (this PR) | beta9-alpha7 |
| SystemCore OS | ≤ 13 | 16d; image 14+ **requires alpha-7** |
| PhotonLib | none for alpha-6 | dev builds for alpha-7 |

**What it means for us:**

- **SystemCore CAN FD isn't ready.** The native buses run CAN 2.0, and odometry falls back to
  **100 Hz**. A CANivore is legal for 2027 (but not 2028). The template already uses one, giving
  250 Hz. In the sim, 100 Hz doesn't hurt alignment; it adds about 1 cm of drift during fast
  steering.
- **Timebase:**
  - On SystemCore, Phoenix (≥ 26.50), WPILib and NetworkTables share one clock.
  - Limelight capture times are *receive time − latency*. The camera's clock is synced to the robot
    once per NT connection.
  - The code converts using `getAgeSeconds()`, so it never mixes clocks.
- **Commands v3 `fork`:**
  - Forked children start the same loop and are **cancelled when the parent ends**.
  - **Trap:** if the driver later triggers a command that conflicts with a forked child, the whole
    parent is cancelled.
  - In alpha-7, `fork`/`await` return a `ForkResult`.
- **WPILib alpha-7 changes:**
  - Removed `HolonomicDriveController`.
  - Rewrote `TrapezoidProfile` (renames).
  - Renamed constants (`kZero` → `ZERO`).
  - Made `Mechanism` an interface.
  - The new code avoids the first two so it ports easily.
- **PathPlanner and Choreo** 2027 builds are **Commands v2 only**. BLine (FRC 3476's GitHub) has v3
  support.
- **LimelightLib beta9** moved result types to the top level (`com.limelightvision.LimelightResults`
  and similar) and renamed `setPriorityTagID` → `setPriorityTagIDOverride`.

## 5. Simulation results

The simulator ([tools/alignsim](../../tools/alignsim/README.md)) models:
- swerve physics at 500 Hz with real wheel slip;
- CTRE's 250 Hz odometry thread and a faithful port of WPILib's `PoseEstimator`;
- Limelight-like cameras that project tags with the *true* mount and lens, then solve with the
  *believed* ones;
- a field element that is off the map.

A run **succeeds** when the robot declares itself done *and* is truly within 2 cm and 1°. A
**false done** (declared done, truly outside) is the dangerous case. **Every error size is a guess
until robot logs replace it.**

### Tag alignment ([tag-tracking.md](studies/tag-tracking.md))

Success % / false-done % as the world gets harder (n = 100 each):

| approach | default | hard | very hard |
| --- | --- | --- | --- |
| global pose, all tags | 57 / 43 | 31 / 69 | 15 / 85 |
| global pose, target tag only (581, 254) | 57 / 43 | 31 / 69 | 16 / 84 |
| raw tag pose every loop (the old `DriveToTag`) | 10 / 0 | 9 / 10 | 6 / 24 |
| fused-pose history (2026 `samplePoseAt` style) | 100 / 0 | 71 / 29 | 45 / 55 |
| odom-frame tracker, no calibration | 100 / 0 | 72 / 28 | 46 / 54 |
| **odom-frame tracker + per-camera calibration + retuned controller** | **100 / 0** | **99 / 1** | **98 / 2** |

What it says:
1. **Measure the target.** Anything that drives to the map's goal fails as soon as the element is off
   the map.
2. **Then calibrate the cameras.** With the field error gone, camera mount yaw (0.8° → 92–95%) and the
   pose solve's yaw bias (1° → 78–83%) are what's left. Each is the same every frame, so no filter
   averages it out. A per-camera correction fixes both.
3. **The filter barely matters** once cameras are calibrated, so the simple one ships.
4. **Barely matter:** frame rate, pixel noise, handoff speed, 100 Hz odometry, the done gate, and
   approach shaping (+0.1 s for 99 → 100%).

### Drive-to-point controller ([controllers.md](studies/controllers.md))

- `VectorBrake` (this repo's law) was the only one with no overshoot, an exact finish, straight paths,
  no command steps from a moving start, and graceful behavior with latency.
- The old constants slipped the wheels about 300 ms per long move (`ANGULAR_ACCEL` 20 added a 0.16 m/s
  per-loop step at each module). The retune cuts that to 26 ms and finishes the approach 0.28 s sooner.
- **Accelerating at 10 m/s² is worth it:** moves get 0.23–0.28 s faster and tag alignment stays at
  96–99%. Braking stays at 6, because braking at 9 costs tag accuracy in the worst-case world
  (97% → 85–89%). Keep the slip current high: with weight transfer, a low one is slower and doubles the
  odometry error.
- **Physics and timing (Q6):** weight transfer makes the light wheels slip first. Running the law at
  250 Hz and splitting a wheel force feedforward by each wheel's load cut slip 3–4× at 6–8 m/s² in the
  sim. Motion Magic on the drive motors makes it worse (wheels scrub when turning while driving).
- Profile-based laws (6328 rolling, WPILib per-axis, CTRE LinearPath) step the command after a moving
  start. The wheels slip and odometry is off 61–146 cm, which wrecks the local pose (79% false done).
- Pass-through waypoints with the corner rule are 11–26% faster than stop-and-go and stay inside the
  corner.

### Game pieces ([game-pieces.md](studies/game-pieces.md))

- **Steering on the bearing** (`yaw at capture − tx`), with speed from `ty`, gets 99.0% mean success
  over 18 stresses (85% worst) in 1.05 s. The current `DriveToPiece` gets 94.8% (49% worst) in 1.34 s.
- **Range is the error, not bearing:** 3–15 cm in range vs 1–1.5 cm sideways at contact. Never stop on
  a measured range; let the intake's depth absorb it.
- **Mount the detector so it sees the piece into the intake** (about 0.5 m high, 25° down). A camera
  that goes blind 0.5 m out drops rolling-piece success from 89% to 47%.
- **Check the tx sign on the robot:** flipped, success drops to 2%.

### Skids and collisions ([skid-odometry.md](studies/skid-odometry.md))

- **Clean odometry is ~50× better than needed** (0.02 cm drift over the last second). Wheel-radius error
  doesn't affect the local tracker. Slip early in the approach is harmless: the tracker re-anchors.
- **A hit near the target is the failure:** 300–800 N mid-approach gives 32% false done, with 40–70 cm
  errors. All four wheels slide together, so the encoders never see it.
- **Detect with the IMU:** Pigeon acceleration vs wheel-derived acceleration catches 95% with no false
  alarms. Wheel-only checks (Orbit's skid ratio, current) catch ~30%.
- **Respond by distrusting the lock:** after a flagged skid, wait for 3 fresh frames or back off 30 cm
  and re-lock. False done drops from 32% to 0–10%.

## 6. What's in the code

```
Limelight frame ──► capture time = now − (frame age + capture + pipeline latency)   [Phoenix clock]
                          │
WheelOdometry (250 Hz, wheels + gyro only, 1.5 s history) ──► robot pose AT capture time
                          │
     tag in robot = CameraCorrections[camera] ⊕ measured        (from "Calibrate Cameras")
     tag in odometry frame = robotAtCapture ⊕ tag in robot      (TagTracker)
                          │   per-axis Kalman filter, 25 cm jump gate, lock after 3 frames,
                          │   coasts on odometry when the tag leaves view
                          ▼
                 goal = tag ⊕ tagToGoal   (tagToGoal from CAD)
                          │   re-anchored: fieldPose ⊕ (odometryNow⁻¹ ⊕ goal) — vision jumps cancel
                          ▼
DriveToPose controller  ◄── before lock: field-map goal (global);  after lock: local goal
```

| File | What it is |
| --- | --- |
| [`commands/DriveToPose.java`](../../src/main/java/frc/robot/commands/DriveToPose.java) | The one controller, with the sim-tuned limits. Fixed blue-origin goal (flipped for red) or a moving goal; `passingThrough(next)` waypoints with the corner rule; `withTolerance`; closed-loop velocity requests |
| [`commands/DriveToTag.java`](../../src/main/java/frc/robot/commands/DriveToTag.java) | Map goal until locked, then the local goal; finishes only when locked |
| [`commands/DriveToPiece.java`](../../src/main/java/frc/robot/commands/DriveToPiece.java) | Same idea for a detected game piece. Stops in place until locked; gives up after 0.5 s without a lock |
| [`subsystems/WheelOdometry.java`](../../src/main/java/frc/robot/subsystems/WheelOdometry.java) | Wheel + gyro pose with timestamped history, fed from CTRE's odometry thread |
| [`subsystems/vision/TagTracker.java`](../../src/main/java/frc/robot/subsystems/vision/TagTracker.java) | The local tag pose. Logs the raw per-frame inputs for offline replay |
| [`subsystems/vision/CameraCorrections.java`](../../src/main/java/frc/robot/subsystems/vision/CameraCorrections.java) | One correction per camera; paste the calibration output here |
| [`subsystems/vision/PieceTracker.java`](../../src/main/java/frc/robot/subsystems/vision/PieceTracker.java) | Floor projection of detector angles into the odometry frame |
| [`subsystems/vision/Vision.java`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) | Global pose; adds frames oldest-first |
| [`opmodes/CalibrateCamerasOpMode.java`](../../src/main/java/frc/robot/opmodes/CalibrateCamerasOpMode.java) | Park on a scoring spot, enable, the robot backs away 0.6 m and prints each camera's correction |
| [`opmodes/DriveThenAlignOpMode.java`](../../src/main/java/frc/robot/opmodes/DriveThenAlignOpMode.java) | Example auto: `fork` the arm, pass through a waypoint, align, score |
| [`tools/alignsim/replay.py`](../../tools/alignsim/replay.py) | Re-runs `TagTracker` and its alternatives on a robot `.wpilog` |

**Verification so far:**

- **Not compiled with Gradle** (this environment blocks the WPILib and CTRE maven servers).
- **Type-checked** with the Eclipse compiler on Java 25 against the real WPILib alpha-6 source, the
  real LimelightLib beta9-alpha5-6 jar, and AdvantageKit's real signatures. A planted bug was
  caught, so the check is real.
- **CTRE calls** (`registerTelemetry`, `getKinematics`, `SwerveDriveState.Timestamp`/`RawHeading`/
  `ModulePositions`, `ApplyRobotVelocity`) were checked against Spectrum's working code on the same
  Phoenix version, not compiled.
- **The math** (controller, tracker, calibration) matches the simulator versions that produced the
  numbers above. The replay tool reproduces `TagTracker`'s filter exactly.
- **Run `./gradlew build` and the sim first.**

**Known limits of this prototype (deliberate, to keep it small):**

- `TagTracker` uses the newest frame per camera per loop (≤ 50 fps). Draining the queue would need
  `Vision` to share its frames.
- No skid/collision handling yet (decision 11) and no abort/retry gate.
- `DriveToPiece` still drives to a `DriveToPose` goal; the bearing-pursuit law is next (decision 12).
- `PieceTracker` assumes the Limelight's tx is positive to the right. Check it: a piece on the
  robot's left must log a positive y. Also set `PIECE_HEIGHT_METERS`.
- `DriveToPose` uses **closed-loop velocity**, which needs real drive gains in `TunerConstants`
  (from Tuner X SysId). Slow final moves need it.
- Every limit, noise number, camera correction and `tagToGoal` is a first guess (`TODO`).

## 7. Test plan on the robot

The goal is to replace the simulator's guesses with measurements. Record a `.wpilog` for every step;
the keys are listed in the `log-reading` skill.

1. **Build, deploy, sanity.**
   - `./gradlew build`, deploy, and check that "4 - Drive Then Align" and "Calibrate Cameras" appear.
   - Drive around. `Drivetrain/OdometryPose` and `Drivetrain/Pose` should agree until vision
     corrects the latter.
   - `Drivetrain/OdometryFrequencyHz` should be about 250.
2. **Odometry and grip.**
   - Push the robot 3 m along a tape measure and compare with `OdometryPose`.
   - Measure the grip: ramp drive current on carpet until a wheel spins. Keep `kSlipCurrent` at about
     360 A (a ceiling); the real limit belongs in `DriveToPose`'s `TRACTION_ACCEL`.
   - Measure the CG height (or balance the robot on a rod) for the weight-transfer math.
   - Check acceleration: from rest, command a hard 3 m move and compare `OdometryPose` with a tape
     measure. More than ~2 cm off means the wheels slipped; lower `TRACTION_ACCEL`.
   - Weigh the robot for `ROBOT_MASS` (it sets the motor torque limit).
3. **Camera noise and bias vs distance.** Park still, facing a tag, at 1, 2 and 3 m, head-on and at
   30°, for 10 s each. The spread of `TagTracker/<id>/TagInRobot` gives the real noise; the mean vs a
   tape measure shows whether the yaw bias changes with distance or angle.
4. **Timing.**
   - **Spin test:** rotate in place at about 1 rad/s with a tag in view. `TagTracker/<id>/TagPose`
     must stay still. If it swings back and forth with rotation, the timestamps are off. If it
     traces a circle, the camera's mounting pose (set in the Limelight UI) is off.
   - Repeat while driving straight at the tag.
5. **Calibrate and repeat.**
   - Put the robot on a real scoring spot with a jig and run "Calibrate Cameras" three times,
     re-parking each time. Paste the average into `CameraCorrections`. The spread is our floor.
   - `CalibrateCameras/<camera>/Sample` vs `Distance` should be flat. A slope means one correction
     can't fix that camera.
   - Then do 20 align runs (hold A) from different starts, some at speed.
   - **Measure the true final error with a tape or jig** and write it down per run. Logs can't see
     field truth.
   - Repeat on a second element of the same type, and with the element rotated 2° on purpose.
6. **Collisions.** Log the Pigeon's acceleration with the wheel speeds, then shove the robot (gently)
   during a few approaches near the target. This tunes the skid detector before it is built.
7. **Game pieces.** Place a piece off to the robot's left: `PieceTracker` must log a positive y. Then
   check the range at two known distances to set `PIECE_HEIGHT_METERS`.
8. **Send me** the `.wpilog`s, the tape-measured errors per run, and the camera settings. Then:

   ```bash
   cd tools
   python3 -m alignsim.replay robot.wpilog --tag 7 --corr 0:x,y,deg 1:x,y,deg
   ```

   compares `TagTracker` with its alternatives (no correction, keep ambiguous frames, near frames only,
   the geo filter) on the same frames. The numbers also refit the simulator's noise and latency.

## 8. Next steps

1. **The robot test plan above.**
2. **Template upgrade to alpha-7** (separate PR): `Mechanism` becomes an interface, `ZERO`
   constants, the AprilTag vendordep, Phoenix 26.70, LimelightLib beta9-alpha7, AdvantageKit
   alpha-5/6, and SystemCore image ≥ 14.
3. **Skid and collision handling** ([skid-odometry.md](studies/skid-odometry.md) recommendations 2–5):
   an IMU-vs-wheels detector on the odometry thread, then "lock suspect after a skid" in `TagTracker`
   and a back-off-and-re-lock in `DriveToTag`.
4. **Game pieces** ([game-pieces.md](studies/game-pieces.md) §6): bearing pursuit in `DriveToPiece`,
   a constant-velocity Kalman filter in `PieceTracker`, and the camera mount from code.
5. **Logging for full replay:** have `Vision` own each camera's queue and share every frame.
6. **Mechanics:** a CANrange for depth to the element's face, a funnel for ±3–5 cm, and a beam break
   to confirm.
7. **Event workflow:** nothing per event (the tag is measured live). Re-run "Calibrate Cameras" only
   when a camera is moved or bumped.

## Sources

The research notes in [research/](research/) contain full URLs and source tags:

- [6328 and 254](research/1-6328-254.md)
- [Orbit, 581, 2056](research/2-orbit-581-2056.md)
- [Other top teams](research/3-elite-survey.md)
- [Path and drive libraries](research/4-libraries.md)
- [Industry](research/5-industry.md)
- [SystemCore and the 2027 stack](research/6-systemcore-stack.md)

Chief Delphi, docs.wpilib.org, CTRE's docs and Orbit's talks were unreachable from the research
environment. Claims from those sources come from search snippets and are tagged `[doc]` or
`[unverified]`.
