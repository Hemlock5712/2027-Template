# Drive-to-point and precise alignment for 2027

**Status: research + first prototype, Oct 2026. Untested on a robot.**

This document covers how the 2027 robot should (1) drive to a point and (2) line up precisely with an
AprilTag or a game piece. It includes what top FRC teams and industry do, what SystemCore and the
2027 libraries change, the decisions we made, and the plan for testing them on a real robot.

Raw notes from each research thread are in [research/](research/). The strategy simulator is in
[tools/alignment-sim](../../tools/alignment-sim/README.md).

---

## Decisions at a glance

| # | Decision | Confidence | Revisit when |
| --- | --- | --- | --- |
| 1 | **Two poses, one controller.** Use the *global* (vision-fused field) pose to get close and to follow paths, and a *local* pose (the target measured from the robot) for the final approach. Both feed the same drive-to-point controller; only the goal changes. | High | — |
| 2 | **Local pose lives in the wheel-only odometry frame.** Place each camera frame using the odometry pose *at capture time*, then filter. Don't use the fused pose's history (`samplePoseAt`). | High | Logs show odometry drift > 1 cm per approach |
| 3 | **Drive-to-point commands velocity directly:** `min(vmax, √(v_end² + 2·a·d), kP·d)`, then a vector acceleration limit seeded from the measured velocity. This is your 2026 approach plus 581's version plus a gentle zone near the goal. | High | — |
| 4 | **Teach each scoring goal** with the robot parked on a real element (Teach Tag Goal OpMode). | Medium-high | Taught goals don't repeat to < 1 cm |
| 5 | **Tag yaw: trust the measured yaw for now**; keep "use the map's yaw" as the alternative to test. | **Medium — settle it with logs** | See test plan step 5 |
| 6 | **Timestamps:** capture time = now − (frame age + Limelight latency), on the Phoenix clock. Measure the camera's clock offset once. | High | LED/spin test shows > 5 ms bias |
| 7 | **Global fusion: add both cameras' frames oldest-first.** The estimator throws away stored updates that are newer than the one being added. | High (read in source) | — |
| 8 | **2027 hardware:** SystemCore **plus a CANivore** (SystemCore's own CAN FD isn't ready), keep the Pigeon 2, and use Limelight 4s. | Medium (beta) | SystemCore CAN FD ships |
| 9 | **Paths:** Commands v3 coroutines: `DriveToPose` waypoints with `withEndSpeed`, and `fork` for mechanisms. For curved paths use BLine (supports v3) or Choreo behind a small v3 follower. | Medium | 2027 game reveal |
| 10 | **Upgrade the template next** to WPILib alpha-7, Phoenix 26.70.0-alpha-2, LimelightLib beta9-alpha7, AdvantageKit 27 alpha-5/6 and SystemCore image ≥ 14. Do it as its own PR. | High | — |

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
  game piece). It is good for the last 1–2 m. It must stay correct while the robot moves, which
  means latency compensation.

## 2. What FRC teams do

### Your 2026 code (`2026-AutoTesting`)

- **`DriveToPoint`** commands velocity directly. Each axis has a braking speed of
  √(v_end² + 2·a·d) with a reaction buffer. Grip is shared between rotation and translation, and a
  motor-torque and friction acceleration limiter follows. The goal is a `Supplier<Pose2d>`. This is
  a strong base, and the new `DriveToPose` keeps its core idea.
- **Gaps:**
  - The √ law has no gentle zone near the goal, so it creeps or chatters. The simulator shows about
    1.6 cm/s of residual motion.
  - Limiting each axis separately bends the path.
  - Vision fusion is global only, so there is no local pose.
  - There is no skid or collision handling.

### How top teams split up

The strategies they use, with the teams in each:

| Approach | Teams | Takeaway |
| --- | --- | --- |
| Global pose → map goal | 1678, 971, 4481 | Simple, but inherits field error |
| Global + per-event offsets or field mapping | Spectrum (offset tables), 971 (mapping tool), WPIcal | Works, but costs calibration time and discipline |
| Fuse **only the target's tag** while aligning | 581 (`CLOSEST_REEF_TAG`), 254, 604, 1678 | Cheap, and gets most of the benefit. Still a fused pose, so still jumpy |
| **Local pose: target tag + odometry since capture** | **6328 2025** ("txTy" poses, blended in at 24–36 in), **604 2025** (reset a separate odometry at capture time and replay), 2056 (chassis-relative offset) | Map error cancels. This is what we chose |
| Reset odometry to the target tag | 2910 | Map error cancels, but they had no latency compensation (their own TODO) |
| Physical registration | 3476 (beam breaks), 1678 (drive into the tower) | Best last-cm precision, but needs mechanism design |

**Other details worth copying:**

- **6328:**
  - A rolling `DriveToPose` that re-seeds its profile from the last setpoint every loop, with a
    `Supplier<Pose2d>` goal.
  - Approach shaping by sliding the target back along the element's normal, so the robot finishes
    straight in.
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
  1.1–1.2 when driving and turning together. In 2026 they also corrected Pigeon drift from vision
  and turned off Pigeon2 No-Motion Calibration.

## 3. What industry does

The plan matches standard practice almost exactly:

- **ROS REP-105** keeps two frames. `map` is globally correct but *jumps*; `odom` is continuous but
  drifts. Controllers run in `odom`.
- **Nav2 docking, Fetch, Spot, ArduPilot and PX4** all track the target in the odom (or
  vehicle-relative) frame, transforming each detection *at its own timestamp*, then filtering.
  - Nav2's docs: the fixed frame is *"recommended to be a smooth odometry frame, not map."*
  - They coast through dropouts using odometry, and abort and retry when off-corridor (Fetch:
    inside 0.4 m, more than 2.5 cm or 5° off).
- **Teach by showing (visual servoing):** record what the camera sees at the true goal, then servo
  until the view matches. Constant errors cancel: focal length, camera mounting and where the tag
  sits on the element. This is the strongest single idea for the last centimetre, and it is why the
  Teach Tag Goal OpMode exists.
- **Error budget:**
  - A 1° camera-yaw error shifts the robot 1.7 cm sideways at 1 m.
  - A 20 ms timestamp error costs 2 cm at 1 m/s, and 6 cm while spinning at 2 rad/s with the tag
    1.5 m away.
  - Single-tag yaw from the camera's pose solve is weak at range (about 1° at 1 m, about 10° at
    3 m).
  - Depth error grows with distance squared.
  - So: finish close, slow down near the goal, and use a time-of-flight sensor (CANrange) for
    depth if needed.
- **The last centimetre is mechanical.** Spot docks with indexing cones, and AGVs use fork
  lead-ins. Design a funnel that forgives ±3–5 cm and ±3°, and confirm with a beam break.

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
  250 Hz.
- **Timebase:**
  - On SystemCore, Phoenix (≥ 26.50), WPILib and NetworkTables share one clock.
  - Limelight capture times are *receive time − latency*. The camera's clock is synced to the robot
    once per NT connection, so measure the bias once (LED test).
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
  - The new code avoids all of these so it ports easily.
- **PathPlanner and Choreo** 2027 builds are **Commands v2 only**. BLine (FRC 3476's GitHub) has v3
  support.
- **LimelightLib beta9** moved result types to the top level (`com.limelightvision.LimelightResults`
  and similar) and renamed `setPriorityTagID` → `setPriorityTagIDOverride`.

## 5. Simulation results

`python3 tools/alignment-sim/align_sim.py --scenario all` runs 150 trials per row. The setup:

- The robot starts about 2.8 m out, already moving at 1.5 m/s.
- The goal is 0.55 m in front of the tag.
- Camera latency is 30 ms.
- The element is off the map by 2.5 cm and 1° (1σ).

**Every error size is an assumption until the robot logs replace it.**

| Strategy | Baseline: median / p95 error, % within 2 cm & 1° | Element 5 cm / 2° off | Camera yaw bias 1.5° | End twitch (baseline) |
| --- | --- | --- | --- | --- |
| Global pose → map goal | 0.9 / 2.4 cm, 71% | 1.4 / 3.7 cm, 40% | 0.9 / 2.4 cm, 71% | 0.7 cm/s |
| Global, target tag only | 0.7 / 2.0 cm, 67% | 1.3 / 3.6 cm, 40% | 0.7 / 2.0 cm, 67% | 0.1 cm/s |
| Raw tag pose, no compensation (old `DriveToTag`) | 8.6 / 21.8 cm, 3% | — | — | 4.5 cm/s |
| Tag via fused-pose history (`samplePoseAt`) | 0.7 / 1.7 cm, 92% | 0.7 / 1.9 cm, 89% | 1.3 / 3.1 cm, 48% | 0.7 cm/s |
| **Tag via wheel odometry (`TagTracker`)** | **0.6 / 1.2 cm, 92%** | **0.6 / 1.2 cm, 89%** | 1.0 / 3.1 cm, 52% | **0.07 cm/s** |
| Wheel odometry + map's tag yaw | 0.7 / 1.9 cm, 73% | 1.3 / 4.2 cm, 39% | **0.7 / 1.9 cm, 73%** | 0.05 cm/s |
| **Wheel odometry + taught goal** | **0.14 / 0.36 cm, 100%** | **0.15 / 0.37 cm, 100%** | **0.14 / 0.36 cm, 100%** | 0.27 cm/s |

What the table says:

1. **Raw target-space pose with no compensation is unusable at real gains.** Single-frame yaw noise
   becomes position noise, and the stale frames make the loop oscillate.
2. **The local pose in the wheel-odometry frame wins or ties**, and finishes about 10× calmer than
   the fused-history version, because vision jumps never reach it. It is also the most robust to a
   25 ms timestamp error: 91% vs 81% within tolerance.
3. **The yaw question decides the rest.** If elements are rotated more than the camera's yaw solve
   is biased, the measured yaw wins; otherwise the map's yaw wins. **Teaching the goal makes it
   moot** by cancelling the camera's bias.

   The 100% for teaching is optimistic: real biases change a little with viewing angle. The
   direction of the result is solid.
4. **The gentle zone near the goal matters.** With the √ law alone, 84% end within tolerance (vs
   92%) and the robot still creeps at 1.6 cm/s.
5. Fusing **only the target's tag** into the global pose (581/254's trick) beats plain global
   alignment. It's a good fallback if the local pose misbehaves.

## 6. What's in the code

```
Limelight frame ──► capture time = now − (frame age + capture + pipeline latency)   [Phoenix clock]
                          │
WheelOdometry (250 Hz, wheels + gyro only, 1.5 s history) ──► robot pose AT capture time
                          │
                 tag in odometry frame = robotAtCapture ⊕ tagInRobot        (TagTracker)
                          │   per-axis Kalman filter, 25 cm jump gate, lock after 3 frames,
                          │   coasts on odometry when the tag leaves view
                          ▼
                 goal = tag ⊕ tagToGoal   (tagToGoal taught on a real element)
                          │   re-anchored: fieldPose ⊕ (odometryNow⁻¹ ⊕ goal) — vision jumps cancel
                          ▼
DriveToPose controller  ◄── before lock: field-map goal (global);  after lock: local goal
```

| File | What it is |
| --- | --- |
| [`commands/DriveToPose.java`](../../src/main/java/frc/robot/commands/DriveToPose.java) | The one controller. Fixed blue-origin goal (flipped for red) or a moving goal; `withEndSpeed` waypoints; `withTolerance`; closed-loop velocity requests |
| [`commands/DriveToTag.java`](../../src/main/java/frc/robot/commands/DriveToTag.java) | Map goal until locked, then the local goal; finishes only when locked |
| [`commands/DriveToPiece.java`](../../src/main/java/frc/robot/commands/DriveToPiece.java) | Same idea for a detected game piece |
| [`subsystems/WheelOdometry.java`](../../src/main/java/frc/robot/subsystems/WheelOdometry.java) | Wheel + gyro pose with timestamped history, fed from CTRE's odometry thread |
| [`subsystems/vision/TagTracker.java`](../../src/main/java/frc/robot/subsystems/vision/TagTracker.java) | The local tag pose. Logs raw per-frame inputs for offline replay |
| [`subsystems/vision/PieceTracker.java`](../../src/main/java/frc/robot/subsystems/vision/PieceTracker.java) | Floor projection of detector angles into the odometry frame |
| [`subsystems/vision/Vision.java`](../../src/main/java/frc/robot/subsystems/vision/Vision.java) | Global pose; now adds frames oldest-first |
| [`opmodes/TeachTagGoalOpMode.java`](../../src/main/java/frc/robot/opmodes/TeachTagGoalOpMode.java) | Prints the `tagToGoal` to paste into code |
| [`opmodes/DriveThenAlignOpMode.java`](../../src/main/java/frc/robot/opmodes/DriveThenAlignOpMode.java) | Example auto: `fork` the arm, pass through a waypoint, align, score |

**Verification so far:**

- **Not compiled with Gradle** (this environment blocks the WPILib and CTRE maven servers).
- **Type-checked** with the Eclipse compiler on Java 25 against the real WPILib alpha-6 source, the
  real LimelightLib beta9-alpha5-6 jar, and AdvantageKit's real signatures. A planted bug was
  caught, so the check is real.
- **CTRE calls** (`registerTelemetry`, `getKinematics`, `SwerveDriveState.Timestamp`/`RawHeading`/
  `ModulePositions`) were checked against Spectrum's working code on the same Phoenix version, not
  compiled.
- **The controller law** was exercised in the Python simulator.
- **Run `./gradlew build` and the sim first.**

**Known limits of this prototype (deliberate, to keep it small):**

- `TagTracker` uses the newest frame per camera per loop (≤ 50 fps). Draining the queue would need
  `Vision` to share its frames.
- There is no abort/retry gate, no approach shaping (use a staging waypoint along the normal for
  now), no skid/collision handling, and no per-camera taught goals.
- `PieceTracker` assumes the Limelight's tx is positive to the right. Check it: a piece on the
  robot's left must log a positive y. Also set `PIECE_HEIGHT_METERS`.
- `DriveToPose` now uses **closed-loop velocity**, which needs real drive gains in `TunerConstants`
  (from Tuner X SysId). Slow final moves need it.
- Every limit, noise number and `tagToGoal` is a first guess (`TODO`).

## 7. Test plan on the robot

The goal is to replace the simulator's guesses with measurements, then settle decision 5. Record a
`.wpilog` for every step; the keys are listed in the `log-reading` skill.

1. **Build, deploy, sanity.**
   - `./gradlew build`, deploy, and check that "4 - Drive Then Align" and "Teach Tag Goal" appear.
   - Drive around. `Drivetrain/OdometryPose` and `Drivetrain/Pose` should agree until vision
     corrects the latter.
   - `Drivetrain/OdometryFrequencyHz` should be about 250.
2. **Odometry scale.** Push the robot 3 m along a tape measure and compare with `OdometryPose`.
3. **Camera noise vs distance.** Park the robot still, facing a tag, at 1, 2 and 3 m for 10 s each.
   The spread of `TagTracker/<id>/TagInRobot` gives the real depth, sideways and yaw noise.
4. **Timing.**
   - **Spin test:** rotate in place at about 1 rad/s with a tag in view. `TagTracker/<id>/TagPose`
     must stay still. If it swings back and forth with rotation, the timestamps are off. If it
     traces a circle, the camera's mounting pose (set in the Limelight UI) is off.
   - Repeat while driving straight at the tag.
   - Optional LED test to measure the camera clock bias directly.
5. **Teach and repeat.**
   - Park on a real element and run Teach Tag Goal three times, re-parking each time. The spread of
     the printed numbers is our floor.
   - Then do 20 align runs (hold A) from different starts, some at speed.
   - **Measure the true final error with a tape or jig** and write it down per run. Logs can't see
     field truth.
   - Repeat on a second element of the same type (or rotate the element 2° on purpose) to see
     whether the measured yaw or the map's yaw is closer to the truth.
6. **Send me** the `.wpilog`s, the tape-measured errors per run, and the camera settings
   (resolution, mounting pose). From those I can:
   - fit the simulator's noise and latency numbers;
   - replay `TagTracker` offline with different filters, yaw sources and timestamp offsets, using
     the logged `TagInRobot` / `RobotAtCapture`;
   - tune `DriveToPose`'s limits.

## 8. Next steps

1. **Template upgrade to alpha-7** (separate PR): `Mechanism` becomes an interface, `ZERO`
   constants, the AprilTag vendordep, Phoenix 26.70, LimelightLib beta9-alpha7, AdvantageKit
   alpha-5/6, and SystemCore image ≥ 14.
2. **Logging for full replay:** have `Vision` own each camera's queue and share frames, and log the
   raw frame bytes (`readRawFrameQueue`) so any strategy can be re-run from a log.
3. **Approach shaping and safety:** a staging point on the element's normal, an abort/retry gate
   inside about 0.4 m, and a ready gate that checks a predicted pose (6328).
4. **Skid and collision detection** (Orbit's ratio plus an IMU jerk check) that inflate odometry
   trust, and Pigeon drift correction from vision.
5. **Depth and mechanics:** a CANrange for depth to the element's face, a funnel for ±3–5 cm, and a
   beam break to confirm.
6. **Event workflow:** re-teach goals during calibration or practice time.

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
