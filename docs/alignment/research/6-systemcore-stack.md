> Raw research notes gathered by AI research agents on 2026-10-11 for [the alignment decision doc](../README.md). Claims are tagged [code] (read in source), [doc] (talk/post/docs, often via search snippets — Chief Delphi was not reachable), or [unverified]. Check anything important at the source before relying on it.

# 6 — What the 2027 control system and software stack means for drive-to-point and target-relative alignment

Research date: 2026-10-11. Tags: **[confirmed+source]** = I read it in source code, release notes or an official repo. **[doc]** = vendor documentation or a search-engine summary of it that I could not open directly. **[speculative]** = my inference.
Network note: docs.wpilib.org, api.ctr-electronics.com, maven.ctr-electronics.com, docs.limelightvision.io and chiefdelphi.com were unreachable. Facts from those sites come from search summaries. I read GitHub sources directly.

## 0. Version reality check (pinned vs newest)

| Piece | Template pins | Newest public | Source |
|---|---|---|---|
| WPILib | 2027.0.0-alpha-6 | **alpha-7** (Sep 1 2026); `main` is 126 commits past it, no beta yet | [confirmed] `git ls-remote` of allwpilib; [release notes](https://github.com/wpilibsuite/allwpilib/releases/tag/v2027.0.0-alpha-7) |
| SystemCore OS | image 10–13 era | Beta/Alpha **16d** (Oct 10 2026). Image 14+ **requires alpha-7** | [confirmed] [systemcore-os-public releases](https://github.com/LimelightVision/systemcore-os-public/releases), [SystemcoreTesting README](https://github.com/wpilibsuite/SystemcoreTesting) |
| Phoenix 6 | 26.50.0-alpha-1 | **26.70.0-alpha-2** (for alpha-7, firmware 26.70.x) | [confirmed] [CTR-Phoenix.md](https://github.com/wpilibsuite/SystemcoreTesting/blob/main/CTR-Phoenix.md) |
| LimelightLib 2 | 2.0.0-beta2 | **2.0.0-beta9-alpha5-6** / **2.0.0-beta9-alpha7** | [confirmed] [limelightlib-public](https://github.com/LimelightVision/limelightlib-public) maven dir and tags |
| AdvantageKit | 27.0.0-alpha-4 | 27.0.0-alpha-5/-6 for alpha-7 | [confirmed] SystemcoreTesting table |
| PhotonLib | none | **No release for alpha-5/6.** For alpha-7, a Dev build (PV `main` merged "WPILib Alpha 7" on 2026-09-17) | [confirmed] SystemcoreTesting table, PhotonVision git log |

Moving from alpha-6 to alpha-7 breaks these parts of the template [confirmed, alpha-7 notes plus CTRE example diffs]:
- All vendordeps must change.
- `Mechanism` is now an interface, so `DriveMechanism`, `Arm` and `Flywheel` stop compiling as `extends Mechanism` with `super(name)`.
- Constants use ALL_CAPS (`Rotation2d.kZero` becomes `ZERO`).
- The AprilTag library moved to a vendordep, which breaks `AllianceFlip`.
- `Robot.class` becomes `Robot::new`.
- Raw integer timestamps are now nanoseconds. LimelightLib alpha7 sets `NT_TICKS_PER_SECOND=1e9` for this.
- `fork`/`await` now return `ForkResult`.
- Phoenix renames: `setOperatorPerspectiveForward` becomes `setOperatorForwardDirection`, `withCANBusName` becomes `withNetwork(CANBus)`, and the hoot-path `CANBus` constructor becomes `HootReplay`.
- LimelightLib beta9 (even the alpha5-6 build) moves the nested `Limelight.PoseEstimate` and similar types to top-level `com.limelightvision.PoseEstimate`. It also renames `setPriorityTagID` to `setPriorityTagIDOverride`/`clearPriorityTagIDOverride`, and adds a `flush` argument to `setRobotOrientation`.

## 1. SystemCore hardware

- **CPU:** Raspberry Pi CM5, quad-core Cortex-A76, 4 GB, VideoCore VII. An RP2350 handles the I/O [confirmed: [FIRST announcement](https://community.firstinspires.org/introducing-the-future-mobile-robot-controller), frc-docs source].
- **Kernel:** Linux 6.12 "with built-in preemption" (PREEMPT_RT) since image 10 [confirmed, [OS README](https://github.com/LimelightVision/systemcore-os-public)].
- **Thread priority:** The robot program does **not** run at RT priority by default. In [#352](https://github.com/wpilibsuite/SystemcoreTesting/issues/352), loop spikes came from a diagnostics daemon preempting robot code; it was fixed in image 14. RT priority 1 also removed the spikes, but maintainers discourage it because of starvation risk. The HAL CAN read thread sets RT priority 50 [confirmed, `hal/.../systemcore/CAN.cpp`].
- **CAN, five native buses:** `can_s0..s4` are five MCP2518FD SPI controllers. HAL also reserves `can_d0..d19` for Motioncore [confirmed: HAL `kNumCanBuses=25`, `CAN.cpp`; [rpi5 guide](https://github.com/netarcx/systemcore-rpi5-guide)].
- **CAN FD is not ready yet.** As of Oct 7 2026 the native buses run CAN 2.0 at 1 Mbps. FD can only be enabled by hand with SocketCAN commands, and FIRST says FD should be in a good state "in about a month" or may slip to next season, with **CANivore as the 2027 fallback**. Without a CANivore, Phoenix swerve odometry runs at **100 Hz**, not 250 Hz [confirmed, [#412](https://github.com/wpilibsuite/SystemcoreTesting/issues/412)].
- **CAN throughput:** The SPI limit is about 11.5k frames/s per bus. FD at 5–8 Mbps dropped frames at 60–65% utilization before the fixes in image 13 [confirmed, [#342](https://github.com/wpilibsuite/SystemcoreTesting/issues/342)].
- **CANivore rules:** A CANivore is legal in 2027 as an interim measure, and that allowance is removed for 2028 [doc, [rule preview](https://community.firstinspires.org/2026-event-robot-rule-preview-for-2027-season-part-1)]. **The template's `CANBus("canivore")` is the right choice for 2027**: 250 Hz FD odometry, plus CANivore time sync for Pro users.
- **Onboard IMU:** The part number is unpublished. In the HAL, IMU data arrives over **NetworkTables from a system process** (`/imu/yaw_flat` and similar), timestamped with NT time [confirmed, `hal/.../systemcore/IMU.cpp`]. Release 14 "significantly improved no-motion yaw drift", but there are no drift figures and a fault history exists (#135, #351, #126) [confirmed, OS README and issue list].
  - **Verdict: keep the Pigeon 2.** CTRE swerve fuses the Pigeon in its own CAN-synchronous odometry thread. The SystemCore IMU is an asynchronous NT feed with no published spec [speculative but well-founded].
- **USB vision on SystemCore:** Four USB vision instances, `limelightsc0..3` (`Limelight.SYSTEMCORE_USB0..3`), run Limelight's pipeline software on SystemCore itself. They have curated default calibrations for 20 USB cameras (OV9281, OV2311 and others), and Hailo accelerators work on USB ports 0 and 1. "Systemcore USB instances always use an external heading, even when using the Systemcore IMU" [confirmed, [LimelightVision.md](https://github.com/wpilibsuite/SystemcoreTesting/blob/main/LimelightVision.md), OS README r12/r14]. These instances share SystemCore's clock exactly, but they compete with robot code for 4 cores. PhotonVision has **no SystemCore image** [confirmed, PV repo].
- **Timebase:** On SystemCore, `wpi::util::NowDefault()` is raw `steady_clock` (CLOCK_MONOTONIC): *"We want clock synchronized across the system, so just use steady_clock"*. The HAL monotonic time, `Timer.getMonotonicTimestamp()` and NT local time all use it [confirmed, `wpiutil/.../timestamp.cpp` at alpha-6]. Alpha-7 makes all timestamps steady_clock in nanoseconds (#9074, #9335).
- **Post-alpha-7 `main`:** Adds AOS as the RT transport inside `mrccomm`, bridged to NT (#9415). This affects DS/control-data timing, not user APIs [confirmed, commit message].

## 2. WPILib 2027

**Commands v3 semantics for a long-running drive command that forks mechanism commands** [confirmed, `Scheduler.java`/`Coroutine.java`/`Command.java` on `main`]:
- **Requirements:** `drivetrain.run(co -> {...})` requires only the drivetrain. Forked children may require *other* mechanisms without the parent declaring them. Built-in `Command.parallel`/`race`/`sequence`, in contrast, own the union of requirements for their whole duration.
- **Start and lifetime:** A forked child starts **in the same scheduler cycle**, skipping the queue. It is **cancelled when the parent exits** and can never outlive the parent.
- **Priority:** A child's effective priority is the max over its ancestry, so raising the drive command's priority protects its children.
- **Trap A, forking cancels whole groups:** Forking `arm.scoring()` cancels the **root** of whatever holds the arm. In this template that is the whole `stow()` parallel group, so the flywheel stops too.
- **Trap B, interrupting a child cancels the drive:** If the driver later triggers any arm command at equal or higher priority, the scheduler cancels the root of the conflicting child, which is **the drive command itself**. The Javadoc says: "Interrupting one of the inner commands while it's running will cancel the entire sequence."
- **Sibling conflicts:** Two forked siblings that share a mechanism throw `IllegalArgumentException`.
- **Fork failures:** In alpha-7, `fork`/`await` return `ForkResult`, and `setCancelOnForkFailure(false)` lets code handle failures. In alpha-6 they return `void`.
- **New on `main`:** `Command.onExit()` (#9402), robot-mode scopes (#9271), `Mechanism.controllableDuringDisabled()` (#9417), and an await run-ID fix (#9531).
- **Closest official pattern:** The **`rebuiltcmdv3`** example. `followPath` does `co.fork(driveFieldRelative(follower::next)); co.waitUntil(follower::isDone)`, and `shootAt` does `co.await(aimAt(...)); co.awaitAll(intake.agitate(), shooter.shootAtHub(...))` ([source](https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/org/wpilib/examples/rebuiltcmdv3)). There is **no official drive-to-pose or tag-alignment example**, and `HolonomicDriveController` is absent from both alpha-6 and `main` [confirmed, source listing].

**Math** [confirmed, wpimath source]:
- **Kinematics:** `ChassisVelocities`, `ChassisAccelerations` and `SwerveModuleAcceleration` exist in alpha-6. `main` finishes the "speeds" to "velocities" rename (#9427) and renames `Units` to `UnitConversions` (#9469).
- **Pose estimator:** `PoseEstimator` keeps a **1.5 s** buffer (`BUFFER_DURATION`). `sampleAt(t)` returns `Optional<Pose2d>`. Vision older than buffer-start is dropped.
- **Buffers:** `TimeInterpolatableBuffer.createBuffer(sec)` with `addSample`/`getSample` is unchanged.
- **Twist math:** `Pose2d.exp/log` are gone. Use `Twist2d.exp()` to get a `Transform2d`, `Transform2d.log()` to get a `Twist2d`, and `poseA.minus(poseB)` to get a `Transform2d`.
- **Other API changes:** `Translation2d.getAngle()` now returns `Optional` (alpha-7, #9266), which matters for aim-at code.
- **Trajectories and utilities:** Alpha-7 rewrote trajectories into `HolonomicTrajectory`/`HolonomicSample`/`DrivetrainSplineTrajectory*` (#8172). `main` is preparing `TrapezoidProfile` on top of `Trajectory` (#9524), so expect the profile API to move again. Alpha-7 also added `AntiTipping` and `TwoDeadWheelOdometry`.

## 3. CTRE Phoenix 6 for 2027

- **Timebase:** The 26.50.0-alpha-1 changelog (June 12 2026) says *"Phoenix 6 and WPILib use the same timebase on the Systemcore"* and removes `fpgaToCurrentTime`/`currentTimeToFPGATime`. It also removes the Tuner-template `CommandSwerveDrivetrain` overrides of `addVisionMeasurement`/`samplePoseAt` [doc, [changelog](https://api.ctr-electronics.com/changelog.html) via search; paraphrased in [Biocore #46](https://github.com/TripleHelixProgramming/Biocore/issues/46)].
  - This is corroborated: CTRE's [2026 `main` example](https://github.com/CrossTheRoadElec/Phoenix6-Examples) wraps both methods in `Utils.fpgaToCurrentTime`, while the `2027_alpha5` and `2027_alpha7` branches drop the wrappers and pass `llMeasurement.timestampSeconds` straight in [confirmed, cloned branches].
  - The 26.70 Javadoc text still mentions `fpgaToCurrentTime`, which looks like stale boilerplate [doc].
- **`waitForAll`:** Works on **all** SystemCore buses since 26.50.0-alpha-1 [doc, changelog].
- **`samplePoseAt(double)` → `Optional<Pose2d>`:** A `SwerveDrivetrain` base-class method listed in the 26.70.0-alpha-2 Javadoc. Only the template override was removed, so it should exist in 26.50 [doc]. I could not open the jar because Maven is blocked; a one-line compile check settles it.
- **Pose history:** CTRE's internal `impl::SwerveDrivePoseEstimator` uses its own `TimeInterpolatableBuffer` [doc]. The length is undocumented; it is likely 1.5 s like WPILib [speculative].
- **Fused output:** `samplePoseAt` returns the **fused** estimate, so it includes vision corrections [doc/speculative].
- **Drive state:** `SwerveDriveState.Timestamp` is in the `Utils.getCurrentTimeSeconds()` timebase, which is the WPILib timebase on SystemCore. `OdometryPeriod` is the measured loop period, and `SuccessfulDaqs`/`FailedDaqs` count acquisitions [doc].
- **Odometry rate:** The default is 250 Hz on CAN FD and 100 Hz on CAN 2.0. `isOnCANFD()` exists [doc].
- **`addVisionMeasurement`:** Same semantics as WPILib's latency-compensated Kalman update. The template's MT2 configs set theta std-dev to `UNTRUSTED`, so vision never fixes heading [confirmed, LimelightLib `PoseEstimateConfig`].
- **`LinearPath`:** Used by `DriveToPose`. Its API was unchanged in 26.70 as far as the CTRE examples show [speculative].

## 4. Limelight

- **Limelight 4 hardware:** CM5, OV9281 **global-shutter** mono sensor (1280×800 at 120 fps), **built-in IMU** (part number unpublished), optional Hailo-8. Pipeline latency is listed at 3–7 ms [doc, [Robotics247](https://www.robotics247.com/article/limelight_hailo_introduce_limelight_4_smart_camera_at_ces_2025), AndyMark listing].
- **IMU modes for MegaTag2:** `EXTERNAL(0)`, `EXTERNAL_SEED_INTERNAL(1)`, `INTERNAL(2)`, `INTERNAL_MT1_ASSIST(3)` and `INTERNAL_EXTERNAL_ASSIST(4)`, plus `setIMUAssistAlpha` [confirmed, LimelightLib `IMUMode.java`].
  - Each frame reports `interpbotyaw`, the "interpolated robot yaw used for botpose". So the camera interpolates the heading you publish to its capture time [confirmed, `RobotOrientationData`].
  - Seeding in mode 1 while disabled and then running mode 4 removes NT heading latency. That should let the template drop its `spinningTooFast` MT2 rejection [speculative].
- **Frames and coordinates (LimelightLib 2 on "Limelight OS 2027.0+"):** All 3D data uses NWU. Target space has +X out of the tag face. `tx/ty/txnc/tync` keep the optical convention. Per-fiducial fields include `robotPoseTargetSpace`, `targetPoseRobotSpace` and `cameraPoseTargetSpace`, and `corners` are sent only when "send corners" is enabled. `DetectorTarget` carries `classId`, `confidence` and `trackId` [confirmed, source].
- **Frame queue:** The queue holds 20 frames. Use only one queue method per camera. `readRawFrameQueue()` + `Limelight.decode(bytes, ts)` exist for log replay [confirmed].
- **Timestamps are *not* hardware-stamped in the WPILib timebase.** LimelightLib computes `timestampSeconds = TimestampedRaw.timestamp/NT_TICKS − (captureLatency + targetingLatency)/1000` [confirmed, `Limelight.getPoseEstimate`, beta2 and beta9 alike].
  - The NT stamp is the client-sent value timestamp expressed in robot time. The ntcore server keeps client timestamps, and the NT4 client computes its offset **once per connection** from the first ping [confirmed, `ntcore` `ClientImpl.cpp`/`ServerClient4.cpp`].
  - So for an Ethernet LL4, clock drift and offset error add directly as timestamp bias [speculative magnitude: crystal drift at about 20–50 ppm accumulates to roughly 1–3 ms per minute of connection, which matters for long-lived connections].
  - PhotonVision documents the NT4-sync problems ("messages arrive at the server timestamped in the future") ([time-sync.md](https://github.com/PhotonVision/photonvision/blob/main/docs/source/docs/contributing/design-descriptions/time-sync.md)).
  - Each frame also carries `ntTimestampMicros`, the camera's own estimate of server time. Log it next to the receive stamp to measure the bias.
  - LL docs define `cl` as end of middle-row exposure to pipeline start [doc, from memory of docs.limelightvision.io, not re-verified].
- **Most precise single-tag robot-relative pose with an exact timestamp:**
  1. Consume every frame from `readResultsQueue()`, not `getLatestResults()`.
  2. For the target ID, take `robotPoseTargetSpace` and its frame's compensated timestamp.
  3. **Replace the single-tag PnP yaw** (the weakest axis) with gyro yaw relative to the tag's surveyed field yaw. Equivalently, compute `tagFieldPose.relativeTo(MT2 pose)` with `setFiducialIDFiltersOverride(new int[]{id})` so MT2 solves from that tag only. MT2 *is* a gyro-constrained single-tag solve.
  4. Forward-propagate with the odometry delta from capture to now.
  5. Error check: a 1° heading error at 2 m is about 3.5 cm of lateral error, so seed and verify the gyro first [speculative design].

## 5. PhotonVision 2027

- **Release state:** The latest tag is `v2027.0.0-alpha-2` (May 27, built on WPILib alpha-6). The PhotonLib build for alpha-7 is a Dev release [confirmed, [release](https://github.com/PhotonVision/photonvision/releases/tag/v2027.0.0-alpha-2)].
- **Timestamps (technically stronger than Limelight):** It has a UDP **time-sync protocol** that runs continuously, separate from NT. PhotonLib starts the server, results carry capture timestamps already in robot time, and there is a timesync alert. Capture time comes from **V4L start-of-exposure** CLOCK_MONOTONIC stamps [confirmed, `PhotonCamera.java`, [e2e-latency.md](https://github.com/PhotonVision/photonvision/blob/main/docs/source/docs/contributing/design-descriptions/e2e-latency.md)].
- **Solvers:** Strategies include `CONSTRAINED_SOLVEPNP` (`headingFree`, `headingScaleFactor`), `PNP_DISTANCE_TRIG_SOLVE` and coprocessor multi-tag, all fed by `addHeadingData(timestamp, heading)` [confirmed, `PhotonPoseEstimator.java`].
- **Realistic for the template?** **Not on alpha-6**, because no PhotonLib exists for it. On alpha-7 it is plausible with a separate coprocessor and Dev builds, which is extra risk for a teaching template [speculative].

## 6. Latency budget and how to measure it

The chain, assuming LL4 over Ethernet, a CANivore and a 50 Hz main loop [speculative numbers]:

| Stage | Typical | How to measure |
|---|---|---|
| Exposure (stamp to mid-exposure) | ~0.5–2 ms | known from exposure setting |
| Readout to pipeline start (`cl`) | ~3–10 ms | logged per frame |
| Pipeline (`tl`) | 3–7 ms | logged per frame |
| NT publish to robot | <1 ms, plus sync bias | `receiveTs − ntTimestampMicros` |
| Wait for robot loop | 0–20 ms | `robotNow − receiveTs` |
| `setControl`, then applied on next odometry tick | 0–4 ms at 250 Hz | [doc: CTRE applies requests in the odometry thread] |
| CAN to motor, plus the motor's closed loop | ~1–2 ms, then mechanical lag | `ModuleTargets` vs `ModuleVelocities` |

Measurement tests:
- **LED test (timestamp bias):** Drive a SystemCore DIO LED in view of the camera. Log the robot time it turns on and the first bright frame's compensated timestamp. The difference is the bias; PhotonVision's method is described in e2e-latency.md.
- **Spin test:** Spin at constant ω facing a tag. Cross-correlate vision bearing against `samplePoseAt(captureTs)` heading. The residual lag should be about 0.
- **Odometry period:** Check that the logged `OdometryPeriod` is about 4 ms; 10 ms means a CAN 2.0 fallback.

## 7. Template-specific findings and recommendations

1. **`Vision.java` timestamp:** Passing `estimate.timestampSeconds` straight to Phoenix is **correct on SystemCore with Phoenix ≥26.50**: same epoch, and CTRE does exactly this. The `DriveMechanism` comment ("share the WPILib timebase") is right about the *epoch*. For an Ethernet LL4 it does not guarantee *accuracy*, because of the NT4 one-shot sync, so measure the bias once.
   - In sim, desktop WPILib time is wall-clock based (`zerotime_val + steady`). Re-verify the epochs before adding a sim-vision path [speculative].
2. **`DriveToTag`:**
   - **No latency compensation.** `getLatestResults()` is treated as "now", which is 20–50 ms stale. At 2.5 m/s that is 5–12 cm. Today it is masked because kP=0: the profile runs open-loop from the first reading and measurements only feed P.
   - **Fix:** Drain `readResultsQueue()`. For each frame of the target ID, compute capture time with the same formula as LimelightLib. Then compute `robotInTagNow = robotInTagAtCapture.plus(odomNow.minus(odomAtCapture))` from an **odometry-only** pose buffer.
   - **Why odometry-only:** `samplePoseAt` deltas include vision corrections. Build that buffer from `registerTelemetry` (odometry-thread rate, `state.Timestamp` + `ModulePositions` + `RawHeading`) into a `TimeInterpolatableBuffer`.
   - **Yaw:** Swap the PnP yaw for the gyro yaw relative to the tag, as in section 4.
   - **API rename:** Rename `setPriorityTagID` when upgrading LimelightLib.
3. **`DriveToPose`:** Fine structurally (`LinearPath` + `Utils.getCurrentTimeSeconds`). Its feedback uses the fused pose, so a vision jump becomes a PID kick. Consider limiting vision std-devs during the final approach, or switching to the tag-relative estimate inside about 1 m.
4. **Commands v3 composition:** Build "drive-and-prepare" as `drivetrain.run(co -> { co.fork(robot.arm.scoring()); ...loop with co.yield()...; co.await(...); })`. Teach traps A and B from section 2, and give the alignment command a higher `priority()` if driver buttons must not abort it.
5. **Logging for offline replay** (AdvantageKit; note that `Logger.AdvancedHooks.disableRobotBaseCheck()` means today is log-only, not deterministic replay):
   - **Raw frames:** Per camera, `readRawFrameQueue()` bytes plus the NT timestamp, in an `@AutoLog` inputs class. Decode with `Limelight.decode(bytes, ts)` so any strategy (MT1/MT2/target-space/tx-ty) can be re-run offline.
   - **Odometry samples:** Every odometry-thread `SwerveDriveState` (Timestamp, ModulePositions, RawHeading, Pose, Velocity, OdometryPeriod), queued from `registerTelemetry` and flushed on the main loop. Today `Telemetry` logs one state per 20 ms.
   - **Heading feed:** Each published heading and its timestamp, the camera's `interpbotyaw` and `imumode`, and the per-frame `cl`, `tl`, `ntTimestampMicros`, receive stamp and robot time at consumption.
   - **Command internals:** Goal pose, profile setpoint (pose and velocity), measurement used, its capture time, and the commanded `ChassisVelocities`.
   - **Phoenix hoot logs:** Keep them for full-rate CAN signals. They share the timebase on SystemCore.
6. **Upgrade timing:** Plan the jump to alpha-7 (image ≥14, Phoenix 26.70.0-alpha-2, LimelightLib beta9-alpha7, AdvantageKit alpha-5/6) before building on these APIs. Alpha-6 images are frozen at 13, and the newest IMU and CAN fixes are 14+ only.
