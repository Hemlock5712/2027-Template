# Prompt 1: build the advanced drive code

Start a new Claude Code session with **Hemlock5712/Workshop-Code**, attach
`drive-precision-handoff.zip`, and paste everything below the line.

---

You're helping FRC team 5712 build an **advanced, team-first** version of drive-to-point and AprilTag
alignment for WPILib 2027 alpha-7 (SystemCore, Commands v3, OpModes, CTRE Phoenix 6 swerve,
Limelight 4). It's a separate track. Local schools learn from the simple projects in Workshop-Code
(`5-DriveToPoint`, `6-ProfiledToPoint`, `7-InlineCommands`), and those stay exactly as they are. Don't
change, rename, or "upgrade" any existing Workshop-Code branch in this task.

## Read first

1. Unzip the attachment into your scratchpad, not into the repo. Never commit it.
2. `README.md` and this prompt, from the zip.
3. `reference/research/README.md`: the decisions table, the design and the robot test plan. Then
   `reference/research/studies/controllers.md` (sections 6 and 7: the physics limit, timing, B-Line)
   and `reference/research/studies/tag-tracking.md`.
4. `reference/2027-template/src/main/java/frc/robot/`: the working prototype. **This is where the
   logic comes from.**
5. Hemlock5712/Workshop-Site `CLAUDE.md`, the section "Workshop Content Stack". Attach it read-only
   with `add_repo` and never push there. Workshop-Code has no CLAUDE.md, so this section is the
   convention list for its code. Where it and the shipped WPILib alpha-7 source disagree, the
   shipped source wins. Never invent an API.

## Step 0: ask where this lives, before writing code

Workshop-Code and 2027-Template are public, and schools use them. This work is ours first, maybe
shared with other teams later. Ask the user where to put it:

- **(Recommended) A new private repo**, e.g. `Hemlock5712/Precision-Drive`, seeded from
  Workshop-Code's swerve project. The user creates it, or you create it with their OK; then attach it
  with push access. It can be shared later by inviting teams or making it public.
- The team's competition repo.
- Public Workshop-Code branches, only if the user says it should be public.

Also ask the three code decisions from the zip's README: gains (simulator values or `0.0`
placeholders), whether to include game pieces, and the branch plan below.

## Do not use these as models

Each of these is in your reach and each one will mislead you:

- **`5-DriveToPoint`, `6-ProfiledToPoint`, `7-InlineCommands`.** These are the simple teaching
  versions: P control per axis, a CTRE `LinearPath` trapezoid plus PID, and a camera-only
  `ProfiledPIDController` per axis with `LimelightHelpers.setPriorityTagID`. The research replaced
  every one of those ideas. P on distance alone either overshoots or crawls, per-axis control drives
  curved paths, and camera-only control has no latency compensation. Copy no structure, constants or
  names from them.
- **The old swerve chain (`1-Swerve` to `4-DynamicFlywheel`)** for vision. It still copies
  `LimelightHelpers.java` and names its subsystem `Limelight`. Alpha-7 vision is the LimelightLib 2
  vendordep, and the class is `Vision` (see the site's CLAUDE.md). Use the chain's drivetrain
  scaffolding as a base only after checking it against the conventions below.
- **The template code in the zip, for its shape.** It is alpha-6 and behind: `extends Mechanism`,
  `kZero`, AdvantageKit `Logger.recordOutput`, method references like `wheelOdometry::update`, and the
  `LimelightLib-alpha5-6` vendordep. Port its **logic**, not its shape.
- **The site's lessons and videos** `drive-to-point`, `advanced-drive-to-point`,
  `drive-to-tag-inline`. They teach the simple track.

The `mech-*` chain is the best example of current conventions (rebuilt August 2026).

## What to port

| Prototype file (zip) | What it does | Notes for the port |
| --- | --- | --- |
| `utils/AccelerationLimiter.java` | Caps how fast the commanded velocity may change. Heading accel cap, Kraken X60 FOC torque-speed limit (dyno numbers, applied only when speeding up), and a friction circle shared by driving and turning | Keep the dyno `DCMotor` with **free current 0** (DCMotor counts friction as push otherwise). Keep `TRACTION_ACCEL` above DriveToPose's `DECEL` and `ANGULAR_ACCEL` above `ANGULAR_DECEL` |
| `commands/DriveToPose.java` | One controller for any goal. Speed = min(max speed, braking-distance speed, KP·d), one vector straight at the goal, corner rule for `passingThrough(next)`, heading on the same plan, then `AccelerationLimiter` | Its Javadoc lists four questions the lesson answers. Keep `goal()` overridable (DriveToTag uses it) |
| `subsystems/WheelOdometry.java` + the `DriveMechanism` hooks (`getOdometryPose`, `getOdometryPoseAt`) | Wheel+gyro pose that vision never touches, with 1.5 s of history | Fed from CTRE's odometry thread, so its methods stay `synchronized` |
| `subsystems/vision/TagTracker.java` | Tracks one tag in the odometry frame at each frame's capture time: a small per-axis Kalman filter, an ambiguity gate, per-camera corrections | It logs every frame **before** the ambiguity skip; log replay depends on that |
| `subsystems/vision/CameraCorrections.java` | Per-camera `Transform2d` fix, applied as correction ⊕ measured | Order matters: `ZERO.plus(correction).plus(measured)` |
| `commands/DriveToTag.java` | DriveToPose whose goal starts as the field-map spot and switches to the tag-measured spot once locked | It finishes only once locked, so autos add `.withTimeout(...)` |
| `opmodes/CalibrateCamerasOpMode.java` | Utility OpMode. Park at the spot, back off slowly, average each camera's error, print `Transform2d` lines for CameraCorrections | It stops the back-off itself with a zero-velocity request |
| `opmodes/DriveThenAlignOpMode.java`, `AutonomousOpMode.java`, `DriveStowDriveOpMode.java`, the bindings in `TeleopOpMode.java` | Examples: a staging leg with `passingThrough`, then DriveToTag; chained legs; a race with a mechanism | Rebuild them in the current OpMode style (the site's CLAUDE.md: `robot.x` directly, `() -> foo()`) |
| `subsystems/vision/Vision.java` | Global pose fusion with LimelightLib 2 | Only if the base has no LimelightLib 2 `Vision` yet |
| `commands/DriveToPiece.java`, `subsystems/vision/PieceTracker.java` | Game-piece version | Only if the user wants it. Less tested |
| `generated/TunerConstants.java` | `kSlipCurrent` 360 A | A hardware ceiling only. **Don't lower it to limit acceleration**: weight transfer needs the loaded wheels to push hard. `AccelerationLimiter` limits acceleration instead |

## Conventions (alpha-7)

- `implements Mechanism`, `ZERO` (not `kZero`), `ChassisVelocities`, `org.wpilib.*` packages.
- Lambdas are always `() -> foo()` or `state -> foo(state)`, never `Foo::bar`. No enums.
- Logging is WPILib `Telemetry` (`Telemetry.getTable("TagTracker").log("TagInRobot", pose)`), as
  Workshop-Code's `DriveMechanism` does. **Never AdvantageKit.** Keep every key the prototype logs,
  because `alignsim.replay` reads them.
- LimelightLib 2 for alpha-7 (`LimelightLib-alpha7.json`, `2.0.0-beta9-alpha7`). Read the API names
  off that jar. The prototype uses `getLatestResults()`, `frameIndex`, `getAgeSeconds()`,
  `captureLatencyMillis`, `targetingLatencyMillis`, `fiducialTargets`, `getTargetPose_RobotSpace()`,
  `ambiguity` and `fiducialId`. `Vision` drains `readResultsQueue()`; make sure TagTracker still sees
  every frame.
- Comments say **what**, one or two lines. The lessons (the videos) carry the **why**.

## Traps (each of these cost us a bug)

- **`SwerveRequest.Idle` is not a stop.** It leaves every module on its last request. End with
  `driveRequest.withVelocity(new ChassisVelocities())`.
- **`registerTelemetry` keeps only one callback.** Workshop-Code already uses it for logging, so feed
  WheelOdometry from the same lambda: `state -> { wheelOdometry.update(state); logState(state); }`.
- **Capture time on one clock:** now − (frame age + capture + targeting latency), on the Phoenix clock
  (`Utils.getCurrentTimeSeconds()`). On SystemCore, Phoenix, WPILib and NetworkTables share a clock;
  still, never mix a camera timestamp with a different time source.
- **Don't fuse a camera result twice** by reading the same frame in two places without its
  `frameIndex` check.

## Branch plan (propose it, then wait for the user's OK)

One commit per lesson, in a linear chain, so each diff is exactly one lesson:

1. `main`: the swerve base, alpha-7 conventions, vision on LimelightLib 2 `Vision`.
2. `precision-1-DriveToPose`: `AccelerationLimiter`, `DriveToPose`, a teleop binding and the example
   autos. This is videos 1 and 2.
3. `precision-2-DriveToTag`: `WheelOdometry`, `TagTracker`, `CameraCorrections`, `DriveToTag`,
   Calibrate Cameras, `DriveThenAlign`. This is video 3.
4. (optional) `precision-3-DriveToPiece`.

## Check your work

- `./gradlew build` and `./gradlew spotlessApply`. If Gradle can't download, say so plainly. Never
  push code you couldn't compile without telling the user.
- **Motor numbers:** with the example TunerConstants (7.36:1, 2.167 in wheel) and 60 kg,
  `AccelerationLimiter.motorAccel` at 3.0 / 3.5 / 4.0 / 4.3 / 4.53 m/s should give about
  26.3 / 19.0 / 9.7 / 4.2 / 0 m/s². Add this as a JUnit test if the repo has tests set up
  (`mech-6-Testing` shows the shape).
- **Simulation:** DriveToPose to a point 3 m away settles inside 3 mm and 0.3° with no overshoot, and
  `passingThrough` keeps speed through the corner. Check the logs.
- **The Python simulator** still runs: `cd reference/sim && python3 -m pytest -q alignsim/tests`.
  `python3 -m alignsim.replay <file>.wpilog --tag <id>` replays a robot log once one exists.

## Report back

The branch list and one line per lesson, build and simulation results, every API that didn't port
one-to-one and what you did, and the decisions still open. Then stop: the videos (prompt 2) come
next, in another session.
