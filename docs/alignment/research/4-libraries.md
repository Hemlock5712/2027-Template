> Raw research notes gathered by AI research agents on 2026-10-11 for [the alignment decision doc](../README.md). Claims are tagged [code] (read in source), [doc] (talk/post/docs, often via search snippets — Chief Delphi was not reachable), or [unverified]. Check anything important at the source before relying on it.

# Drive-to-point and path-following options for 2027 (checked 2026-10-11)

Tags: [code] means I read the source. [doc] means a README, docs page, or release notes. [unverified] means an inference or search snippet only.

## Versions checked (latest offseason code)

| Library | Latest 2027 code | Command framework |
|---|---|---|
| WPILib | tag `v2027.0.0-alpha-7` (Aug 30 2026), main to Oct 9 2026 | v2 + v3 |
| PathPlannerLib | `2027` branch, "Update for WPILib alpha 7" (#1181, Sep 30 2026). No 2027 release; latest release is v2026.1.2 | **v2 only** (`org.wpilib.command2`) [code] |
| ChoreoLib | `v2027.0.0-alpha-3` pre-release (Sep 13 2026), `wpilibYear: 2027_alpha7` | **v2 only** (`vendordeps/CommandsV2.json`) [code] |
| BLine-Lib | `wpilib-2027` branch, **2027.0.0-beta.2** (Sep 28 2026), alpha-7 + Java 25 | **v3 and v2 adapters** [doc] |
| Phoenix 6 | **26.70.0-alpha-2** (PyPI, Sep 18 2026). Its Java twin is [unverified] because CTRE maven and the API site are blocked here | n/a |

**The template is behind on both.** It runs WPILib **alpha-6** and Phoenix **26.50.0-alpha-1**.

### Offseason changes that affect this decision

- **WPILib `TrapezoidProfile` was rewritten** (#8556, merged 2026-07-12; in alpha-7, not in alpha-6) [code]. It now uses a phase-space formulation that "correctly handles large velocity differences with small position differences". That is exactly the "arriving fast at a close goal" case. The API also changed: `totalTime()` became `duration()`, and `timeLeftUntil(double)` became `timeLeftUntil(State, State)`. `LinearPath`, `ProfiledPIDController` and the template inherit all of this.
- **`HolonomicDriveController` and `SwerveControllerCommand` are gone in 2027** (#8119) [code]. WPILib has no holonomic follower now.
- **WPILib's Trajectory API was rewritten** (#8172, in alpha-7) [code]. The new classes are `Trajectory<S>`, `HolonomicTrajectory` and `HolonomicSample`, where a sample holds field-frame `Pose2d`, `ChassisVelocities` and `ChassisAccelerations`, with kinematic interpolation. These are data types only; there is no controller. On main, #9524 (Oct 5) keeps refactoring this API.
- **`ChassisAccelerations` was added** (#8185) [code].
- **Commands v3 moved into allwpilib** [code]. `Coroutine.fork` and `await` now return a `ForkResult`. Other additions are `onExit`, robot-mode scopes and a v3 `SysIdRoutine`. One semantic to remember: **forked children are cancelled when the parent exits** (`Coroutine.java` Javadoc).
- **Phoenix 6 2027 alpha** [code, Python wheel]:
  - `ApplyFieldSpeeds` became `ApplyFieldVelocity`.
  - `LinearPath.totalTime()` became `duration()` (26.50 to 26.70).
  - `setOperatorPerspectiveForward` became `setOperatorForwardDirection`.
  - `PhoenixPIDController` is now Tunable/Telemetry.
  - `WheelForceCalculator.calculateFromVelocities(dt, prev, cur)` was added.
  - **There is no new drive-to-pose helper.** The request list is unchanged: FieldCentric, RobotCentric, ApplyRobot/FieldVelocity, FieldCentricFacingAngle, RobotCentricFacingAngle, SysId…
- **PathPlanner's `new-path` branch** (Sep 2026) is a GUI rewrite with tree-based autos, events on waypoints and branches, and a "new coordinate system" [doc: commit titles]. Its library impact is [unverified].
- **Choreo 2027 alpha-3 is a "Systemcore transitional release"** [doc, PR #1514]. It targets alpha-7 but keeps the 2026 schema, and "API shape and schema" changes are still to come. It adds `AutoChooserOpMode` [code], which helps an OpMode template.

---

## 1. PathPlanner (team 3015, MIT) — https://github.com/mjansen4857/pathplanner/tree/2027

- **Algorithm.** Bézier paths with a time-parameterized trajectory that includes module-level torque and friction limits from `RobotConfig` [code/doc]. Pathfinding uses **LocalADStar** (AD*) on a 0.2 m navgrid in a background thread, and supports `setDynamicObstacles` [code].
- **Starting from current velocity.** `FollowPathCommand` regenerates the trajectory from the current speeds unless they are within 0.25 m/s and 30° of the path's ideal start. The first state's module velocities are seeded from the measured speeds [code]. The path geometry itself stays fixed.
- **Pathfinding gotchas** (all [code]):
  - Within **0.5 m** of the target at start, it outputs zero and finishes immediately.
  - It stops accepting replans within **2 m** of the end.
  - `pathfindThenFollow` finishes when the remaining distance ≤ v²/2a.
- **Feedforward and feedback.** `PPHolonomicDriveController` = field-velocity FF + PID on X, Y and θ, plus `DriveFeedforwards` (per-module forces) passed to the output consumer [code].
  - `overrideXFeedback`, `overrideYFeedback`, `overrideXYFeedback` and `overrideRotationFeedback` let you replace the feedback with your own, such as a vision error, **mid-path** [code]. This is PathPlanner's only "moving target" hook.
- **Done** = trajectory time elapsed. If the goal velocity is under 0.1 m/s and the command was not interrupted, it sends zero speeds [code].
- **Changing the target** = no; the target is fixed when the command is built [code].
- **Events.** Event commands may not require the drive, and their requirements are added to the path command (v2 semantics) [code].
- **2027 status.** Builds against alpha-7 on a branch, v2 only. To use it from v3 you would call `PathPlannerTrajectory` and `Pathfinding` directly from your own v3 command. There is one main maintainer.

## 2. Choreo / ChoreoLib (SleipnirGroup, BSD-3) — https://github.com/SleipnirGroup/Choreo

- **Algorithm.** Offline time-optimal trajectory optimization (TrajoptLib/Sleipnir) with full drivetrain dynamics [doc]. There is no on-robot generation; the Java library only loads and samples files [code].
- **Constraints.** Velocity and acceleration limits, keep-in and keep-out zones, stop points, and angular acceleration (added 2026.0.3) [doc]. **Obstacle avoidance is offline only.**
- **Starting from current velocity.** No. Sampling starts at t=0 of a pre-planned state.
- **Feedforward and feedback.** The controller is user-supplied (`Consumer<SwerveSample>`). Each sample carries vx, vy, ω, ax, ay, α and **per-module forces X/Y** [code], which map directly onto CTRE's `withWheelForceFeedforwardsX/Y`.
- **Done** = `timer > totalTime`. On a normal end it sends the final sample once [code].
- **Changing the target** = no.
- **Triggers.** `AutoFactory` and `AutoRoutine` provide `atTime(event)`, `atPose`, `atTranslation`, `done()`, `chain()` and `spawnCmd()` (#1518). These are all v2 `Trigger`s [code].
- **2027 status.** The core classes are framework-free: `Choreo.loadTrajectory()`, `Trajectory.sampleAt(t, flip)` and `getEvents(name)` [code]. **A v3 follower is a small custom command.**

## 3. BLine (Edan Liahovetsky; announced as Team 2638 Rebel Robotics; BSD-3) — https://github.com/edanliahovetsky/BLine-Lib/tree/wpilib-2027

The team attribution comes from the Chief Delphi announcement via a search snippet [unverified]. The license copyright reads "Edan Liahovetsky" [code].

- **Algorithm.** Polyline waypoints, **no trajectory or time parameterization** ("There is no trajectory or trapezoidal motion-profile controller" [doc]). Each cycle [code, `FollowPath.execute`]:
  - speed = P(**total remaining path distance**), clamped to the segment's max/min velocity, pointed at the current waypoint;
  - plus cross-track P along the segment normal;
  - plus heading P, interpolated by geometric progress;
  - then a `ChassisRateLimiter` that limits |Δv| (vector magnitude), ω and α.
  - Waypoints hand off by radius or by projected progress.
- **Starting from current velocity.** Yes; the limiter is seeded from the measured field velocity [code].
- **Constraints.** Per-segment v, a, ω and α presets; no per-axis limits [doc].
- **Done.** Remaining distance ≤ tolerance and heading within tolerance. Velocity is not checked (the source carries `// TODO add final velocity tolerance` [code]). Setting a positive final minimum velocity gives a "rolling exit" for chaining into the next command [doc].
- **Changing the target** = no: "Editing the source affects the next run, never an active run" [doc]. `overrideRotation(supplier)` exists [doc].
- **2027 status.** **Native v3** `FollowPath.Builder` (drive is a `Mechanism`). Events are dispatched on `Scheduler.getDefault()` and can outlive the path [doc]. The README benchmarks against PathPlanner: 97% less computation and 66% lower waypoint cross-track error, in simulation only [doc].
- **Health.** Small project (about 23 stars), student-maintained, very active (commits on Oct 9 2026).

## 4. CTRE Phoenix 6 swerve utilities (proprietary CTRE license)

**`LinearPath`.** Read from the Python twin in 26.70.0a2, `phoenix6/swerve/utility/linear_path.py` [code]:

- `calculate(t, current, goal)` **re-plans on every call** from `current`. It builds:
  - a 1-D WPILib `TrapezoidProfile` along the straight line from `current.pose` to `goal`, starting at velocity `vx·cosθ + vy·sinθ` (where θ is the direction of the line);
  - an **independent** angular trapezoid, with the goal wrapped to the shortest direction.
- **Velocity that isn't along the line is dropped.** Its perpendicular component is silently discarded, so the setpoint jumps by |v|·sin(Δθ). A negative along-line component is handled because the profile reverses.
- The translation and rotation profiles are not time-synchronized. `isFinished(t)` returns `t >= max(duration)`, with **no pose check**.
- CTRE's documented usage is the rolling form: `current = path.calculate(dt, current, target)` [code docstring]. That form *does* allow the goal to change every cycle. The template instead uses fixed-start, absolute-time mode, so its goal is frozen.
- **The Python implementation has a bug in all three versions checked:** it builds Y from `initial_pose.x + pos·sinθ`. Check whether Java has the same bug [unverified].

**Other Phoenix utilities:**

- **`FieldCentricFacingAngle`**: heading PID (`PhoenixPIDController`) plus `targetRateFeedforward`, with an optional `maxAbsRotationalRate` [code].
- **`ApplyFieldVelocity`**: field-centric `ChassisVelocities` plus field-centric **wheel force feedforwards X/Y in newtons**. Its default drive request type is `OpenLoopVoltage` [code].
- **`WheelForceCalculator`**: takes mass and MOI and returns forces from `ChassisAccelerations`, or with `calculateFromVelocities(dt, prev, cur)` from successive velocities [code]. Use it for profiles that don't output forces.
- **`samplePoseAt(timestamp)`**: returns the pose from the drivetrain's odometry buffer at a timestamp on the `Utils.getCurrentTimeSeconds` epoch [code]. In Java it returns `Optional<Pose2d>` [unverified]. This is the latency-compensation primitive for vision-relative targets.

## 5. WPILib 2027

- **`TrapezoidProfile`** (fixed in alpha-7) and **`ExponentialProfile`**: 1-D profiles, `calculate(t, current, goal)`. `ExponentialProfile` uses `Constraints.fromCharacteristics(maxInput, kV, kA)` to model back-EMF, so it gives a more realistic top-speed approach [code].
- **`ProfiledPIDController`.** Each `calculate` advances the profile one period from the **previous setpoint** toward the current goal [code]. A goal changed with `setGoal` is therefore followed continuously, which suits moving targets.
  - It returns **feedback only**; you must add `getSetpoint().velocity` yourself.
  - `reset(pos)` assumes zero velocity. The template's `DriveToTag` calls exactly this, so it brakes to a stop at start whenever the robot was already moving.
  - Per-axis X/Y instances produce curved paths and up to √2·a combined acceleration.
- **Removed:** `HolonomicDriveController`. **Not added:** any new holonomic follower or drive-to-pose helper.

## 6. Potential fields (Repulsor and related)

- **6995 NOMAD `RepulsorFieldPlanner`** (Robot-2025; edited Jul 2026) [code]:
  - Goal attraction plus 1/d² repulsion from obstacles and walls.
  - Each cycle it takes one step of `maxSpeed·min(d/2, 1)·dt` along the net force.
  - It emits a Choreo `SwerveSample` (intermediate pose plus velocity), which a Choreo-style PID+FF follower tracks on CTRE swerve.
  - The goal is `setGoal(target.get())` **every loop**, so it handles moving targets.
  - The repo has no license file, so by default all rights are reserved [unverified]. It is still on the `edu.wpi` packages (2025/2026).
- **CurtinFRC "Repulsor"** (team 4788, 2026) [code]: https://github.com/CurtinFRC/Repulsor
  - A large autonomy stack: vector-field `FieldPlanner`, coarse global fallback planner, reactive bypass, behaviours, tracking and shot solving.
  - Speed law: **v = vmax if d > v²/2a, else √(2·a·d)**, smoothstep-scaled inside 0.40 → 0.02 m, with a stuck-abort.
  - **GPL-3.0** for the `util/Repulsor` subtree (copyright Paul Hodges), with MIT for the rest of the repo.
  - Embedded in a 2026 robot project (GradleRIO 2026.2.1), not a library and not 2027. Use it for ideas, not as a dependency.

## 7. Other notable code

- **6328 `DriveToPose` (2026, MIT)** [code]: https://github.com/Mechanical-Advantage/RobotCode2026Public (`commands/DriveToPose.java`). This is the best published moving-target design. It takes a **`Supplier<Pose2d>` goal**, and every cycle:
  - re-seeds a 1-D trapezoid from the **last setpoint** (its distance to the *current* goal, and the last setpoint velocity projected onto the new direction);
  - advances one loop period;
  - adds PD on the measured distance;
  - fades the feedforward to zero inside a 1–5 cm radius;
  - runs θ on a `ProfiledPIDController`, using the goal's rotation rate as feedforward.
  - It does not end on its own; callers use `withinTolerance()`.
- **PathBuilder** (team 4188, built on PathPlannerLib) and **9032's custom follower** ("similar to BLine") [unverified, search snippets only].

## 8. Commanded-velocity vs time-parameterized controllers

**Kinematic (commanded-velocity) law.** Command v = min(vmax, √(2·a_brake·d), kP·d) along the error, then slew-limit the velocity *vector* (|Δv| ≤ a·dt).

- It has **no clock**; the only state is the last command. The goal can move freely, and starting at speed is natural.
- Costs:
  - braking is just-in-time, so latency causes overshoot unless a_brake is set below the real deceleration;
  - √d has infinite slope at d = 0, so it needs a linear (kP) or smoothstep zone near the goal;
  - there is no explicit acceleration, so you must derive wheel-force FF with `calculateFromVelocities`.
- BLine (P on distance), Repulsor (√2ad) and 6995 (linear ramp) are all this family.

**Time-parameterized profiles** (Choreo, PathPlanner, LinearPath in fixed-start mode) plan from a start state to a **fixed** goal and are sampled by time.

- Strengths: exact FF (velocity, acceleration, forces), deterministic timing for events, and good constraint coupling.
- Weakness: a goal change means a restart. Restarting from the *measured* state injects noise; restarting from the *setpoint* is continuous.

**The two families meet.** Re-solving a trapezoid every cycle from the last setpoint (6328 style, `ProfiledPIDController`, or rolling-mode `LinearPath`) is the kinematic law plus an acceleration ramp, and it still gives a known acceleration for FF.

---

## Answers

**(a) Long path following that forks mechanism commands in v3.**

No major library ships a v3 path command except **BLine 2027 beta** [doc]. For a teaching template I recommend:

1. **Choreo trajectories sampled by a small custom v3 command.** It is about 60 lines: `sampleAt(t)` → PID+FF → `ApplyFieldVelocity` with module forces.
2. Run the auto as `Command.noRequirements(co -> …)`. It does `fork(follow)`, then `co.waitUntil(() -> follow.time() > t_event || nearPose(...))`, then `fork(intake)`, then awaits the path's `ForkResult`.
3. Because children die with their parent, **the auto coroutine (not the path command) must own the forks.**

Choreo is time-optimal and deterministic, its events are timestamps, and its core is framework-free.

- **Use BLine** if they want runtime-editable polylines with native v3 and no offline solver.
- **Use PathPlanner** only for runtime AD* around dynamic obstacles. Call its trajectory and pathfinding classes from your own v3 command, since `AutoBuilder` and `FollowPathCommand` are v2-only.

**(b) Final precise approach with a target refined every camera frame.**

Use a **rolling-horizon drive-to-point** (6328 pattern) in the **odometry/field frame at the drive loop rate**, not tag-space PID at camera rate.

- Each frame: `goal = samplePoseAt(t_capture) ⊕ robotToTag_measured ⊕ tagToScoringOffset`.
- Gate outliers, low-pass the result, and feed it through the goal `Supplier`.
- This "local" goal is anchored to the real element, so field-placement error cancels; only odometry drift during the last ~1 s remains.
- **Do not** use PathPlanner pathfinding here (0.5 m dead zone, 2 m replan lock), Choreo (static), BLine (frozen path) or fixed-start `LinearPath`.

**(c) Can one implementation do both?**

**Yes, for drive-to-point.** One class: `Supplier<Pose2d>` goal, rolling 1-D trapezoid along the current error re-seeded from the last setpoint, a 2-D velocity-vector slew limiter, PD trim with feedforward fading near the goal, and θ on a `ProfiledPIDController`. Then a goal swap from the global pose to the vision pose is continuous in commanded velocity.

The slew limiter is what keeps the *perpendicular* component that `LinearPath` and 6328 drop. Also blend or rate-limit the goal itself over about 0.2–0.5 s, so a big correction cannot demand an instant direction flip near the goal.

**For long paths** the same controller can chase a carrot point on a polyline, which is essentially BLine. Time-optimal or obstacle-aware legs are still better as trajectories. The practical hybrid is a trajectory leg ending at nonzero velocity, handed off to the rolling controller seeded with the current setpoint velocity.

**The template's `DriveToPose` is close.** Switching it to CTRE's documented rolling form (`state = path.calculate(dt, state, goalSupplier.get())`) plus a vector slew gives most of this. Also:

- replace the time-only `isFinished` with pose and velocity tolerances;
- upgrade to WPILib alpha-7 and Phoenix 26.70 for the fixed profile.

## Comparison table

| Option | Algorithm | Start moving | Obstacles | Constraints | End / done | FF + FB | Live target | 2027 / v3 | License / health |
|---|---|---|---|---|---|---|---|---|---|
| PathPlanner | Bézier + time-param; AD* pathfind | Yes (regen from speeds) | AD* navgrid + dynamic | v, a, ω, α, module torque/friction, zones | Time; pathfind: v²/2a | vel FF + XYθ PID + module forces | No (feedback overrides only) | alpha-7 branch, v2 only | MIT; 1 main dev, active |
| Choreo | Offline time-optimal | No | Offline keep-out | Full dynamics, zones | Time | Your PID + v/a + module forces | No | alpha-3 transitional, v2 triggers; core usable | BSD-3; active |
| BLine | Polyline, P(remaining dist) + cross-track + slew | Yes | None at runtime | Per-segment v/a/ω/α | Distance + heading tolerance | P only, no FF | No | **beta.2, native v3** | BSD-3; small, very active |
| CTRE LinearPath | 1-D trapezoid on a line + independent θ | Partly (perpendicular velocity dropped) | None | Linear v/a + angular | Time only | Profile vel; add PID + force FF | Rolling mode only | 26.70-alpha-2 | Proprietary; vendor |
| WPILib ProfiledPID | Per-axis trapezoid | Only if reset with velocity | None | Per-axis | atGoal | PID; add setpoint vel | Yes | alpha-7 | BSD; core |
| 6328 DriveToPose | Rolling 1-D trapezoid on error | Yes | None | v, a; θ v/a | External tolerance | profile FF + PD | **Yes** | Port (2026 code) | MIT |
| Repulsor (4788) / 6995 | Potential field, √(2ad) / linear ramp | Yes | **Runtime fields + fallback** | vmax, a_brake | Tolerance / stuck abort | Velocity command (+ follower) | **Yes** | 2026, not a library | GPL-3 / no license |

## Recommendation

- **Long autos:** Choreo trajectories behind a small home-grown v3 follower using `ApplyFieldVelocity` and module-force FF. The auto coroutine forks mechanism commands by time or pose.
- **Final alignment:** one rolling-horizon `DriveToPose(Supplier<Pose2d>)` in the 6328 style, used both for teleop snap-to and as the last auto leg. Its goal starts global and is replaced by a `samplePoseAt`-compensated vision goal.
- **Watch BLine 2027** as a v3-native alternative if a path editor on the robot side matters more than time-optimality.
- **Before tuning anything,** move the template to WPILib alpha-7 and Phoenix 26.70.0-alpha-2.

Sources: [PathPlanner 2027 branch](https://github.com/mjansen4857/pathplanner/tree/2027), [PP releases](https://github.com/mjansen4857/pathplanner/releases), [Choreo](https://github.com/SleipnirGroup/Choreo), [Choreo PR #1514](https://github.com/SleipnirGroup/Choreo/pull/1514), [BLine-Lib](https://github.com/edanliahovetsky/BLine-Lib), [BLine 2027 guide](https://github.com/edanliahovetsky/BLine-Lib/blob/wpilib-2027/WPILIB_2027.md), [phoenix6 on PyPI](https://pypi.org/project/phoenix6/26.70.0a2/), [allwpilib](https://github.com/wpilibsuite/allwpilib) (#8556, #8119, #8172, #8185, #8479), [6328 2026 code](https://github.com/Mechanical-Advantage/RobotCode2026Public), [CurtinFRC Repulsor](https://github.com/CurtinFRC/Repulsor), [6995 Robot-2025](https://github.com/frc6995/Robot-2025), [Repulsor CD thread](https://www.chiefdelphi.com/t/repulsor-a-field-aware-autonomous-robot-library/514314), [BLine CD thread](https://www.chiefdelphi.com/t/introducing-bline-a-new-rapid-polyline-autonomous-path-planning-suite/509778).
