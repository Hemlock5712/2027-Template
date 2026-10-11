> Raw research notes gathered by AI research agents on 2026-10-11 for [the alignment decision doc](../README.md). Claims are tagged [code] (read in source), [doc] (talk/post/docs, often via search snippets — Chief Delphi was not reachable), or [unverified]. Check anything important at the source before relying on it.

# 3 — Elite-team survey: drive-to-pose and final alignment (excluding 6328/254/1690/581/2056)

**Method.** I read code from shallow clones of the newest public repos, preferring 2026 and 2026-offseason/2027-alpha over 2025. www.chiefdelphi.com was blocked for fetching, so every Chief Delphi (CD) claim comes from search-result snippets and is marked **[unverified]**. Tags: **[code]** = I read it in the repo; **[doc]** = official document or a repo's design doc.

---

## 1. Field-tolerance facts

- **Manual.** Arena dimensions are "nominal." Tolerances live in the drawings, and the title blocks say fractional ±1/2 in and angular 1° ([2025 Arena PDF](https://firstfrc.blob.core.windows.net/frc2025/Manual/Sections/2025GameManual-05ARENA.pdf)) [doc]. The standing manual line is that arenas are "assembled in different venues by different event staff and some small variations occur" [doc].
- **2025 Team Update 12** gives two tag maps, welded and AndyMark, and warns that "field setup variation and movement during the event will exceed the differences between these two maps" ([CD](https://www.chiefdelphi.com/t/2025-team-update-12/491724)) [doc via snippet]. I diffed WPILib's two JSONs. Reef tags differ by 0.2 in, coral-station tags by 0.8–1.1 in, and processor tags by about 2.9 in [code]. So FIRST itself expects venue error of more than about 1–3 in.
- **Measured at events, 2024.** A PSA from the Hueneme Port Regional reports alliance walls that shifted, speaker tag sheets that bowed, and trusses rotated up to 8°. It says the ±½ in / 1° figures "are not what might be at an actual event" ([PSA](https://www.chiefdelphi.com/t/psa-field-tolerances-for-vision/457135)) [unverified].
- **2025.** One team's two cameras disagreed by 10–30 cm and they suspected tag placement ([thread](https://www.chiefdelphi.com/t/for-teams-who-know-a-thing-about-vision-multiple-limelight-vision-and-field-apriltag-setup-accuracy-questions/504982)) [unverified]. Another poster said the 12 scoring poses "must be tuned at each event" because of differences in reef peg placement and tag position ([thread](https://www.chiefdelphi.com/t/teams-how-do-you-tune-auto-align-and-vision-to-competition-fields-particularly-for-reef-alignment/495387)) [unverified].
- **Indirect measurement (Spectrum).** Spectrum's per-event offset tables move the reef standoff between 9.0 and 11.0 in, the lateral trim between 0 and −4 in, and the heading between 175° and 180°, depending on the tag and the event ([offsets/](https://github.com/Spectrum3847/2025-Spectrum/tree/main/src/main/java/frc/reefscape/offsets)) [code]. These numbers mix field error with robot changes, so they are an upper bound.
- **WPIcal docs.** Competition-field corrections "offer limited benefit." The tool is "exceedingly helpful" on practice fields, which are often out of tolerance ([WPIcal](https://docs.wpilib.org/en/latest/docs/software/wpilib-tools/wpical/)) [doc].

---

## 2. Teams (newest work first)

### 971 Spartan Robotics: from a custom C++ localizer to Java + an Orin coprocessor
- **2025, C++/AOS** ([y2025](https://github.com/frc971/971-Robot-Code/tree/master/y2025)) [code]
  - **Drive-to-pose.** `auto_align_updater.cc` picks the nearest reef tag from the *target map*. The goal is that tag plus a 6 in tangent offset plus a 3 in normal offset (left/right poles 13 in apart). If the robot is more than 5 cm off the approach line, the goal is pushed 0.2 m out from the reef, so the robot lines up first and then drives straight in. A `theta_lock` mode sets heading only.
  - **Localizer.** Camera capture time comes from the Orin and is corrected by the message-bridge clock offset. "Images from the future" are rejected, as are tags beyond 4.5 m and tags with high pose error or pose-error ratio. There is a tag whitelist and blacklist. The weighted-average variant ignores detections older than 100 ms, weights tags by 1+1/d², and blends at `vision_weight=0.01` every 10 ms. It rejects any update that would move the pose more than 0.5 m. **Neither localizer variant replays odometry from capture time:** the Kalman variant explicitly does `(void)capture_time`.
  - **Field mapping.** `frc971/vision/target_mapper` is a Ceres pose-graph SLAM. It solves tag-to-tag constraints from logged detections, freezes one tag, then fits the map into the world ([target_mapper.h](https://github.com/frc971/971-Robot-Code/blob/master/frc971/vision/target_mapper.h)). This is per-field calibration. The map checked into y2025 is the official one.
- **Nov 2025 Java rewrite of the 2025 robot** ([AutoAlignHandler.java](https://github.com/frc971/971-robot-2025/blob/main/src/main/java/frc/robot/commands/AutoAlignHandler.java)) [code]
  - Global pose with CTRE `FieldCentricFacingAngle` and one PID on the distance to the goal.
  - Done when the error *in the target frame* is under 0.11 m normal, 0.07 m tangent and 2.5°. Autonomous adds a 2.5 s timeout.
- **2026, Java + BOS** ([bos](https://github.com/frc971/bos), [971-second-robot-2026](https://github.com/frc971/971-second-robot-2026)) [code]
  - The Orin runs a GPU AprilTag detector and an IPPE-square single-tag solve.
  - An "unambiguous estimator" tries every combination of each camera's two single-tag solutions and keeps the most consistent set. It only uses frames within 0.25 s of each other.
  - BOS publishes x, y, θ, variance and the capture timestamp, shifted into robot time by the NT server-time offset.
  - The robot reads the whole topic queue (`readQueueValues`) and calls CTRE `addVisionMeasurement` with that timestamp. It hard-resets the pose if odometry is off the field.
- **What changed.** 971 abandoned the custom estimator and its event-mapping tooling. It kept its own detector and timestamps and handed fusion to the stock estimator. There is no tag-relative final alignment.

### 1678 Citrus Circuits
- **2025** ([C2025-Public](https://github.com/frc1678/C2025-Public)) [code]
  - **Vision.** One Limelight, MegaTag2 only, std-devs (0.3, 0.3, 99999) × average tag distance.
  - **Controller.** `PIDToPoseCommand` runs a PID on the scalar distance along the error vector, plus a heading PID.
  - **End condition.** 3 cm and 1.2° for L2–L4, held for 40 ms (debounced). `FollowSyncedTagPIDToPose` also requires a *raw* vision estimate that is under 100 ms old and inside the same tolerance of the goal. If none exists, the controller rumbles ("you're on your own"). This is a vision-confirmed finish.
  - **Synced mechanisms.** `FollowSyncedPIDToPose` switches to softer "tippy" gains near the end. It also flags `readyToRaiseElevator` when the robot is both close and slow.
  - **Bug.** The speed check is `hypot(vx, vx)`, so vy is never checked.
  - **Game piece.** `DetectionPIDToPoseCommand` turns each coral detection into a field pose and averages the last 15 detections to get the goal. Detections are gated by zone, by heading within 60°, and by a 40 ms debounce. The command ends on the beam break or after 3.5 s.
- **2026** ([C2026-Public](https://github.com/frc1678/C2026-Public)) [code]
  - Switched to PhotonVision: multi-tag PnP on the coprocessor, with `PNP_DISTANCE_TRIG_SOLVE` as the single-tag fallback (the gyro is fed in through `addHeadingData`).
  - Rejects any frame that contains a tag more than 5.5 m away, or when the intake covers the camera.
  - **Climb.** PID-to-pose with *separate X/Y tolerances*, then drive sideways into the tower (`hitTower`) to register on it.
  - 1678 also has a SystemCore alpha port, [C2025-SystemCore](https://github.com/frc1678/C2025-SystemCore).

### 3847 Spectrum (the most relevant 2027 work)
- **2025** ([2025-Spectrum](https://github.com/Spectrum3847/2025-Spectrum)) [code]
  - **Fusion.** Global pose with tiered std-devs (stationary and close, multi-tag, target area, ambiguity). MegaTag1 is used while disabled to seed heading; teleop uses MegaTag2 and MegaTag1.
  - **Goal.** Reef goal = tag pose + **per-tag, per-event offset tables**: `HomeOffsets`, `RobonautsOffsets`, `StateChampsOffsets`, `WorldChampsOffsets`.
  - **Image-space servos.** `TagCenterAlignController` (servo on tx) and `TagDistanceAlignController` (servo on tag area) exist, but the bound buttons use the pose-based XY align.
- **2026** ([2026-Spectrum](https://github.com/Spectrum3847/2026-Spectrum)) [code]
  - Three Limelights; only the best camera fuses (score = tag count + area).
  - Only MegaTag1 while enabled. A code comment says "MegaTag2 fuses the IMU, so its estimate drifts as soon as the robot moves."
- **Oct 2026: [2026-FM-SystemCore](https://github.com/Spectrum3847/2026-FM-SystemCore)** — WPILib 2027 alpha-6, AdvantageKit 27 alpha, Phoenix 26.50 alpha ([pose-sources.md](https://github.com/Spectrum3847/2026-FM-SystemCore/blob/main/docs/pose-sources.md)) [doc/code]
  - Odometry samples (250 Hz) are copied off CTRE's thread and fused on the main loop, so AdvantageKit replay reproduces them. CTRE's own estimate is then re-synced to the fused pose.
  - Every source (Limelight MT1, Limelight MT2, Orin/PhotonVision, SystemCore cameras, QuestNav) runs a named **gate chain**. Every frame is logged before gating with its verdict. Each source gets a **shadow estimator** ("what we'd believe with only this source").
  - Gates include ambiguity >0.9, outside the field, |z| >0.75 m, roll/pitch >5°, frames older than 1.0 s, and the **peak yaw rate over the *last 0.3 s* ≥ 1.6 rad/s**. The last one exists because "a frame arriving after a spin stops was captured during it."
  - The gyro owns heading while enabled. It is set by the disabled seed and by a "gross-heading net" (≥20° off, stationary, for 1 s). MegaTag1 is used until the seed is confirmed, MegaTag2 after.
  - An EKF with **per-tag bearing and distance** measurements runs beside WPILib's estimator for comparison.
  - **`OrderedVisionUpdates`** works around WPILib's estimator *discarding every stored vision update newer than the one being added*. I confirmed this behavior in [PoseEstimator.java](https://github.com/wpilibsuite/allwpilib/blob/v2026.2.1/wpimath/src/main/java/edu/wpi/first/math/estimator/PoseEstimator.java), "Step 8" [code].
  - **Sim results.** Fused error 1.4 cm, odometry alone 6.8 cm. MegaTag2 shadows were worse than MegaTag1 because MegaTag2 inherits gyro drift.
  - "Set shots": park at a known spot and fire with no pose.

### 2910 Jack in the Bot: tag-anchored hard reset
- **2025** ([VisionIOLimelight.java](https://github.com/FRCTeam2910/2025CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/subsystems/vision/VisionIOLimelight.java), [Superstructure.java](https://github.com/FRCTeam2910/2025CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/subsystems/Superstructure.java)) [code]
  - **No pose estimator.** The trig pose is built from the tag's map pose, `tx` scaled by an empirical 38.12/35.99, distance from the *pixel height* of the tag corners (with a per-camera scalar), and **gyro heading**.
  - While scoring, it calls `resetTranslation()` every loop from one observation. Only the expected tag counts: the one facing the robot's nearest 60° heading, seen by the camera on the scoring side.
  - Goal = the *same* map tag pose + offset. **Any map error cancels out**, so the robot aligns to the real tag.
  - **Drive-to-point.** P on distance (3.6 teleop / 3.0 auto) plus a static-friction feedforward (0.02·vmax when more than 0.5 in away), capped at 10 ft/s. The goal stays 6 in back until heading is within 5°.
  - **Release** at 1 in and 2° with the arm at setpoint.
  - The latency constant is **0.0**: no compensation.
- **2026** ([2026CompetitionRobot-Public](https://github.com/FRCTeam2910/2026CompetitionRobot-Public)) [code]
  - The distance formula became a measured pixel-to-distance lookup table (`VisionInterpolation`).
  - Their own TODO admits the reset "doesn't compensate for the latency... Fusing vision in a pose estimator would help."
  - Fallback "known translation" shots.
  - In the offseason they forked [AOS](https://github.com/FRCTeam2910/aos) and a Jetson image. I infer they are moving toward an AOS coprocessor; I can't confirm it [unverified].

### 604 Quixilver: the cleanest "local pose" implementation
- **2025** ([QuixSwerveLocalizer.java](https://github.com/frc604/2025-public/blob/main/FRC-2025/src/main/java/frc/quixlib/localization/QuixSwerveLocalizer.java), [ScoreCoralCommand.java](https://github.com/frc604/2025-public/blob/main/FRC-2025/src/main/java/frc/robot/commands/ScoreCoralCommand.java)) [code]
  - **`computeSingleTagPose()`** (the code credits 6328's RobotState):
    1. Take the newest frame of the *selected* tag.
    2. Distance comes from PnP; bearing comes from yaw/pitch.
    3. Heading is the **gyro interpolated at capture time**.
    4. Reset a separate odometry at the capture-time sample and **replay every odometry sample since**.
  - The global estimator is an offboard particle filter. It runs on the Driver Station laptop and is synced by measurement ID; the robot replays odometry on top of it. `setTagsToTrack({selectedTag})` restricts it during scoring.
  - The selected tag is the nearest one after projecting the pose 0.15 s ahead with current velocity.
  - Motion: profiled trajectories that are regenerated at each stage (prescore pose, then scoring pose).
  - **Finish:** |depth| <1.5 in, |lateral| <1.0 in and 1°, all *in the target frame*, with elevator, arm and wrist in tolerance, held for 50 ms.
  - Live depth/lateral trims over NetworkTables.
- **2026** ([2026-public](https://github.com/frc604/2026-public)) [code]
  - The particle filter is gone. The robot pose = **the newest multi-tag PnP** (0 ambiguity, prefer hub tags, under 0.25 s old) with gyro heading at capture, plus replayed odometry.
  - Every fix *replaces* the previous one; nothing is blended.
  - A separate trig pose uses hub tags only.
  - The README promises a 2027 SystemCore version ("FRC-2026-NEW").

### 3476 Code Orange: sensor-registered finish + BLine
- **2025** ([AutoAlignment.java](https://github.com/FRC3476/FRC-2025/blob/main/src/main/java/org/codeorange/frc2025/subsystem/AutoAlignment.java), [BranchAligner.java](https://github.com/FRC3476/FRC-2025/blob/main/src/main/java/org/codeorange/frc2025/subsystem/branchAlign/BranchAligner.java)) [code]
  - **Vision.** Limelight MegaTag1/MegaTag2. Frames are rejected if heading disagrees by more than 5°. Std-dev ∝ d²/n².
  - **State machine:**
    1. INTERMEDIATE pose, halfway out from the reef center (8 cm / 2°).
    2. GOAL (5 cm / 2°).
    3. SHOVE: drive 0.1 s at 10% toward the reef.
    4. **BEAMBREAK**: three beam breaks per side (front/middle/back) strafe the robot until the middle beam is the only one blocked, debounced 0.1 s.
    5. Error state if all beams are on or all are off for 1 s.
- **[BLine-Lib](https://github.com/FRC3476/BLine-Lib)** (Feb 2026; 971's 2026 robot uses it) [code]
  - PID on the *remaining path distance*. Velocity points at the current waypoint, which hands off inside a radius.
  - A cross-track PID, a rate limiter, and event triggers keyed in the path JSON.
  - This is drive-to-point chained into paths.

### 4481 Team Rembrandts
- [2026-robot-public](https://github.com/FRC-4481-Team-Rembrandts/2026-robot-public) [code]
  - `TOPPGenerator` builds a Bézier curve from the robot's *current pose and velocity* (control-point length scales with speed) and time-parameterizes it.
  - The `TrajectoryFollower` adds a "dynamic kP... when close to the reef."
  - Vision std-dev ∝ distance (×2 for a single tag). `preferenceIDs` halve the std-dev for preferred tags. Single-tag rotation is ignored.

### Forum-only reports [unverified] ([approaches](https://www.chiefdelphi.com/t/approaches-to-reef-auto-alignment/495742), [auto-align at comp](https://www.chiefdelphi.com/t/for-teams-using-apriltag-based-auto-align-has-it-worked-as-well-at-competition-as-it-has-at-home/495415))
- **5987:** "much better" after moving from global to tag-relative. They keep fusion but ignore every tag except the target, plus a second "local" estimator for that tag.
- **4561:** MegaTag2 to about 1 m out, then single-tag trig.
- **2491:** four distance sensors.
- **9021:** direct Limelight-tag servoing "didn't work out well."
- **Others** turned off rear cameras during alignment. One saw 5 cm pose jumps make a 1 cm-tolerance controller "move in circles."

---

## 3. Strategy clusters

| # | Approach | Teams | Pros | Cons |
|---|---|---|---|---|
| A | **Global pose → map goal** (tag pose + offset, PID-to-pose) | 1678, 971 (all versions), 3476 stage 1, 4481, Spectrum 2025 | Simple, one frame, chains with paths | Inherits tag-map error (cm to inches); jumps at close range |
| B | **A + per-event calibration** (offset tables, map solver, WPIcal) | Spectrum, 971 (tool), CD posters | Fixes systematic field error | Needs calibration-period time and discipline; doesn't fix movement during the event |
| C | **Tag-anchored reset**: re-localize from the target tag, goal = same tag + offset | 2910 (reset), 604-2026 (latest fix + replay, global) | Map error cancels; no tuning per event | Hard reset without replay = latency error (2910's own TODO); heading must come from the gyro |
| D | **Local single-tag pose + gyro + odometry replay** | 604-2025 (6328 lineage), 5987/4561 [unverified] | Exactly the "vision + twist since capture" ideal; robust to field error | Must pick the right tag; drifts once the tag is lost; needs a timestamped odometry buffer |
| E | **Restrict which tags count** while approaching | 604 `setTagsToTrack`, 971 whitelist, 1678 5.5 m cutoff, 4481 preferred IDs, 5987 | Kills distant-tag jumps cheaply | Fewer fixes; can starve the estimator |
| F | **Image-space servo** (tx / ta / pixel height) | Spectrum 2025 (unused), many CD teams | No map, no extrinsics | Loses the tag when close or rotating; coupled axes; teams report it unreliable |
| G | **Physical registration**: beam breaks, drive into the element, set shots | 3476, 1678-2026 climb, 2491, 2910/Spectrum set shots | Immune to vision; truly local | Mechanism-specific; slower; needs a contact surface |
| H | **Vision-confirmed finish** | 1678-2025 | Prevents scoring on a stale or drifting pose | Can stall when tags drop out (they rumble instead) |

---

## 4. Best ideas to steal

1. **The 604 recipe for the local pose.** Take a single-tag translation, use the gyro heading at capture time, and replay buffered odometry from the capture time to now. CTRE's high-rate odometry thread already gives timestamped samples; keep a ~1.5 s `TimeInterpolatableBuffer` of them. [code]
2. **Make the goal and the correction come from the same tag (2910 / 604).** Then map error cancels and no per-event tuning is needed for the final few cm. [code]
3. **Lock to the target tag at command start** and pick that tag from a velocity-projected pose (604, 0.15 s). [code]
4. **Tolerances in the target frame** (604: 1.5 in depth / 1.0 in lateral; 971: normal/tangent; 1678: separate X/Y), plus a short debounce of 40–50 ms. [code]
5. **Approach along the target normal.** Hold the goal back (971: 0.2 m if >5 cm off-line; 2910: 6 in until heading is within 5°; 3476: intermediate pose). [code]
6. **Gate on the yaw-rate peak over the lookback window, not the current yaw rate.** Feed vision to the estimator **in timestamp order**: WPILib's estimator deletes newer updates when you add an older one. This matters with two Limelights. [code]
7. **Gyro owns heading.** Use a huge heading std-dev while enabled and seed heading while disabled (Spectrum, 1678, 3476). [code]
8. **Shadow tracks plus pre-gate logging of raw frames** (Spectrum SystemCore). This lets you tune the gates with AdvantageKit replay. [doc/code]
9. **Synced mechanisms.** Fork "raise elevator" when the robot is within distance *and* under a speed (1678). This maps directly to Commands v3 `fork`/`await`. [code]
10. **Cheap physical backstop** where geometry allows: drive into the element or use a beam-break strafe (3476, 1678). [code]
11. **Game piece target.** Convert each detection to a field pose and drive to a moving average of the last N, gated by zone and heading. End on the beam break or a timeout (1678). [code]

## 5. Pitfalls

- **No latency compensation hides in "working" code.** 2910's latency constant is 0.0; 971's 2025 localizers ignored capture time. Either replay odometry or reset only when the robot is nearly stopped. [code]
- **WPILib `addVisionMeasurement` deletes newer stored updates** when it receives an older one. Interleaved cameras silently lose corrections. [code]
- **MegaTag2 trusts your gyro.** Spectrum's sim and 2026 notes found it drifts with gyro error. Prefer MegaTag1 or PnP for seeding, and MegaTag2 only after a confirmed heading. [doc/code, sim only]
- **Frames from distant tags cause jumps at close range.** 1678 cuts at 5.5 m, 971 at 4.5 m; CD teams saw circling. [code / unverified]
- **Image-space tx/ta loops lose the tag up close.** Multiple CD teams moved off them. [unverified]
- **Field tolerances are worse than the manual suggests.** Per FIRST, event variation exceeds about 1–3 in; trusses were seen rotated up to 8°. Don't rely on tag-to-tag consistency across field elements. [doc / unverified]
- **Copy-paste velocity checks.** 1678's `hypot(vx, vx)` never looks at vy. [code]
- **Timeouts and fallbacks are required.** 971 auto 2.5 s, 1678 detection 3.5 s, 3476 sensor error states, 1678 rumble when no tag is seen. [code]
- **2027 / SystemCore specifics** (Spectrum): the NI DS controller index mapping differs from WPILib 2027 `Gamepad`. A main thread at normal priority suffered 11–46 ms worst-case loops; real-time priority 1 fixed it. MapleSim has no 2027 build. [doc]
