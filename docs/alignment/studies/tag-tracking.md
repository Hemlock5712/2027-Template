# Study: tracking the target tag for the final approach

**Status:** simulation only, Oct 2026. Untested on a robot. Code:
[`tools/alignsim/studies/tag/`](../../../tools/alignsim/studies/tag/). Full tables:
[`ablation_results.md`](../../../tools/alignsim/studies/tag/ablation_results.md),
[`bias_results.md`](../../../tools/alignsim/studies/tag/bias_results.md),
[`conditions_results.md`](../../../tools/alignsim/studies/tag/conditions_results.md),
[`final_results.md`](../../../tools/alignsim/studies/tag/final_results.md).

**Question.** For the last 1–3 m to a scoring spot, how should the robot measure where the target is,
so it finishes within 2 cm and 1° of the *real* element, not the one on the field map?

## Bottom line

1. **Measure the target, don't trust the map.** Every approach that drives to the map's goal fails as
   soon as the element is off the map. Tracking the tag in the wheel-odometry frame doesn't care.

   | approach (success % / false "done" %) | default | hard | very hard |
   |---|---|---|---|
   | global pose, all tags | 57 / 43 | 31 / 69 | 15 / 85 |
   | global pose, target tag only (581, 254) | 57 / 43 | 31 / 69 | 16 / 84 |
   | odom frame, map's tag yaw (our port of 6328 txTy) | 56 / 44 | 31 / 69 | 15 / 85 |
   | raw tag pose every loop (the old `DriveToTag`) | 10 / 0 | 9 / 10 | 6 / 24 |
   | fused-pose history (2026 `samplePoseAt` style) | 100 / 0 | 71 / 29 | 45 / 55 |
   | odom frame, measured yaw (first `TagTracker.java`) | 100 / 0 | 72 / 28 | 46 / 54 |
   | **+ per-camera calibration + retuned controller (ships)** | **100 / 0** | **99 / 1** | **98 / 2** |

   The raw tag pose oscillates and finishes only 10–30% of runs. The worlds are described in
   [Setup](#setup).
2. **Once the field error is gone, the camera is the error.** One error source at a time (ablation,
   n = 60), with the odom-frame tracker:
   - element 4 cm / 2° off: 100% (global, target tag only: 38%);
   - global pose 8 cm off: 100% (global: 63%);
   - wheel radius 2%, focal length 1%, principal point 4 px, camera position 5 mm, clock 10 ms: 100%;
   - **camera mount yaw 0.8°: 92–95%;**
   - **pose-solve yaw bias 1°: 78–83%.**

   A 1° yaw error at the camera swings the measured tag sideways by 1.7 cm per metre, and it does so
   the same way every frame, so no filter can average it out.
3. **Calibrate each camera once, not each goal.** Park the robot exactly on a real scoring spot, back
   it straight away 0.6 m, and for each camera find the small robot-frame correction `A` with
   `A ⊕ measured = expected` (hard world, n = 80):

   | calibration | default | hard |
   |---|---|---|
   | none | 100 | 66 |
   | taught goal (old Teach Tag Goal) | 91 | 64 |
   | **per-camera correction** | **100** | **100** |

   - **Why not teach the goal?** At the scoring spot the tag is out of view (the last ~5 cm), so a taught
     goal has to be measured while backing off and mixes both cameras' errors at short range. The
     per-camera correction fixes each camera where its error comes from.
   - **Why not the map?** The correction is a property of the camera, so it carries to every tag.
4. **The filter barely matters once cameras are calibrated.** A geometry-aware filter was built (noise
   long along the camera ray, short across it; yaw noise growing with ambiguity; a map prior; a yaw
   gate for flipped solves). With calibration, every filter variant landed at 95–100% in every world.
   The simplest one, the first `TagTracker.java` (per-axis Kalman, frames with ambiguity > 0.3
   dropped), scored 100 / 99 / 98%, so it ships. The geo filter stays in the replay tool to compare on
   real logs.
5. **Use two cameras at the front corners.** One center camera gets 73–78%: it sees the tag head-on,
   where the single-tag yaw solve is ambiguous. One corner camera already gets 98–100%. Toe-in angle
   (0–25°) doesn't matter.
6. **These barely matter** (all 98–100%):
   - frame rate 20–120 fps;
   - pixel noise 0.2–1.5 px;
   - handoff speed 0–3 m/s;
   - 100 Hz odometry (SystemCore without a CANivore);
   - the done gate (1 cm vs 2 cm). The controller parks within 3 mm of where it *thinks* the goal is,
     so the remaining misses are estimate errors.
7. **Approach shaping is optional.** A staging point on the tag's normal, or 6328's sliding target,
   gets 100% vs 99% for driving straight in, and costs 0.10–0.16 s. Use it when the element blocks a
   diagonal approach.
8. **Timestamps are forgiving on a straight approach.** No latency compensation at all moves the median
   from 0.4 to 0.8 cm and keeps 98–100%. Timestamps 100 ms too *early* cost 7%. The last frames come in
   while the robot is slow, so timing error mostly lands in depth. Turning while viewing is where timing
   matters more, which is why the spin test is in the robot test plan.
9. **The retuned `DriveToPose` finishes 0.28 s sooner** (1.58 s vs 1.86 s) with zero wheel slip and the
   same accuracy. See [controllers.md](controllers.md).

## Setup

The simulator is [`tools/alignsim`](../../../tools/alignsim/README.md):
- swerve physics at 500 Hz with wheel slip;
- CTRE's 250 Hz odometry thread and a faithful port of WPILib's `PoseEstimator`;
- Limelight-like cameras. Each one projects tag corners with the *true* mount and lens, adds pixel noise
  and motion blur, then solves with the *believed* mount and lens (OpenCV IPPE, with ambiguity);
- a reef-like element whose true pose differs from the map, with neighbouring faces and other field tags.

A trial starts 2.8 m back along the tag's normal, up to ±0.8 m sideways and ±25° of heading, already
moving, and hands off to the local tracker. It **succeeds** when the robot declares itself done *and*
is truly within 2 cm and 1°. A **false done** is the dangerous case: it declared done but is outside.

Errors are drawn per trial (1σ):

| | default | hard | very hard |
|---|---|---|---|
| element vs map | 2.5 cm / 1° | 4 cm / 2° | 8 cm / 4° |
| camera mount yaw / pitch | 0.4° / 0.4° | 0.8° / 0.8° | 1.5° / 1.0° |
| pose-solve yaw bias | 0 | 1° | 1.5° |
| focal length / principal point | 0.5% / 2 px | same | 1% / 4 px |
| camera clock bias | 2 ms | 5 ms | 15 ms |
| global pose error at handoff | 3 cm | 8 cm | 15 cm |
| handoff speed | 1.5 m/s | 1.5 m/s | 2.5 m/s |

**These sizes are guesses.** The robot test plan measures the real ones, and the replay tool re-runs
the trackers on real logs.

## How the calibration works

[`CalibrateCamerasOpMode`](../../../src/main/java/frc/robot/opmodes/CalibrateCamerasOpMode.java), and
`CalWorld.teach` in [`bias.py`](../../../tools/alignsim/studies/tag/bias.py):

1. Put the robot exactly on the scoring spot, with a jig. Where the tag *should* be is then known:
   `tag = parked ⊕ tagToGoal⁻¹`, using the CAD `tagToGoal`.
2. The robot backs straight away at 0.15 m/s for 4 s. For each frame, the tag *should* appear at
   `expected = robotAtCapture⁻¹ ⊕ tag`.
3. Each frame gives a sample `A = expected ⊕ measured⁻¹`. The OpMode averages them per camera and prints
   one `Transform2d` per camera for `CameraCorrections`.
4. `TagTracker` applies `A ⊕ measured` to every frame from that camera.

**What it absorbs:**
- the camera's mounting error (exactly, for yaw and position);
- the pose solve's yaw bias at the calibration distances;
- small lens errors.

**What it also absorbs, and shouldn't:** how far the robot was from the true scoring spot, and the
calibration tag's own mounting error on its element. So park with a jig, calibrate on a well-built
element, and re-run whenever a camera is bumped.

## Caveats

- **Simulation only.** The pose-solve yaw bias is constant here. On a real Limelight it may change with
  distance and viewing angle, which a single correction can't fully fix. Test plan step 3 measures that.
- **One element, straight-ish approaches.** Starts are within ±0.8 m of the normal, 2.8 m out. Steep side
  approaches and blocked views weren't swept.
- **Collisions near the target are the open risk:** 32% false done after a hard push. See
  [skid-odometry.md](skid-odometry.md).
- **n = 60–100 per row** gives about ±2–6 points on success %.

## Reproduce

From `tools/`:

```bash
python3 alignsim/studies/tag/ablation.py 60 2        # one error at a time
python3 alignsim/studies/tag/bias.py 80 2            # calibration: none / taught goal / per camera
python3 alignsim/studies/tag/conditions.py 60 2      # cameras, fps, noise, timing, odometry rate
python3 alignsim/studies/tag/final.py 100 2          # approach shaping, headline, done gate
python3 alignsim/studies/tag/which_filter.py 100 4   # which filter, once calibrated
```
