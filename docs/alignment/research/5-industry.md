> Raw research notes gathered by AI research agents on 2026-10-11 for [the alignment decision doc](../README.md). Claims are tagged [code] (read in source), [doc] (talk/post/docs, often via search snippets — Chief Delphi was not reachable), or [unverified]. Check anything important at the source before relying on it.

# Global-to-local final alignment: how professional robotics solves it, and lessons for an FRC swerve robot

**Bottom line.** The team's plan is the standard industry pattern: a global pose to get close, then a pose relative to the target for the last stretch, with latency covered by odometry alone. Nav2 docking, Fetch, Boston Dynamics Spot, ArduPilot and PX4 all work this way. Three upgrades matter most:

1. Store the observed target as a pose in the **odometry frame** and servo to it there.
2. **Teach** the goal as the camera's own view of the real target, so calibration errors cancel.
3. Add an **abort-and-retry gate** and a **mechanical funnel**.

**Main risk in the plan:** a local pose is *not* free of calibration error. A 1° error in the camera's mounted yaw shifts the result sideways by 1.7 cm at 1 m. That alone uses up the whole 1–2 cm budget.

---

## 1. Visual servoing (PBVS vs IBVS vs 2.5D)

**Key idea.** There are three classic ways to steer a robot from a camera:

- **PBVS** (position-based) estimates the target's 3D pose and drives the pose error to zero. The robot's path is a straight line, but the target can leave the camera's view. Any calibration error ends up in the final pose.
- **IBVS** (image-based) drives the image-feature error *s − s\** to zero. Calibration errors only bend the path or change the convergence speed, because the loop forces the *measured* features to match the *desired* ones. Its weaknesses are only local stability, local minima, singular interaction matrices, and "camera retreat" when a large rotation about the optical axis is needed.
- **2.5D / hybrid** (Malis, Chaumette & Boudet 1999) combines an image point with the 3D rotation. The motion is decoupled, it converges from a wider range of starts, and its stability under calibration error can be proven.

Targets leaving the view are handled with switching or hybrid schemes, or by planning the path in the image (Mezouar & Chaumette 2002).

**The nuance that matters.** IBVS is robust because *s\** is **taught**: it is recorded by the same camera at the real goal ("teach by showing", Hutchinson, Hager & Corke 1996). If *s\** is instead computed from a model pose, the calibration errors come back. So the robustness comes from teaching, not from controlling in image space.

**Sources:** Chaumette & Hutchinson, *Visual Servo Control I/II*, IEEE RAM 2006/2007 (doi:10.1109/MRA.2006.250573); Malis et al., IEEE T-RA 15(2), 1999.

**FRC recommendation: "taught-pose PBVS."**

- Put the robot at the perfect scoring spot against a real element. For each camera, record the tag pose the camera reports (camera→tag).
- In the final approach, servo the *measured* camera→tag pose toward the *taught* one.
- Any constant error cancels when the robot arrives: focal length, camera extrinsics, even where the tag is mounted on the element. This works with ordinary pose-space control; interaction matrices are not needed.
- Swerve can rotate and translate independently. Use that like 2.5D: aim the heading to keep the tag in view, and let translation do the rest.
- Re-teach after any camera bump; keep resolution and decimation unchanged.

## 2. Mobile-robot docking with fiducials

**Nav2 `opennav_docking`** (from the source code: `docking_server.cpp`, `simple_charging_dock.cpp`, `pose_filter.cpp`):

- **Staging pose:** 0.7 m in front of the dock (`staging_x_offset −0.7`). The drive to it is skipped if the robot is already within `dock_prestaging_tolerance` 0.5.
- **Initial estimate:** the dock pose from the database, transformed into `fixed_frame`. That frame defaults to **`odom`**, and the docs say it is *"recommended to be a smooth odometry frame **not** map."*
- **Each detection** (a `PoseStamped` from an AprilTag detector) is transformed into odom using TF **at the detection's own timestamp**. This is exactly the team's latency-compensation idea.
- **Filter:** an exponential moving average on position (`filter_coef 0.1`) plus a slerp on orientation. It resets if the gap between detections is longer than the timeout.
- **Controller:** a "graceful" smooth control law for differential-drive robots (Park & Kuipers 2011), at 0.10–0.25 m/s and 50 Hz. The target is projected 0.25 m *past* the dock "so the robot never gets to the end of the spiral," which avoids the math blowing up near the goal.
- **Dropouts:** a detection older than `external_detection_timeout` (1.0 s) raises `FailedToDetectDock`. The robot then retries from the staging pose, up to `max_retries` 3.
- **`rotate_to_dock`:** detect the dock facing forward, turn 180°, and back in **blind** on the stored pose in the odom frame.
- **Done when:** within `docking_threshold` (0.05 m), or when contact or a motor stall is detected.

**Fetch** (`fetch_open_auto_dock`):

- Matches the dock's shape in the lidar scan (ICP), tracking it in `tracking_frame_ = "odom"`, with a 2nd-order Butterworth filter on the dock pose.
- Treats a detection as stale after 0.35 s, and works best when starting 0.75–1.0 m away and facing the dock.
- **Abort gate:** inside 0.40 m, if the sideways error is over 2.5 cm or the yaw error is over 5°, it backs up and retries (up to 5 times).

**Spot:**

- Finds the dock with a 500-series AprilTag. **Two indexing cones** on the dock then mechanically locate the connector.
- Uses a "prep pose" before docking. `STATUS_MISALIGNED` aborts *"to save ending up in an unrecoverable state."*
- `filtered_fiducial_XXX` frames are valid *"only for fiducials which are static in the world."*
- Each tag detection carries a covariance and a status flag: `STATUS_AMBIGUOUS` or `STATUS_HIGH_ERROR`.

**Other fleets:** **MiR** docks to lidar VL/L markers at **±2–3 mm, ±0.3°** (vs ±10–18 mm for bar markers). **Kiva/Amazon** reads floor fiducials with a downward camera, dead-reckoning between them. **Pallet AGVs** detect pockets at 1.5–3 m; error is mostly along the approach (depth) axis and shrinks when close (ADAPT, arXiv 2503.14331); fork lead-ins absorb the rest.

**FRC recommendation.** Copy Nav2's structure almost exactly:

- Use a staging pose 0.6–1.0 m out, squared up to the element.
- Start with the target's map pose expressed in odom, and refine it with each detection, transformed at its capture time.
- Filter with an EMA (α≈0.1–0.3 at 30–100 fps), add an outlier gate, and coast through gaps of 0.3–0.5 s.
- Add an abort gate. For example, inside 0.4 m, if the sideways error is over 3 cm or the yaw error is over 4°, back out to staging and retry.
- Swerve is holonomic, so use three independent profiled P controllers in the target frame. The diff-drive graceful law is not needed.

## 3. Drone precision landing

**ArduPilot `AC_PrecLand`:**

- Estimates the target's position **relative to the vehicle**, in a north-aligned frame. It never estimates the target's absolute position.
- An **inertial history ring buffer** spans `PLND_LAG` (0.02–0.25 s). The estimator runs at that *delayed* time and rotates the line of sight using the **attitude at image time**. Then `run_output_prediction()` integrates forward through the buffer to the present.
- **KF option:** one 2-state (position, velocity) filter per axis, predicted with IMU delta-velocity. Measurements pass a **NIS < 3** gate, but one is accepted anyway **after 3 rejections in a row**, so a real jump does not lock the filter out.
- **Target lost:** keep descending for `PLND_TIMEOUT`, then retry (`PLND_RET_MAX`).

**PX4:**

- `landing_target_estimator` uses one KF per axis "as simple low-pass filters that also produce a velocity estimate and allow for outlier rejection."
- The newer **Vision Target Estimator** has the state *[relative position r, vehicle velocity, bias b]*. Here *b* is the offset between the absolute reference (GNSS or the mission waypoint) and the target as seen by vision.
- It fuses delayed vision through **OOSM** (out-of-sequence measurement) updates against a state history.
- Once *b* is known, the absolute reference *"effectively becomes a second relative-position sensor."* That lets the landing survive the marker leaving the view in the final metres.
- Without velocity aiding, "vision dropouts cause a visible relative-position drift that snaps back when vision returns."

**FRC recommendation.** Adopt PX4's bias idea per field element: *b = observed − map*. Use it to:

- coast through dropouts;
- **remember** it for the next cycle to the same element, so the approach starts already corrected;
- sanity-check detections (if |b| > ~10 cm, suspect the wrong tag or an ambiguity flip).

Gate measurements the way ArduPilot does.

## 4. State estimation with delayed measurements

**Frames (REP-105).**

- `odom` is continuous, *"an accurate, short-term local reference."*
- `map` makes *"discrete jumps… a poor reference frame for local sensing and acting."*

**`robot_localization`** runs an odom-EKF on continuous sensors only. A separate map-EKF takes the global data and publishes map→odom. `smooth_lagged_data` with `history_length` rewinds and replays when lagged data arrives. Bar-Shalom (IEEE TAES 2002) gives the exact out-of-sequence update.

**WPILib's `PoseEstimator`** already follows this split. It keeps a 1.5 s buffer of raw odometry and applies vision corrections at the measurement timestamp, which is map→odom under another name.

**Is "track the target in odom and servo there" standard?** **Yes.** Nav2 uses `fixed_frame=odom`, Fetch uses `tracking_frame=odom`, Spot filters fiducials in its odom/vision frames, and ArduPilot and PX4 track the target relative to the vehicle in the local NED frame.

**FRC recommendation.**

- Keep a separate **odometry-only** pose buffer: a `SwerveDriveOdometry` fed with the 250 Hz module positions and gyro, stored in a `TimeInterpolatableBuffer` about 1 s long.
- For each detection, compute `T_odom_target = T_odom_robot(t_capture) · T_robot_cam · T_cam_tag`.
- Every 50 Hz loop, compute the error as `T_odom_robot(now)⁻¹ · T_odom_target`.

This equals the "twist since capture" approach, but frames average naturally and dropouts cost nothing. Do **not** use CTRE's fused pose (it contains vision jumps). OOSM replay is unnecessary because nothing corrects the odom frame.

## 5. AprilTag accuracy

**Ambiguity.**

- A planar 4-point pose solve has two minima (Schweighofer & Pinz, PAMI 2006; IPPE, Collins & Bartoli, IJCV 2014). It is worst when the tag is small, far away, or seen head-on.
- The AprilTag library's `estimate_tag_pose_orthogonal_iteration` returns **both** solutions and their errors.
- Ways to resolve it:
  - reject the detection when the error ratio is high (PhotonVision's "ambiguity", typical threshold 0.2);
  - pick the solution that agrees with the gyro heading (as Limelight MegaTag2 does);
  - use rigid multi-tag bundles;
  - use several views (Ch'ng et al., arXiv 1909.11888).
- Seeding the solve from the last pose can lock onto the wrong solution (arXiv 2312.08488).

**Error vs distance.** These are first-order geometry estimates for f≈760 px (1280×800, 80° horizontal FOV, as on the LL3G), 0.3 px corner noise, and a 165 mm tag:

| Range | Lateral σ | Depth σ | Single-tag yaw σ |
|---|---|---|---|
| 1 m | 0.4 mm | 2.4 mm | ~1° |
| 3 m | 1.2 mm | 21 mm | ~10° |

- Depth error grows with the square of the range (∝Z²/(f·s)), so it is about Z/s times the lateral error.
- Near head-on, the yaw cue is second-order. That is the same cause as the ambiguity.
- **Biases dwarf this noise.** A 1% focal-length error gives 1 cm of depth error at 1 m. A 1° extrinsic yaw error gives **1.7 cm** sideways at 1 m and 5.2 cm at 3 m.
- Abbas et al. (Sensors 2019): camera yaw relative to the tag is the main error source. Kalaitzakis et al. (JINT 2021) benchmark bundles and blur.

**Detector settings.**

- `apriltag.h` says `quad_decimate` costs pose accuracy. Use **decimate 1** close in, and keep `refine_edges` on.
- Tag size is measured at the black/white border.
- Solve **one** pose over all the corners of every tag on the same rigid element.
- Two cameras that see the same tag form a stereo pair. With a 0.4 m baseline, depth from two bearings is about 2.5× better than depth from the size of one 0.165 m tag.

**Shutter and blur.**

- Use a global shutter (the OV9281 in LL3G/LL4), with exposure of 1–2 ms or less and gain to make up the brightness.
- Blur in pixels ≈ f·(v/Z + ω)·t_exp. Rotation dominates: at 2 rad/s and 2 ms that is 3 px.
- Timestamp each frame at **mid-exposure**: start of exposure + t_exp/2, plus about half the readout time for a rolling shutter.

**Extrinsics.**

- The usual method is hand-eye calibration (AX=XB: Tsai-Lenz, or OpenCV `calibrateHandEye`).
- On a ground robot moving in a plane, the camera's height **cannot be recovered from motion** (Guo, Mirzaei & Roumeliotis 2012), so measure it directly.
- Best for FRC: calibrate against the target. Place the robot at several jig-known poses relative to a tag and solve by least squares. Or skip that and teach the view (§1).

## 6. Sensor time sync

**Professional practice:** hardware triggering with mid-exposure timestamps, and PTP (IEEE 1588) between computers. Leftover offsets are estimated from motion: Kalibr does it offline, and Qin & Shen (IROS 2018) showed the apparent camera-IMU offset changes with exposure time.

**FRC equivalent:** NT4 time sync, and PhotonVision's UDP TimeSync (Cristian's algorithm, a lightweight cousin of PTP). PhotonVision's **LED-blink test** works like this: a roboRIO DIO pin drives an LED, and a camera watches it. Using V4L start-of-exposure timestamps cut the error from **26 ms to 11 ms**, which is inside one 60 fps frame.

**Error budget:** e ≈ v·δt + ω·R·δt.

- A 20 ms timing error at 1 m/s gives 2 cm.
- At 2 rad/s with the tag 1.5 m away, it gives **6 cm**.

**FRC recommendation.**

- Run the LED test on each camera.
- Then run a **spin test**: rotate in place while a tag is visible, and log the target's pose in the odom frame. If that pose swings in step with ω, the latency is wrong. If it traces a circle, the extrinsics are wrong.
- Near the target, cap ω at about 0.5 rad/s and v at about 0.5 m/s, so a leftover 10 ms error costs less than 5 mm.

## 7. Game-piece relative alignment

**Ground-plane projection** from the bottom-centre of the bounding box: d = h/tan θ, so ∂d/∂θ = (h²+d²)/h.

- With the camera at h = 0.5 m, a 1° pitch error gives **15 cm** of range error at 2 m and 4.4 cm at 1 m.
- Bearing is much better: 1° gives 3.5 cm sideways at 2 m.

**Standard practice:** track each piece **in odom** with a constant-position KF (SORT, Bewley 2016, uses constant velocity + Hungarian matching on IoU). Gate association by distance (~0.3 m); confirm after N hits, drop after M misses.

**FRC recommendation:** steer on bearing (IBVS-style centring), and let the intake's width absorb the depth error. When the piece disappears under the bumper, commit to its last position in the odom frame.

## 8. Industrial "last-centimeter" practice

**Coarse-to-fine with a sensor handoff:** global navigation → staging → fiducial servo → contact or short-range sensor. Final precision lives in the mechanics (Spot's indexing cones, remote-centre compliance and chamfers, fork lead-ins). Success is *confirmed* by contact (Nav2 stall/effort, Fetch charger voltage); laser/ToF sensors handle depth.

**FRC recommendation.**

- Build a funnel or lead-in that accepts **±3–5 cm and ±3°**. Then vision only has to land inside the funnel.
- Add a ToF sensor (CANrange or LaserCAN) for **depth** to the element's face. It complements the camera, which is strong sideways and weak in depth. Two ToF sensors also give yaw to a flat face.
- Confirm the result with a beam break or a current spike.

---

## Flags for the team's plan

- **Contradicts the plan:** a local pose is not calibration-free (teach or calibrate against the target); don't discard the map (use it as prior + bias); single-tag yaw at range is unreliable (take yaw from the gyro).
- **Probably overthinking:** full IBVS with interaction matrices, nonholonomic graceful controllers, full OOSM/factor-graph replay. Nav2 and Fetch ship an odom-frame target + EMA + gate.
- **Missing:** abort/retry gate, dropout coasting, timestamp validation, rotation-rate limit near the target, ambiguity rejection, per-element offset memory, decimate 1 close in, a mechanical funnel.

## Top 8 transferable lessons

1. **Servo in odom, not map.** Build the target's odom-frame pose at capture time, then servo to it (REP-105, Nav2, Fetch, Spot, ArduPilot).
2. **Teach the goal with the real sensor.** Recording the view at the true target cancels intrinsic, extrinsic and tag-mounting bias. That is where IBVS's robustness comes from.
3. **The map is a prior, not the truth.** Seed the target from the map, refine it by detection, and remember each element's offset (the PX4 VTE bias).
4. **Stage, then approach.** Square up 0.6–1 m out, where detection is good and errors are still correctable (Nav2 0.7 m; Fetch 0.75–1.0 m).
5. **Gate and retry.** Abort inside about 0.4 m if the robot is outside the approach corridor (Fetch 2.5 cm/5°; Spot `MISALIGNED`; Nav2 `max_retries`).
6. **Plan for blindness at the end.** Coast on the odom-frame target (Nav2 `rotate_to_dock`, PX4 timeouts, ArduPilot `PLND_TIMEOUT`).
7. **Respect the error geometry.** Cameras are good sideways but weak in depth (∝Z²) and in single-tag yaw. Use the gyro for yaw, ToF or contact for depth, multi-tag or two cameras for both, and finish close.
8. **Timing and mechanics win the last centimetre.** Use mid-exposure timestamps checked by LED and spin tests, a low ω near the goal, a global shutter with short exposure, and a funnel that forgives ±3–5 cm.

### Key links

- Nav2 docking: https://github.com/ros-navigation/navigation2/tree/main/nav2_docking
- Fetch auto-dock: https://github.com/fetchrobotics/fetch_open_auto_dock
- Spot docking: https://github.com/boston-dynamics/spot-sdk/blob/master/docs/concepts/autonomy/docking.md
- ArduPilot precision landing: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_PrecLand/AC_PrecLand.cpp
- PX4 Vision Target Estimator: https://github.com/PX4/PX4-Autopilot/blob/main/docs/en/advanced_features/vision_target_estimator.md
- REP-105: https://www.ros.org/reps/rep-0105.html
- robot_localization: https://github.com/cra-ros-pkg/robot_localization/blob/ros2/doc/state_estimation_nodes.rst
- WPILib PoseEstimator: https://github.com/wpilibsuite/allwpilib/blob/main/wpimath/src/main/java/org/wpilib/math/estimator/PoseEstimator.java
- AprilTag library: https://github.com/AprilRobotics/apriltag
- AprilTag 3 paper: https://april.eecs.umich.edu/media/pdfs/krogius2019iros.pdf
- Abbas et al. 2019: https://pmc.ncbi.nlm.nih.gov/articles/PMC6960891
- Kalaitzakis et al. 2021: https://link.springer.com/doi/10.1007/s10846-020-01307-9
- Ch'ng et al. (pose ambiguity): https://arxiv.org/abs/1909.11888
- PhotonVision latency test: https://docs.photonvision.org/en/latest/docs/contributing/design-descriptions/e2e-latency.html
- Qin & Shen 2018: https://arxiv.org/abs/1808.00692
- SORT: https://arxiv.org/abs/1602.00763
- MiR250 spec sheet: https://www.hartfiel.com/wp-content/uploads/2024/04/MiR250-Specs.pdf
- ADAPT (pallet detection): https://arxiv.org/pdf/2503.14331
