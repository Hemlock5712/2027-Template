# Video 3: "Trust the tag, not the map" (driving to a tag: `DriveToTag`, `TagTracker` and camera calibration)

An in-depth lesson in 23 chapters (about 25 minutes of narration, 4,477 words; about
27 with the pauses between lines) on how
the robot stops within 2 cm and 1° of a *real* field element. It walks the series' five steps: the
**goal** (real fields are a few cm off the map, so even a perfect field pose drives to the wrong spot),
the **idea** in plain words (far away use the map, near the target measure its tag; keep the tag in the
smooth wheel world; rewind every photo to when it was taken; blend photos with a smart average; finish
on the wheels; fix each crooked camera once), the **maths** with this robot's real numbers (composing
poses, the photo's age, the per-axis Kalman filter, the jump and lock rules, re-anchoring the goal, the
1° camera error and the calibration fix), the **code line by line**
([`DriveToTag.java`](../../../../src/main/java/frc/robot/commands/DriveToTag.java),
[`TagTracker.java`](../../../../src/main/java/frc/robot/subsystems/vision/TagTracker.java),
[`WheelOdometry.java`](../../../../src/main/java/frc/robot/subsystems/WheelOdometry.java),
[`CameraCorrections.java`](../../../../src/main/java/frc/robot/subsystems/vision/CameraCorrections.java),
[`CalibrateCamerasOpMode.java`](../../../../src/main/java/frc/robot/opmodes/CalibrateCamerasOpMode.java), and a
glance at [`Vision.java`](../../../../src/main/java/frc/robot/subsystems/vision/Vision.java) for contrast),
and a **recap** with check-yourself questions, a "try it" and an honest "what's next". One traced
approach (photos 1–3, one re-anchor, one calibration sample) carries the same numbers from the board
into the code's ghost values. The study behind it: [tag-tracking.md](../../studies/tag-tracking.md).

Each chapter is its own animate piece with its own voice-over file:

```
/animate chapter 1 of the explainer in brief.md, use ch01-voice.json as the voice-over
```

Change the chapter number and file for each one (`chapter 2` … `ch02-voice.json`, up to `ch23`).
The `ch1-voice.json` … `ch6-voice.json` files in this folder belong to the old six-chapter version;
regenerate `chNN-voice.json` from the numbered lists below (see [../README.md](../README.md), "Editing
narration": ids `c01-1`, `c01-2`, …, `after` 0.6, the last line 0.3). In chapter 22, give the question
lines (6, 8, 10, 12 and 14) `"after": 4.0` so the pause card can hold.

**Reading the beat tables.** Each row is one narration line, so animate can time it to the voice. The
Time column is a guide computed at 3 words a second plus the 0.6 s gap (it leaves out the 1.2 s lead).
Inside a row, "→" separates the changes that keep something new on screen every 2–4 s. Mode is the
STYLE.md mode at the end of the row; moving between modes is always a camera move, never a cut.
Numbers in the narration are spelled the way they are written on screen.

## Intake answers (same for every chapter)

| Question | Answer |
| --- | --- |
| **Subject** | How the robot lines up within 2 cm and 1° of a real field element using that element's AprilTag: why the map isn't enough, the two "where am I" answers, rewinding old photos, combining photos with a smart average, re-anchoring the goal, and calibrating each camera once; then the real Java (`DriveToTag`, `TagTracker`, `WheelOdometry`, `CameraCorrections`, `CalibrateCamerasOpMode`) line by line. |
| **The one true thing** | Near the target, measure the target itself: place every photo where the robot was when it was taken, in the smooth wheel world, trust sharp photos more, and fix each camera once. |
| **Must include** | Map vs real element and the 2 cm / 1° goal; field pose vs wheel pose and why the tag lives in the wheel world; the photo's age and the rewind (≈5 cm at 1.5 m/s); the smart average with real numbers (σ, variance, gain, drift); jumps, unsure photos, lock after 3; finishing on the wheels; the re-anchor and why vision jumps cancel; 1° → 1.7 cm per metre and why averaging can't fix it; the calibration drive and `A = expected ⊕ measured⁻¹`; two corner cameras; the simulator results; every code line named in the chapters; collisions as the open risk. |
| **Must avoid** | Claiming robot-tested accuracy: every result is "in our simulator". The words "Kalman filter", "odometry", "ambiguity" and "IMU" before their picture. "PnP", "latency compensation" and "interpolation" in the narration (say "the camera's tag solve", "rewind", "blend"). Re-teaching video 2's driving loop. Calling other teams' code wrong. Red for anything that isn't an error. |
| **Formats and length** | 16:9 first, 9:16 second. 23 chapters of about 1–1.5 min each, about 25 min of narration in all. Chapters can be posted as a series. |
| **Voice** | Narrated voice-over, one JSON per chapter (`ch01-voice.json` … `ch23-voice.json`), generated from the numbered lists. |
| **Look** | `"style": "field-lab"`, the custom style in [../STYLE.md](../STYLE.md): **World** (isometric robot and field on navy), **Board** (the maths, written on), **Code** (editor panel with real line numbers, a highlight bar and ghost values). The STYLE.md colour code holds everywhere: positions and distances green, speed blue, time yellow, headings purple, the tag and vision white with a thin team-colour outline. This video adds: the **field pose** is a solid green robot outline with a small map-pin badge; the **wheel pose** is a dashed green outline with a wheel badge, riding a faint glass grid sheet called "the wheel world"; **σ, variance, gain and the estimate** use the vision colour (white with the team outline), because they describe photos. The AprilTag is a black-and-white square. |
| **Hero** | The series robot: square frame, four swerve modules (wheel + steering ring), bumpers in the team colour with the team number, two cameras on the front corners with small lens LEDs. TODO: team colours and number. |
| **Music** | Composed in code. A "detective" motif (measuring, checking clues) under Parts 1–2; a quieter pulse under the maths and the code, with a soft tick when the highlight moves; the motif resolves on every LOCK; the series theme under the recap. |
| **Story format** | F4 mission: the robot wants to stop on the real scoring spot. Part 2 solves one problem per chapter. Parts 3–4 follow one traced approach (photos 1–3, the re-anchor, one calibration sample) whose numbers appear first on the board and then as ghost values in the code. Part 5 replays the whole approach, then asks the questions. |

## Numbers on screen

| Thing | Value | Source |
| --- | --- | --- |
| Field tolerance | ±½ in (1.3 cm) and 1° on the drawings; FIRST warned events can be worse | arena drawings; FIRST 2025 Team Update 12 (README §1) |
| Scoring target | within 2 cm and 1° of the real spot | the simulator's success rule (README §5) |
| Driving to the map, in our simulator | 57% / 31% / 15% (default / hard / very hard world) | tag-tracking study, bottom line 1 |
| Tag in the wheel world, no calibration | 100% / 72% / 46% | tag-tracking study, bottom line 1 |
| Tag + per-camera calibration + retuned drive loop | 100% / 99% / 98% | tag-tracking study; `final_results.md` "which filter" |
| One error at a time (wheel-world tracker) | element 4 cm / 2° off: 100%; camera mount yaw 0.8°: 92–95%; tag-solve yaw bias 1°: 78–83% | tag-tracking study, bottom line 2 |
| Hard world: no calibration / taught goal / per-camera fix | 66% / 64% / 100% | `bias_results.md` |
| One centre camera vs two corner cameras | 73–78% vs 100% (one corner camera 98–100%) | `conditions_results.md`, camera layout |
| Wheel-pose drift, in our simulator | 0.2 cm over a 4 m approach; 0.02 cm over the last second | skid-odometry study |
| Wheel memory | last 1.5 s, 250 samples a second (a dot every 0.004 s) | `WheelOdometry.java:24`; README §4 |
| Photos used | up to 50 a second per camera (the newest one each loop) | `TagTracker.update()`; README §6 known limits |
| Photo scatter, simulated camera | ≈2 mm in depth at 1 m, ≈2 cm at 3 m; ≈1 mm sideways | alignsim camera model |
| Photo too old / too far / too unsure | 0.25 s / 3.5 m / ambiguity 0.3 | `TagTracker.java:38–40` |
| Jump check | 25 cm from the estimate; 3 jumps in a row restart it | `TagTracker.java:42–43` |
| Locked | 3 good photos | `TagTracker.java:44` |
| Noise (1 σ) | x/y: 0.005 + 0.01·d² m (1.5 / 4.5 / 9.5 cm at 1 / 2 / 3 m); yaw: 0.5° + 1.5°·d (2° / 3.5° / 5°) | `TagTracker.java:47–50` |
| Drift per photo | 2 mm and 0.1° | `TagTracker.java:52–53` |
| Camera turned 1° | tag 1.7 cm sideways per metre; goal 0.6 m out swings 1.05 cm | tan 1° = 0.0175 |
| Goal from the tag (`tagToGoal`) | 0.60 m out of the tag face, facing it (180°) | `TeleopOpMode.java:36–37`, `CalibrateCamerasOpMode.java:46–47` |
| Calibration drive | park with a jig, back off 0.6 m at 0.15 m/s for 4 s, stop, print at 4.5 s, needs ≥ 20 photos | `CalibrateCamerasOpMode.java:48–49, 103–112, 167` |
| Collisions, in our simulator | 32% false "done" after a 300–800 N push near the target; gyro-accelerometer check caught 95% | skid-odometry study |
| **Traced photo 1** | robot at capture (0.97, 0.50, 0°), camera: tag (2.00, 0.07, 179°) → tag (2.97, 0.57, 179°); d 2.001 m; starts the estimate, σ 4.5 cm, count 1 | Python, fact check |
| **Traced photo 2** (the code trace) | now 12.500 s; age 0.015 + 0.008 + 0.012 = 0.035 s → capture 12.465 s; wheel dots 0.9985 m @ 12.464 s and 1.0045 m @ 12.468 s → robot at capture (1.00, 0.50, 0°); camera: tag (2.00, 0.10, 180°), ambiguity 0.1, d 2.0025 m → measured (3.00, 0.60, 180°); noise 0.002034; gain 0.50; estimate (2.985, 0.585, 179.5°), σ 3.2 cm, count 2 | Python, fact check |
| **Traced photo 3** (after a blocked view) | robot at capture (2.00, 0.50, 0°), camera: tag (1.00, 0.10, 180°), d 1.005 m → noise 0.000228; gain 0.82 (0.33 if it came from 2 m); estimate (2.997, 0.597, 179.8°), σ 1.4 cm, count 3: LOCKED | Python, fact check |
| **Traced re-anchor** | wheel pose now (2.04, 0.50, 0°), field pose now (13.34, 4.10, 0°): goal in the wheel world (2.397, 0.599, −0.2°) ≈ (2.40, 0.60, 0°); seen from the robot (0.357, 0.099) ≈ (0.36, 0.10); field goal (13.697, 4.199) ≈ (13.70, 4.20); a +3 cm vision jump moves both pose and goal by 3 cm | Python, fact check |
| **Traced map goal** | example map tag (14.27, 4.17, 180°) → map goal (13.67, 4.17, 0°), 4.0 cm from the measured goal | Python (example field numbers, not a real tag) |
| **Traced calibration** | parked wheel pose (4.00, 1.00, 0°) → tag (4.60, 1.00, 180°); backed off 0.30 m → expected (0.90, 0.00, 180°); camera 25 cm forward, 20 cm left, turned 1° → measured (0.8964, −0.0113, 179.0°); A = (0.0035, −0.0043, 1.00°), the same at 0, 0.3 and 0.6 m | Python, fact check |

**The traced example: ghost values** (chapters 16–21). Photo 2's loop is the main trace; photo 3's loop
and one calibration sample fill in the lines photo 2 can't reach. Values are rounded as shown on screen;
the fact check has the full arithmetic.

| File:line | Quantity | Ghost value |
| --- | --- | --- |
| `TagTracker:104` | `i` | 0 (the camera that took photo 2) |
| `TagTracker:105–109` | `frame.frameIndex` | 4182 (last seen 4181 → new; illustrative) |
| `TagTracker:115–116` | `tagInRobot3d`, `distance` | (2.00, 0.10, …, yaw 180°), 2.0025 |
| `TagTracker:122–124` | `frameAge` | 0.015 + (8 + 12) ÷ 1000 = 0.035 |
| `TagTracker:129` | capture time | 12.500 − 0.035 = 12.465 |
| `WheelOdometry:57` | `getSample(12.465)` | blend of (12.464 s, x 0.9985) and (12.468 s, x 1.0045) → (1.0000, 0.50, 0°) |
| `TagTracker:134–136` | `rawTagInRobot`, `tagInRobot` | (2.00, 0.10, 180°); correction (0, 0, 0°) → unchanged |
| `TagTracker:137–138` | `ambiguity`, measured | 0.1 ≤ 0.3 → `fuse((3.00, 0.60, 180°), 2.0025)` |
| `TagTracker:208–210` | `xyNoise`, `yawNoise`, `noise` | 0.0451; 3.50° (0.0612 rad); {0.002034, 0.002034, 0.003740} |
| `TagTracker:212` | `tagPose` before | (2.97, 0.57, 179°), `variance` {0.002029, 0.002029, 0.003735}, `goodFrames` 1 (set by photo 1's `restart`) |
| `TagTracker:216–219` | `dx`, `dy`, `dYaw`, hypot | 0.030, 0.030, 1.0°; 0.042 ≤ 0.25 |
| `TagTracker:228` | `drift` | {0.000004, 0.000004, 0.000003} |
| `TagTracker:231–233` | variance + drift → `gain` → variance | x, y: 0.002033 → 0.4999 → 0.001017; yaw: 0.003739 → 0.4999 → 0.001870 |
| `TagTracker:235–241` | `tagPose`, `goodFrames` | (2.985, 0.585, 179.5°), 2 |
| `TagTracker:164` / `DriveToTag:70` | `hasLock()`, `goal()` | false → map goal (13.67, 4.17, 0°) |
| `TagTracker:208–210` (photo 3) | noise at 1.005 m | 0.0151; 2.01°; {0.000228, 0.000228, 0.001228} |
| `TagTracker:231–233` (photo 3) | variance + drift → `gain` → variance | x, y: 0.001021 → 0.8174 → 0.000186; yaw: 0.001873 → 0.6040 → 0.000742 |
| `TagTracker:235–240` (photo 3) | `tagPose`, `goodFrames` | (2.997, 0.597, 179.8°), 3 |
| `TagTracker:164` (photo 3) | `hasLock()` | true |
| `TagTracker:185–187` (photo 3) | goal in wheel world, seen from robot, field goal | (2.397, 0.599, −0.2°); wheel pose (2.04, 0.50, 0°) → (0.357, 0.099, −0.2°); field pose (13.34, 4.10, 0°) → (13.697, 4.199, −0.2°) |
| `CalibrateCamerasOpMode:87` | `expectedTag` | (4.00, 1.00, 0°) ⊕ (0.6, 0, 180°) = (4.60, 1.00, 180°) |
| `CalibrateCamerasOpMode:136–141` | `measured`, `robotAtCapture` | (0.8964, −0.0113, 179.0°), (3.70, 1.00, 0°) |
| `CalibrateCamerasOpMode:148–151` | `expected`, `correction` | (0.90, 0.00, 180°), (0.0035, −0.0043, 1.00°) |
| `CalibrateCamerasOpMode:178` | averaged yaw | atan2(Σ sin, Σ cos) = 1.00° |


# Part 1: Goal

## Chapter 01: The map is not the field (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–8 s | World | WIDE: the whole field from high above, navy ground, carpet sheen; the hero robot idles mid-field (modules twitch, lens LEDs pulse). → A thin white blueprint of the field map draws itself over the carpet. | **THE FIELD MAP** | The camera glides toward one scoring element. | Soft pad; pencil scratch as the blueprint draws. |
| 2 | 8–16 s | World | The real element (white slab with a black-and-white tag) sits under its floating blueprint outline. → The slab springs 3 cm sideways and turns 1° away from the outline. | **MAP ≠ FIELD** | Push in on the gap. | Paper rustle; a low "hmm" note. |
| 3 | 16–23 s | World | EXTREME CLOSE-UP: the gap between outline and slab edge; a green ruler snaps in: **3 cm**; a purple protractor: **1°**. → A drawing callout stamps on, "±½ in (1.3 cm), 1°", then a card, "FIRST: events can be worse". | **DRAWINGS: ±½ in, 1° · EVENTS: WORSE** | Pull back to the robot. | Two stamp thuds. |
| 4 | 23–31 s | World | A green 2 cm circle and a purple 1° wedge appear on the real scoring spot. → A thumb silhouette slides in beside the circle for scale. | **GOAL: WITHIN 2 cm AND 1°** | The robot turns toward the element. | A bright "ping". |
| 5 | 31–39 s | World | The robot gets a gold "PERFECT FIELD POSE" halo, drives (blue velocity arrow, springy) to the blueprint's scoring spot and parks exactly on it. → A red arrow spans the 3 cm gap to the real 2 cm circle. | **PERFECT POSE, WRONG GOAL** | Freeze; the red arrow pulses. | Smooth drive, then a dull "miss" thud. |
| 6 | 39–48 s | World | Three small fields side by side (slightly off, more off, very off), 20 robots parking on each; hits turn green, misses red. → Counters run up: **57%**, **31%**, **15%**. | **IN OUR SIMULATOR, DRIVING TO THE MAP: 57% · 31% · 15%** | The tag on the hero's element starts to glow. | Three heavier "miss" thuds. |
| 7 | 48–56 s | World | Back on the hero: a white beam with a team-colour outline runs from each corner camera to the tag. → The goal marker hops off the blueprint spot onto the real spot. | **NEAR THE TARGET: MEASURE THE TAG** | The three fields return. | Lock "click". |
| 8 | 56–62 s | World | The three fields rerun with the new method; counters: **100%**, **99%**, **98%**. | **MEASURING THE TAG: 100% · 99% · 98%** | The counters fold into five stepping stones. | Three rising chimes. |
| 9 | 62–70 s | Board | The camera tilts up off the carpet: five stepping stones on the navy ground, labelled GOAL, IDEA, MATHS, CODE, RECAP; GOAL is lit. → IDEA lights. | **GOAL → IDEA → MATHS → CODE → RECAP** | Tilt back down to the field (chapter 02). | Five soft steps; the detective motif starts. |

**Narration (ch01-voice.json):**

1. Every FRC field comes with a map. It says exactly where each field element, and each AprilTag on it, should be.
2. But real fields are built by people, at every event. The real element usually sits a little off its spot on the map.
3. The field drawings allow half an inch and one degree, and FIRST has warned that real events can be worse.
4. Our robot's job: stop on the real scoring spot, within two centimeters and one degree. About the width of your thumb.
5. Here's the catch. Even a robot that knows exactly where it is would drive to where the element should be, not where it is.
6. In our simulator, driving to the map spot hits the target fifty seven percent of the time. On messier fields, thirty one, then fifteen.
7. So near the target, our robot stops trusting the map. It measures the target itself, using the AprilTag stuck on it.
8. With everything in this video switched on, the same simulator scores one hundred, ninety nine, and ninety eight percent.
9. We'll go in five steps: the goal, the idea in plain words, the maths with real numbers, the Java code, and a recap.

# Part 2: The idea

## Chapter 02: Far: the map. Near: the tag. (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–6 s | World | WIDE: the robot 6 m from the element; a dashed green path runs to a goal marker with a small "MAP" badge. → The robot springs off along it (blue arrow). | **FAR: DRIVE TO THE MAP** | The camera follows the robot. | Drive hum. |
| 2 | 6–13 s | World | A small inset card: a car parking, its driver's eyes on the curb, the phone map dimmed. → The inset folds away. | **TO PARK, LOOK AT THE CURB** | Back to the robot on its path. | Car door "thunk". |
| 3 | 13–20 s | World | At about 2.5 m, both corner cameras' view cones (faint white) sweep over the element; the tag lights up white with the team outline. | **CAMERAS FIND THE TAG** | Push in on the tag. | Two shutter clicks. |
| 4 | 20–28 s | World | EXTREME CLOSE-UP: the tag's top-left corner, single pixels of the black square; a corner dot snaps onto it. → Pull back: a white arrow from the robot's centre to the tag, labelled "tag in robot: (2.00, 0.10, 180°)". | **CAMERA: "TAG 2.00 m AHEAD, 0.10 m LEFT, FACING ME"** | The arrow stays. | Tiny pixel blips. |
| 5 | 28–36 s | World | From the tag, a green arrow runs 0.60 m straight out of its face; a ghost robot appears at the tip, turned to face the tag (purple 180° arc). | **GOAL = 0.60 m OUT OF THE TAG, FACING IT** | The two arrows chain head to tail. | Two-note "link". |
| 6 | 36–42 s | World | A counter beside the tag ticks 1, 2, 3 as photos agree; a padlock closes. | **SWITCH ONLY WHEN SURE: 3 PHOTOS AGREE** | The chain glows. | Three ticks, lock click. |
| 7 | 42–49 s | World | The chain robot → tag → goal glows; the blueprint outline fades; the goal marker hops 4 cm from the map spot to the chain's tip. | **THE MAP'S MISTAKE DROPS OUT** | A thought bubble rises from the robot (chapter 03). | Motif resolves briefly. |

**Narration (ch02-voice.json):**

1. Think about driving to a friend's house. The map app gets you to the right street, fast.
2. But to park, you stop looking at the map and look at the curb. You measure the real thing.
3. Our robot does the same. Far away, it drives to the map's spot. Close up, its cameras find the tag.
4. A camera reports the tag relative to the robot. Something like: two meters ahead, ten centimeters left, and facing me.
5. The scoring spot is measured from the tag too: zero point six meters straight out of its face, turned to face it.
6. The robot only switches when it's sure: three photos have to agree first. We'll see how soon.
7. Chain those two arrows, and the robot knows where to stop, measured from the real element. The map's mistake drops out.

## Chapter 03: Two answers to "where am I?" (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–6 s | World | A thought bubble opens over the robot; inside, two ghost robots: a solid green outline with a map-pin badge ("FIELD POSE") and a dashed green outline with a wheel badge ("WHEEL POSE"). | **TWO ANSWERS TO "WHERE AM I?"** | The solid ghost steps out onto the field. | Two different blips. |
| 2 | 6–14 s | World | WIDE: the solid ghost drives with the robot; each time a camera sees any field tag (tags around the walls flash), the ghost snaps a few cm: "+2 cm", "−1 cm". | **FIELD POSE: RIGHT, BUT JUMPY** | The dashed ghost glides in beside it. | A small "snap" per correction. |
| 3 | 14–22 s | World | The dashed ghost glides, smooth; under it a faint glass grid sheet (the wheel world) rides along and creeps very slowly (exaggerated). → Inset: a hiker in the dark counting steps. | **WHEEL POSE: SMOOTH, DRIFTS SLOWLY** | Push in on a wheel. | Soft glide; quiet footsteps. |
| 4 | 22–28 s | World | EXTREME CLOSE-UP: a tire contact patch rolling; a tiny green counter ticks distance. → Readout: "drift over a 4 m approach: 0.2 cm". | **IN OUR SIMULATOR: 0.2 cm OVER 4 m** | Pull back to the bubble. | Fine ticking. |
| 5 | 28–33 s | World | The label morphs: "WHEEL POSE" → "WHEEL POSE = ODOMETRY"; the glass sheet gets the caption "odometry frame". | **ODOMETRY = THE WHEEL POSE** | The tag is drawn from the solid ghost. | Single chime. |
| 6 | 33–40 s | World | The tag drawn from the solid (jumpy) ghost: each snap of the field pose drags the drawn tag; red jitter trails. | **TAG IN THE FIELD WORLD: JITTERS** | A red cross over it. | "Wrong" buzz. |
| 7 | 40–46 s | World | The tag drawn on the glass sheet stays rock still, photo after photo (flashes keep coming, the tag doesn't move). | **TAG IN THE WHEEL WORLD: STILL** | A green tape measure unrolls from the dashed robot. | Calm pad. |
| 8 | 46–54 s | World | The green tape runs from the dashed robot to the goal: "0.36 m ahead, 0.10 m left". → The same tape lifts and hangs on the solid robot in the field world. | **MEASURE THE GOAL FROM THE ROBOT** | The field pose snaps 3 cm. | Tape "zip". |
| 9 | 54–59 s | World | The solid robot snaps 3 cm; the goal at the tape's end snaps with it; the tape length never changes; the robot's blue velocity arrow doesn't flinch. | **ROBOT AND GOAL MOVE TOGETHER** | The bubble shrinks into a trail of dots (chapter 04). | Two synced "boop"s. |

**Narration (ch03-voice.json):**

1. Here's a surprise. The robot keeps two different answers to the question: where am I?
2. The field pose uses the wheels, the gyro, and every tag the cameras see. It's right, but it jumps whenever a camera corrects it.
3. The wheel pose only counts wheel turns and reads the gyro. It's like counting your steps in the dark: smooth, but slowly drifting.
4. In our simulator, it drifted about two millimeters over a whole four meter approach. Rock steady.
5. Engineers call this wheel pose odometry, and its world the odometry frame.
6. So where should the tag live? Drawn in the field world, every jump would drag the tag around with it.
7. Drawn in the wheel world, where nothing jumps, the tag sits perfectly still, photo after photo.
8. Every loop, we measure the goal from the robot in the wheel world, then hang that measurement on the field pose.
9. If the field pose jumps, the robot and its goal jump together. The steering doesn't flinch.

## Chapter 04: Every photo is old (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | World | EXTREME CLOSE-UP: a camera lens; its LED flashes; a small photo card pops out with a yellow timer at 0.000 s. | **CLICK** | The card drops onto a conveyor. | Shutter click. |
| 2 | 7–14 s | World | The card rides past CAPTURE → PROCESS → SEND stations; the yellow timer counts to **0.035 s**. → Meanwhile the robot drives on (blue arrow, 1.5 m/s). | **CAPTURE → PROCESS → SEND: 0.035 s** | The card reaches the robot's brain. | Conveyor rattle, ticking. |
| 3 | 14–19 s | World | A green ruler between the robot at the flash and the robot now: **5 cm**. | **1.5 m/s × 0.035 s ≈ 5 cm** | A tag is drawn from the robot's current spot. | Short slide whistle. |
| 4 | 19–26 s | World | The tag drawn from where the robot is now lands 5 cm past the real one (red ghost tag). | **DRAWN FROM "NOW": 5 cm WRONG** | The photo card gets a time stamp. | "Wrong" buzz. |
| 5 | 26–35 s | World | The photo card gets a yellow time stamp, like a printed photo. → WIDE: behind the robot, a trail of small green dots, 250 a second, stretching back 1.5 s (375 dots). | **EVERY PHOTO HAS AN AGE · THE WHEELS REMEMBER 1.5 s** | The trail glows. | Ticking clock. |
| 6 | 35–44 s | World | Rewind: a ghost robot slides back along the trail to the dot that matches the photo's time; the tag is drawn from the ghost and lands on the real tag (green check). | **REWIND TO WHEN IT WAS TAKEN** | The ghost fades. | Tape rewind, then a click. |
| 7 | 44–49 s | World | A card stamped "0.31 s: TOO OLD" is tossed into a bin. | **OLDER THAN 0.25 s? SKIP IT** | Many cards start raining in (chapter 05). | Paper toss. |

**Narration (ch04-voice.json):**

1. Now, timing. A photo is never fresh. It has to be taken, processed, and sent to the robot's brain.
2. That trip takes a few hundredths of a second, say thirty five thousandths. The robot keeps driving the whole time.
3. At one point five meters per second, that's about five centimeters of driving.
4. Draw the tag from where the robot is now, and it lands five centimeters in the wrong place.
5. So every photo carries its age. And the wheel pose keeps a trail: two hundred fifty dots a second, for one and a half seconds.
6. We rewind the trail to the moment the photo was taken, stand a ghost robot there, and draw the tag from the ghost. Right on target.
7. Photos older than a quarter of a second are thrown away. Too stale to trust.

## Chapter 05: Many photos, one answer (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | World | WIDE: both cameras firing as the robot approaches; white dots land around the true tag, one per photo. | **UP TO 50 PHOTOS A SECOND, PER CAMERA** | Push in on the dots. | Rain-like taps. |
| 2 | 7–14 s | World | Split view: from 3 m the dots scatter ≈2 cm; from 1 m they cluster ≈2 mm. → EXTREME CLOSE-UP of the near cluster against a millimetre ruler. | **FAR: FUZZY (≈2 cm) · NEAR: SHARP (≈2 mm)** | An inset of two friends. | Taps spread, then tighten. |
| 3 | 14–21 s | World | Inset: two friends guess where a door is; one stands beside it, one across the hall; a taller trust bar over the near friend. | **TRUST THE ONE STANDING CLOSE** | The inset folds away. | Light "aha" chime. |
| 4 | 21–31 s | World | One estimate marker (white, team outline) nudges toward each new dot: big nudges for near dots, small for far ones; a trust bar flashes beside each dot. | **NUDGE TOWARD EACH PHOTO: MORE FOR SHARP ONES** | A "sureness" ring appears around the marker. | Gentle pulls. |
| 5 | 31–38 s | World | The ring shrinks as photos pile up; the nudges shrink too. | **SURER → SMALLER NUDGES: A SMART AVERAGE** | A dot lands far away. | Rising tone. |
| 6 | 38–47 s | World | A wild dot 30 cm off bounces off a 25 cm fence around the marker. → Three wild dots in a row: the marker jumps to the newest one and its ring resets. | **JUMP > 25 cm: IGNORED · 3 IN A ROW: START OVER** | A camera straight in front of the tag. | Bounce, bounce, bounce, whoosh. |
| 7 | 47–54 s | World | A camera straight in front of the tag: two ghost tags, tilted left and right, both fit the same picture. Stamp: UNSURE → SKIP. | **UNSURE PHOTO? SKIP IT** | A counter appears. | Two ghosts wobble. |
| 8 | 54–60 s | World | A counter ticks 1, 2, 3; a padlock closes around the marker. | **3 GOOD PHOTOS → LOCKED** | The robot drives in close. | Lock click; motif resolves. |
| 9 | 60–68 s | World | The last few cm: the tag slides out of both view cones (they turn grey); the robot keeps gliding on its dashed wheel pose and stops inside the 2 cm circle. | **TAG OUT OF VIEW? FINISH ON THE WHEELS** | Push in on one camera's bracket (chapter 06). | Quiet glide; soft landing. |

**Narration (ch05-voice.json):**

1. Each camera can send up to fifty photos a second. Every photo puts the tag in a slightly different place.
2. In our simulator, far photos scatter by a couple of centimeters, and close photos by a couple of millimeters.
3. If two friends guessed where a door is, you'd trust the one standing next to it. Same idea here.
4. So the robot keeps one best guess and nudges it toward each new photo: a big nudge for a sharp photo, a small one for a fuzzy one.
5. As photos agree, the guess gets surer, and each new photo moves it less. It's a smart average.
6. A photo more than twenty five centimeters from the guess is ignored. After three jumps in a row, the robot decides it was wrong, and starts over.
7. Some photos are unsure. Seen straight on, a tag can look tilted either way. Those photos are skipped.
8. Three good photos, and the robot is locked on. Now its goal comes from the tag.
9. In the last few centimeters, the tag leaves the cameras' view. Its place in the wheel world is known, so the wheels finish the job.

## Chapter 06: Crooked cameras (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–6 s | World | EXTREME CLOSE-UP: a camera on its bracket turns 1° (a purple arc, "1°", exaggerated). | **CAMERA 1° CROOKED** | Its view ray extends toward the tag. | Low "uh-oh" note. |
| 2 | 6–13 s | World | A thin wedge grows from the lens along the ray: 1.7 cm wide at 1 m, 3.5 cm at 2 m (green labels). | **1° → 1.7 cm PER METRE** | Dots start landing. | Stretching tone. |
| 3 | 13–19 s | World | Dots from that camera land, all shifted the same way; the smart-average marker settles calmly on the shifted spot. | **SAME ERROR EVERY PHOTO → AVERAGING CAN'T FIX IT** | An inset of a bathroom scale. | Identical repeated "tick". |
| 4 | 19–27 s | World | Inset: a bathroom scale reads +1 kg every time someone steps on; then a hand zeroes it once. | **ZERO IT ONCE** | Back to the field. | Scale "boing", then a click. |
| 5 | 27–36 s | World | A wooden jig pushes the robot exactly onto the real scoring spot. | **PARK ON THE REAL SPOT (USE A JIG)** | The robot starts to reverse. | Wood "clack". |
| 6 | 36–43 s | World | WIDE: the robot backs straight away 0.6 m at 0.15 m/s (blue arrow); for each camera, dotted "should be" tags and measured dots appear side by side. | **BACK AWAY 0.6 m, SLOWLY** | Each camera compares. | Reverse beeps. |
| 7 | 43–51 s | World | Each camera gets a small "fix" sticker (a tiny arrow and angle). → The sticker travels with the camera to another tag on the field, which also lands right. | **EACH CAMERA LEARNS ITS OWN FIX** | The two corner cameras glow. | Two fix chimes. |
| 8 | 51–58 s | World | Left: one centre camera sees the tag head-on (two ghost tilts). Right: two corner cameras see it from the sides (one answer). | **TWO CORNER CAMERAS** | The floor starts tilting into a graph (chapter 07). | Two ghosts merge into one. |

**Narration (ch06-voice.json):**

1. One more problem, and it's sneaky. What if a camera is mounted just one degree crooked?
2. Then every tag it sees looks shifted sideways, about one point seven centimeters for every meter of distance.
3. Every photo is wrong the same way, so averaging a thousand of them still gives the same wrong answer.
4. It's like a bathroom scale that always reads one kilogram heavy. Weighing ten times won't fix it. Zeroing it once will.
5. So we calibrate each camera once. The robot is pushed onto a real scoring spot with a jig, so its position is known exactly.
6. Then it backs straight away, slowly, for zero point six meters. It knows where the tag should appear in every photo.
7. Each camera compares what it saw with what it should have seen, and learns its own small fix, good for every tag.
8. And we use two cameras on the front corners. They see the tag from the sides, where its tilt is clear.


# Part 3: The maths

## Chapter 07: Chaining poses (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–6 s | Board | WIDE: the whole field from above tilts flat into the Board; the carpet's edges become the x axis (green, right) and y axis (green, up). The robot shrinks to a green dot with a purple heading arrow. | **POSE = (x, y, θ)** | A pose card writes on. | Smooth whoosh as the floor tilts. |
| 2 | 6–15 s | Board | The dot sits at (1.00, 0.50) facing 0°; pose card: x and y green (metres), θ purple (degrees), live readouts tied to the dot. A small "rewound wheel memory" tag hangs off it. | **ROBOT AT CAPTURE: (1.00, 0.50, 0°)** | A little robot-axes cross rides on the dot. | Writing scratch. |
| 3 | 15–22 s | Board | In the robot's own little axes, a white arrow to the tag, which is drawn facing back toward the robot. | **TAG IN ROBOT: (2.00, 0.10, 180°)** | The ⊕ symbol writes on. | Pixel blips. |
| 4 | 22–31 s | Board | The formula writes on: **tag = robot ⊕ tagInRobot**; the ⊕ glows. | **⊕ = "COMPOSE"** | A "1. ROTATE" label. | Sigmoid write-on. |
| 5 | 31–38 s | Board | Step 1: the white arrow swings by the robot's heading; at 0° it stays put and a "0°" tag flashes. | **1. ROTATE BY THE ROBOT'S HEADING** | A "2. ADD" label. | Click. |
| 6 | 38–46 s | Board | Step 2, three sums written on: 1.00 + 2.00 = 3.00; 0.50 + 0.10 = 0.60; 0° + 180° = 180°. The tag lands at (3.00, 0.60). | **2. ADD: (3.00, 0.60, 180°)** | The result box glows. | Three "plinks". |
| 7 | 46–54 s | Board | The tag pose gets a white box with the team outline, labelled "measured, photo 2". A bookmark ribbon: "the code makes these too". | **MEASURED TAG: (3.00, 0.60, 180°)** | The heading arrow starts to swing. | Bell. |
| 8 | 54–62 s | Board | What-if: the robot's heading arrow swings to 30°; the white arrow rotates with it to (1.68, 1.09). | **IF THE ROBOT FACED 30°: (1.68, 1.09)** | The add step replays. | Swing whoosh. |
| 9 | 62–71 s | Board | Add: (1.00 + 1.68, 0.50 + 1.09) = (2.68, 1.59), heading 210°. → EXTREME CLOSE-UP on a red ghost at (3.00, 0.60), the no-rotation answer, with a ruler: 1.04 m. | **SKIP THE ROTATE: 1 m WRONG** | The white arrow reverses. | Low buzz on the red ghost. |
| 10 | 71–78 s | Board | The arrow flips direction: **⁻¹ = the trip backwards**. → Two poses, A and B; a green tape from B to A: "A seen from B". | **INVERSE ⁻¹ · "SEEN FROM"** | The x axis turns yellow: time (chapter 08). | Rewind chirp. |

**Narration (ch07-voice.json):**

1. The floor tilts into a graph. A pose is three numbers: x, y, and theta, the heading.
2. This is the robot when photo two was taken, from the rewound wheel memory: x one, y zero point five, facing zero degrees.
3. The camera gives the tag in the robot's own frame: two ahead, zero point one left, turned one eighty to face us.
4. To chain them, we compose, written as a plus in a circle. Tag in the world equals robot, composed with tag in robot.
5. Composing has two steps. First, rotate the second arrow by the robot's heading. Our heading is zero, so nothing turns.
6. Second, add. One plus two is three. Zero point five plus zero point one is zero point six. Headings add to one eighty.
7. So photo two puts the tag at three, zero point six, facing us. The code will make exactly these numbers.
8. Why rotate? If the robot faced thirty degrees, ahead points another way. The arrow turns to one point six eight, one point zero nine.
9. Add, and the tag lands at two point six eight, one point five nine. Skip the rotation, and you'd be about a meter wrong.
10. Two more tools. The inverse, a small minus one, is the trip backwards. And seen from measures one pose from another.

## Chapter 08: Rewind the clock (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | Board | The x axis turns into a yellow time axis; a marker drops: "now = 12.500 s". | **ONE CLOCK: now = 12.500 s** | Yellow bars stack backward from now. | Tick. |
| 2 | 7–16 s | Board | Three yellow bars stack backward from now: since arrival 0.015 s, capture 0.008 s, processing 0.012 s. | **AGE = 0.015 + 0.008 + 0.012** | The bars merge. | Three clicks. |
| 3 | 16–22 s | Board | The bars merge into **0.035 s**; formula: capture time = now − age = 12.500 − 0.035 = **12.465 s**. | **CAPTURE AT 12.465 s** | A World inset opens. | Bell. |
| 4 | 22–31 s | Board | Formula writes on: error = v × age = 1.5 × 0.035 = 0.0525 m ≈ 5 cm (v blue, age yellow, error green). → World inset: the red ghost tag 5 cm off. | **IGNORE THE AGE: v × age ≈ 5 cm** | The inset closes; a trail of dots lines the time axis. | Soft "wrong" buzz. |
| 5 | 31–38 s | Board | WIDE (inset): the field from above with the robot's trail; on the Board, the same trail as dots along the time axis, one every 0.004 s; the capture time falls between two of them. | **A DOT EVERY 0.004 s** | Zoom to the two dots. | Fine ticks. |
| 6 | 38–48 s | Board | EXTREME CLOSE-UP: two dots: t = 12.464 s, x = 0.9985 m; t = 12.468 s, x = 1.0045 m. | **12.464 → 0.9985 · 12.468 → 1.0045** | A fraction bar. | Two "plinks". |
| 7 | 48–53 s | Board | Fraction: (12.465 − 12.464) ÷ 0.004 = 0.25. | **A QUARTER OF THE WAY** | A straight green line joins the dots. | Click. |
| 8 | 53–62 s | Board | Blend: x = 0.9985 + 0.25 × (1.0045 − 0.9985) = 0.9985 + 0.0015 = **1.0000**; the point slides along the straight line between the dots. | **x = 1.0000: A STRAIGHT-LINE BLEND** | The result flies into the pose card from chapter 07. | Bell. |
| 9 | 62–69 s | Board | Small footnote: "WPILib blends along a tiny arc; over 4 ms it's the same." → Simulator card: "no age fix at all, driving straight in: median 0.4 → 0.8 cm"; "turning while looking: matters more". | **STRAIGHT APPROACH: FORGIVING · TURNING: NOT** | A bell curve rises (chapter 09). | Soft tone. |

**Narration (ch08-voice.json):**

1. Now the timing maths. Every time is on one clock. Say now is twelve point five zero zero seconds.
2. The photo's age has three parts: zero point zero one five since it arrived, plus the camera's capture and processing time, eight and twelve thousandths.
3. Together, zero point zero three five seconds. So the photo was taken at twelve point four six five.
4. Ignore the age, and the error is v, the speed, times the age. One point five times zero point zero three five: about five centimeters.
5. The wheel memory stores a dot every four thousandths of a second. Our capture time falls between two dots.
6. At twelve point four six four, x was zero point nine nine eight five. At twelve point four six eight, it was one point zero zero four five.
7. Our moment is a quarter of the way from one to the other.
8. So x is zero point nine nine eight five, plus a quarter of six millimeters: exactly one. A straight line blend between the dots.
9. In our simulator, timing mistakes barely hurt a straight approach. Turning while looking at the tag is where they really matter.

## Chapter 09: How sure is one photo? (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | Board | New axes: d (distance, green) across, σ (sigma, white with team outline) up, in cm. | **σ = HOW FAR OFF A PHOTO MIGHT BE** | The formula writes on. | Writing scratch. |
| 2 | 7–16 s | Board | **σ_xy = 0.005 + 0.01 · d²** (metres) writes on; its curve draws itself. | **σ_xy = 0.005 + 0.01·d²** | Readouts ride the curve. | Pen line. |
| 3 | 16–23 s | Board | Readouts pop on the curve: d = 1 → 1.5 cm; d = 2 → 4.5 cm; d = 3 → 9.5 cm. | **1 m: 1.5 cm · 2 m: 4.5 cm · 3 m: 9.5 cm** | Dive into the camera. | Three rising plinks. |
| 4 | 23–31 s | World | EXTREME CLOSE-UP: the tag's image on the camera sensor: 122 pixels wide from 1 m, then it shrinks to 61 pixels from 2 m. → WIDE: pull out to the field showing the two distances. | **TWICE AS FAR → HALF AS BIG** | Back to the Board. | Shrinking "zoop". |
| 5 | 31–40 s | Board | **σ_yaw = 0.5° + 1.5° · d** (purple) writes on; readouts: 1 m → 2°, 2 m → 3.5°. | **σ_yaw = 0.5° + 1.5°·d** | A note card slides in. | Pen line. |
| 6 | 40–48 s | Board | Note card: "Simulated camera: ≈1.5 mm of depth scatter at 1 m. The code assumes 15 mm: a cautious first guess, to be tuned from robot logs." | **CAUTIOUS FIRST GUESSES** | The 4.5 cm readout lifts off. | Soft tone. |
| 7 | 48–57 s | Board | 4.5 cm → 0.045 m → squared → 0.002025 m². → Two bells appear on a number line: a photo's bell and the estimate's bell. | **VARIANCE = σ² = 0.002025** | The bells line up (chapter 10). | Bell "ding". |

**Narration (ch09-voice.json):**

1. Next: how much should we trust one photo? We use sigma, the spread: how far off a photo might be.
2. For x and y, the code uses zero point zero zero five, plus zero point zero one times d squared, where d is the distance.
3. At one meter that's one point five centimeters. At two meters, four point five. At three, nine point five.
4. Why squared? Twice as far, the tag looks half as big, and depth comes from its size. So far errors grow fast.
5. For heading, the spread is zero point five degrees, plus one point five degrees per meter. Two degrees at one meter, three point five at two.
6. These are cautious first guesses, much bigger than our simulated camera's scatter. The team will tune them from robot logs.
7. The maths uses sigma squared, called the variance: four point five centimeters becomes zero point zero zero two zero two five. Our guess has one too.

## Chapter 10: The smart average (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | Board | A number line for x. Photo 1's bell (from 2 m) sits at 2.97 and becomes the estimate: a white marker with the team outline. | **PHOTO 1 (2 m): GUESS x = 2.97** | Photo 2's bell drops in at 3.00. | Soft thud. |
| 2 | 7–15 s | Board | Photo 2's bell lands at 3.00. Formula writes on: **K = variance ÷ (variance + noise)**. | **K, THE GAIN** | The estimate's bell widens a hair. | Sigmoid write-on. |
| 3 | 15–23 s | Board | Drift: the estimate's bell widens: 0.002029 + 0.002² = 0.002033. | **DRIFT: + 0.002²** | Photo 2's noise label. | Small stretch sound. |
| 4 | 23–30 s | Board | Photo 2: σ = 4.51 cm → noise 0.002034. The two bells are almost the same width. | **PHOTO 2 NOISE: 0.002034** | The division writes on. | Plink. |
| 5 | 30–35 s | Board | K = 0.002033 ÷ (0.002033 + 0.002034) = **0.50**. | **K = 0.50** | The update formula. | Bell. |
| 6 | 35–44 s | Board | **new = old + K × (measured − old)**: 2.97 + 0.50 × 0.03 = **2.985**. The marker springs halfway. | **x: 2.97 → 2.985** | Rows for y and heading. | Slide. |
| 7 | 44–51 s | Board | A three-row table fills: y 0.57 → 0.585 (green); θ 179° → 179.5° (purple). | **y: 0.585 · θ: 179.5°** | The bell narrows. | Two plinks. |
| 8 | 51–58 s | Board | **variance × (1 − K)**: 0.002033 × 0.50 = 0.001017; σ 4.5 → 3.2 cm; the bell narrows. → EXTREME CLOSE-UP on the narrowing bell's peak. | **SURER: σ 4.5 → 3.2 cm** | A row of equal photos. | Narrowing tone. |
| 9 | 58–66 s | Board | Four equal bells in a row; gains 1, ½, ⅓, ¼ pop above them; a "plain average" bracket. → WIDE (inset): the field, with dots landing on the tag. | **EQUAL PHOTOS: 1, ½, ⅓, ¼ = THE ORDINARY AVERAGE** | A name card. | Counting plinks. |
| 10 | 66–74 s | Board | Name card: "KALMAN FILTER", with three small copies labelled x, y, θ. | **A TINY KALMAN FILTER PER AXIS** | A new photo approaches (chapter 11). | Motif sting. |

**Narration (ch10-voice.json):**

1. Here's the whole trick. Photo one, from two meters, started our guess at x equals two point nine seven.
2. Photo two says three. How far should we move? The gain, K, decides: our variance, over our variance plus the photo's noise.
3. First, widen our guess a hair for two millimeters of drift. Its variance becomes zero point zero zero two zero three three.
4. Photo two is also from two meters. Its noise is zero point zero zero two zero three four. Almost the same.
5. So K is zero point five. Equal trust, so we move halfway.
6. New guess equals old guess, plus K times the difference. Two point nine seven, plus half of three centimeters: two point nine eight five.
7. The same for y, now zero point five eight five, and for heading, now one seventy nine point five degrees.
8. Then the variance shrinks, times one minus K. We're surer: sigma drops from four point five to three point two centimeters.
9. With equally sharp photos, this is the ordinary average. The first counts fully, the second gets a half, the third a third.
10. This smart average is called a Kalman filter. Ours is tiny: one for x, one for y, and one for heading.

## Chapter 11: Closer photos pull harder, and the rules (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | World | WIDE: a passing robot blocks both cameras; ours drives on its wheels from 2 m to 1 m from the tag. | **VIEW BLOCKED: KEEP DRIVING ON THE WHEELS** | The blocker leaves; the floor tilts to the Board. | Whoosh past. |
| 2 | 7–16 s | Board | Photo 3, from 1.005 m: measured (3.00, 0.60, 180°); σ = 1.5 cm; noise 0.000228; a narrow bell. | **PHOTO 3 (1 m): NOISE 0.000228** | A gain bar. | Sharp "ping". |
| 3 | 16–26 s | Board | K = 0.001021 ÷ (0.001021 + 0.000228) = **0.82**. → A bar beside it: the same moment, a photo from 2 m: **0.33**. | **K = 0.82 (FROM 2 m: 0.33)** | The marker slides. | Two bars rising. |
| 4 | 26–35 s | Board | The marker slides 82% of the way to (2.997, 0.597, 179.8°). EXTREME CLOSE-UP: a 4 mm gap to the true tag (3.00, 0.60). → Counter 3, padlock: LOCKED. | **(2.997, 0.597, 179.8°) · LOCKED** | A gain-over-time graph. | Lock click. |
| 5 | 35–44 s | Board | Graph of K over a made-up straight approach (two cameras, one photo each every 0.02 s): 1, 0.5, 0.34 … dips to ≈0.05 near 2 m, rises to ≈0.14 at 1 m and ≈0.2 at the end. | **GAIN: FALLS AS PHOTOS PILE UP, RISES AS THEY SHARPEN** | The curve's floor glows. | The graph line draws. |
| 6 | 44–51 s | Board | The drift term glows under the curve's floor: "never zero". | **DRIFT KEEPS THE FILTER LISTENING** | A checklist slides in. | Soft hum. |
| 7 | 51–59 s | Board | Checklist rows 1–2: distance ≤ 3.5 m (σ there 12.75 cm); age ≤ 0.25 s. | **≤ 3.5 m · ≤ 0.25 s** | Row 3. | Two ticks. |
| 8 | 59–66 s | Board | Row 3: ambiguity ≤ 0.3; a dial from 0 (sure) to 1 (coin flip); skipped photos drop into a "log" tray, not the bin. | **AMBIGUITY ≤ 0.3 (SKIPPED ONES STILL LOGGED)** | Row 4. | Dial click. |
| 9 | 66–76 s | Board | Row 4: √(dx² + dy²) ≤ 0.25 m, else a jump; 3 jumps in a row → restart, count back to 1. Row 5: locked when count ≥ 3. | **JUMP: √(dx² + dy²) > 0.25 m · 3 IN A ROW → RESTART** | Photo 2's numbers tick each row. | Fence "boing". |
| 10 | 76–83 s | Board | Photo 2's values tick each row: 2.0025 m ✓, 0.035 s ✓, 0.1 ✓, 0.042 m ✓. | **PHOTO 2 PASSES EVERY RULE** | Two worlds appear side by side (chapter 12). | Four ticks, then a chord. |

**Narration (ch11-voice.json):**

1. Back to our story. A passing robot blocks the view while ours drives on, to one meter from the tag.
2. Photo three comes from one meter. Sigma is just one point five centimeters, so its noise is zero point zero zero zero two two eight.
3. Our variance plus drift is zero point zero zero one zero two one, so K is zero point eight two. From two meters, only zero point three three.
4. The guess jumps to two point nine nine seven, zero point five nine seven: about four millimeters from the true tag. Three good photos: locked.
5. Over a whole approach, the gain falls as photos pile up, to about zero point zero five, then climbs as photos sharpen near the tag.
6. The little drift we add with every photo keeps the gain above zero, so the filter never stops listening.
7. Before voting, a photo must pass some rules: no farther than three point five meters, no older than a quarter second.
8. Ambiguity at most zero point three: zero is sure, one is a coin flip. Skipped photos still go in the log.
9. No jump: the square root of dx squared plus dy squared must stay within zero point two five meters. Three jumps in a row restart the guess.
10. Photo two passed them all: two meters, thirty five thousandths old, ambiguity zero point one, four centimeters from the guess.

## Chapter 12: Re-anchoring the goal (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | Board | Two panels: left, the wheel world (glass grid) holding the tag; right, the field world (field axes) holding the robot. A gap between them. | **TAG: WHEEL WORLD · DRIVING: FIELD WORLD** | A bridge sketch spans the gap. | Two tones. |
| 2 | 7–16 s | Board | Step 1: goal = tag ⊕ tagToGoal. The green 0.60 m arrow rotates by 180° and points back toward the robot: (−0.60, 0). | **1. GOAL = TAG ⊕ (0.60, 0, 180°)** | Add. | Swing. |
| 3 | 16–24 s | Board | With the rounded tag (3.00, 0.60, 180°): 3.00 − 0.60 = 2.40; heading 180° + 180° = 0°. Goal (2.40, 0.60, 0°). | **GOAL IN THE WHEEL WORLD: (2.40, 0.60, 0°)** | Step 2. | Plink. |
| 4 | 24–32 s | Board | Step 2: wheel pose now (2.04, 0.50, 0°); "seen from": (2.40 − 2.04, 0.60 − 0.50) = (0.36, 0.10, 0°). A green tape between them. | **2. GOAL SEEN FROM THE ROBOT: (0.36, 0.10, 0°)** | The tape lifts. | Tape "zip". |
| 5 | 32–42 s | Board | Step 3: field pose now (13.34, 4.10, 0°) ⊕ (0.36, 0.10, 0°) = (13.70, 4.20, 0°). The tape hangs on the field-world robot. | **3. FIELD GOAL: (13.70, 4.20, 0°)** | The whole formula writes on. | Bell. |
| 6 | 42–48 s | Board | Full formula: **goal_field = fieldPose_now ⊕ (odometry_now⁻¹ ⊕ goal_odometry)**. | **THE RE-ANCHOR** | A vision correction flashes. | Sigmoid write-on. |
| 7 | 48–58 s | Board | The field pose jumps +3 cm in y: 4.10 → 4.13; the goal: 4.20 → 4.23. EXTREME CLOSE-UP on the tape: still (0.36, 0.10). | **POSE +3 cm → GOAL +3 cm · GAP UNCHANGED** | Video 2's loop icon. | Two synced "boop"s. |
| 8 | 58–64 s | Board | Video 2's drive loop icon reads only the gap, "goal − pose"; the robot's blue velocity arrow doesn't change. | **THE DRIVE LOOP SEES THE SAME GAP** | A dashed map-goal marker appears. | Calm pad. |
| 9 | 64–71 s | World | WIDE: back on the field; the map goal (13.67, 4.17) appears as a dashed marker; a red ruler to the real goal: 4 cm. | **MAP GOAL: 4 cm OFF = TWICE THE BUDGET** | A camera ray reappears (chapter 13). | Low thud. |

**Narration (ch12-voice.json):**

1. The tag lives in the wheel world, but the drive loop steers by the field pose. We need a bridge.
2. Step one: goal equals tag, composed with tag to goal. Rotate zero point six by one eighty degrees, and it points back at the robot.
3. With our tag, rounded, the goal is two point four, zero point six, facing zero degrees: right in front of the tag.
4. Step two: measure the goal from the wheel pose now, two point zero four, zero point five. Thirty six centimeters ahead, ten left.
5. Step three: hang that tape on the field pose, thirteen point three four, four point one. The field goal is thirteen point seven, four point two.
6. Together: the field goal is the field pose now, composed with the goal seen from the robot now.
7. Now a camera nudges the field pose three centimeters, to four point one three. The goal moves to four point two three. The gap is unchanged.
8. The drive loop only looks at that gap, so the jump cancels and the robot doesn't twitch.
9. And the map's goal? Thirteen point six seven, four point one seven. Four centimeters from the real spot: twice our whole budget.

## Chapter 13: One crooked degree (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–6 s | Board | A camera at the origin; a green ray of length d to the tag; a second ray rotated by ε (epsilon, purple). | **ε = THE CAMERA'S ANGLE ERROR** | The thin triangle highlights. | Pen line. |
| 2 | 6–12 s | Board | The thin triangle's far side lights up: **sideways = d × tan ε**. → EXTREME CLOSE-UP on the narrow tip of the triangle. | **SIDEWAYS = d · tan ε** | A tan readout. | Write-on. |
| 3 | 12–22 s | Board | tan 1° = 0.0175; table: 1 m → 1.7 cm, 2 m → 3.5 cm, 3 m → 5.2 cm. | **tan 1° = 0.0175 → 1.7 cm PER METRE** | The tag's own turn. | Three plinks. |
| 4 | 22–29 s | Board | The tag also looks turned 1°; the 0.60 m goal arrow swings: 0.60 × 0.0175 = 1.05 cm; the robot's heading arrow is 1° off. | **GOAL SWINGS 1 cm · ROBOT FACES 1° WRONG** | An equation row. | Swing. |
| 5 | 29–37 s | Board | **photo = truth + bias + noise**; as photos are averaged, the noise bar shrinks toward 0 and the bias bar stays full height. | **AVERAGE = TRUTH + BIAS** | A simulator card. | Hiss fading; a steady tone stays. |
| 6 | 37–46 s | Board | Simulator card, one error at a time: element 4 cm / 2° off → 100%; camera mount yaw 0.8° → 92–95%. → WIDE (inset): the simulated field. | **IN OUR SIMULATOR: CAMERA 0.8° → 92–95%** | Third bar. | Dull thud. |
| 7 | 46–54 s | Board | Third bar: the camera's tag-solve yaw bias 1° → 78–83%. | **SOLVE BIAS 1° → 78–83% · THE CAMERA IS NOW THE ERROR** | The jig appears (chapter 14). | Low note. |

**Narration (ch13-voice.json):**

1. Back to the crooked camera. Call its angle error epsilon. The tag is d meters away.
2. The sideways error is the far side of a thin triangle: d times the tangent of epsilon.
3. The tangent of one degree is zero point zero one seven five. So one point seven centimeters per meter, and three and a half at two meters.
4. The tag also looks turned one degree, so the goal swings about a centimeter and the robot faces one degree wrong.
5. Write each photo as truth plus bias plus noise. Averaging shrinks the noise toward zero, but the bias stays, every single time.
6. In our simulator, a misplaced element no longer hurts. But a camera turned zero point eight degrees drops success to about ninety three percent.
7. And a one degree bias in the camera's own tag solve drops it to about eighty. Now the camera is the error.

## Chapter 14: The calibration maths (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | Board | WIDE: the robot on the jig at the real element, seen from above; the floor tilts to the Board and its wheel pose writes on: (4.00, 1.00, 0°). | **PARKED: (4.00, 1.00, 0°)** | The tagToGoal arrow reverses. | Jig clack. |
| 2 | 7–15 s | Board | tagToGoal (0.60, 0, 180°) reverses: the inverse is the trip from the robot to the tag. expected tag = parked ⊕ tagToGoal⁻¹ = (4.60, 1.00, 180°). | **TAG MUST BE AT (4.60, 1.00, 180°)** | The robot backs off. | Rewind chirp. |
| 3 | 15–22 s | Board | The robot backs to (3.70, 1.00, 0°); the tag seen from it: (0.90, 0.00, 180°), drawn dotted. | **EXPECTED: (0.90, 0.00, 180°)** | The crooked camera's dot. | Reverse beeps. |
| 4 | 22–31 s | Board | The camera (25 cm forward, 20 cm left, turned 1°) reports a white dot 1.2 cm off the dotted one. EXTREME CLOSE-UP on the two dots. | **MEASURED: (0.8964, −0.0113, 179.0°)** | The fix A. | "Wrong" blip. |
| 5 | 31–40 s | Board | **A ⊕ measured = expected** → compose both sides with measured⁻¹ → **A = expected ⊕ measured⁻¹**. | **A = EXPECTED ⊕ MEASURED⁻¹** | The result. | Two write-ons. |
| 6 | 40–48 s | Board | A = (0.0035, −0.0043, 1.00°); the 1° glows purple. | **A = (0.0035, −0.0043, 1.00°)** | Back-off positions. | Bell. |
| 7 | 48–56 s | Board | A computed at 0, 0.3 and 0.6 m of back-off: the same each time; a flat line on a "sample vs distance" plot. | **SAME A AT EVERY DISTANCE → A FLAT LINE** | The averaging step. | Three identical plinks. |
| 8 | 56–65 s | Board | Up to about 200 photos per camera (4 s × 50). An angle trap: 179° and −179°: plain average 0° (red), sine-and-cosine average 180° (green). | **AVERAGE ANGLES WITH sin AND cos** | Applied to photos. | Trap "clang", then a fix chime. |
| 9 | 65–71 s | Board | Every photo from that camera: A ⊕ measured lands on expected. | **EVERY PHOTO: A ⊕ MEASURED** | A two-column card. | Chime. |
| 10 | 71–77 s | Board | Column one, green: ABSORBS: the camera's mounting error, the solve's yaw bias at those distances, small lens errors. | **ABSORBS: MOUNT, SOLVE BIAS, LENS** | Column two. | Tick. |
| 11 | 77–85 s | Board | Column two, red: ALSO ABSORBS: parking error; the calibration tag's own mounting error. Icons: a jig, a well-built element. | **ALSO ABSORBS: PARKING ERROR, THE TAG'S OWN ERROR** | Bar charts (chapter 15). | Warning tone. |

**Narration (ch14-voice.json):**

1. Calibration maths. The robot is parked exactly on the scoring spot. Its wheels say four, one, facing zero.
2. Tag to goal, run backwards, is the trip from robot to tag. So the tag must be at four point six, one, facing back.
3. After backing off thirty centimeters, the tag should appear zero point nine meters straight ahead, facing the robot.
4. A camera turned one degree reports zero point eight nine six four, minus zero point zero one one three, one seventy nine: a centimeter off.
5. We want a fix, A, so that A composed with measured equals expected. Undo measured on both sides: A equals expected, composed with measured inverse.
6. Here A is three and a half millimeters, minus four point three millimeters, and one degree. The one degree undoes the camera's turn.
7. Work it out anywhere along the back-off: A comes out the same. A flat line is what a good calibration log looks like.
8. A is averaged over every photo. Angles use sine and cosine, so one seventy nine and minus one seventy nine average to one eighty, not zero.
9. Then every photo from that camera gets the same fix: A, composed with measured.
10. It absorbs the camera's mounting error, the solve's yaw bias at those distances, and small lens errors.
11. But it also absorbs any parking mistake, and the calibration tag's own error. So park with a jig, on a well-built element.

## Chapter 15: What our simulator says (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | World | WIDE: the simulator's world: many ghost robots start 2.8 m back, up to 0.8 m to the side, already moving. | **IN OUR SIMULATOR** | The camera rises to a bar chart. | Many soft drive hums. |
| 2 | 7–14 s | Board | Success rule card: "said DONE, and truly within 2 cm and 1°". → EXTREME CLOSE-UP: a robot bumper edge inside the 2 cm circle. | **SUCCESS = DONE AND TRULY INSIDE** | Bar-chart axes. | Click. |
| 3 | 14–19 s | Board | Bars: map-based 57 / 31 / 15 (default / hard / very hard). | **MAP: 57 · 31 · 15** | Next bar group. | Three thuds. |
| 4 | 19–25 s | Board | Bars: wheel-world tracker, no calibration: 100 / 72 / 46. | **TAG, NO CALIBRATION: 100 · 72 · 46** | Next group. | Rising tones. |
| 5 | 25–30 s | Board | Bars: plus per-camera calibration and the retuned drive loop: 100 / 99 / 98. | **+ CALIBRATION: 100 · 99 · 98** | A hard-world panel. | Three chimes. |
| 6 | 30–38 s | Board | Hard world: no calibration 66; taught goal 64; per-camera fix 100. | **HARD WORLD: 66 · 64 · 100** | A camera-layout panel. | Two thuds, one chime. |
| 7 | 38–43 s | Board | One centre camera 73–78%; two corner cameras 100%; one corner camera 98–100%. | **CENTRE 73–78% · CORNERS 100%** | A "barely matters" list. | Ghosts merge. |
| 8 | 43–50 s | Board | List: frame rate 20–120 fps, pixel noise, handoff speed 0–3 m/s, 100 Hz wheel updates: all 98–100%. → Caveat card: "simulator only; robot tests will replace these". | **BARELY MATTERS · SIMULATOR ONLY** | The camera pushes into the robot's brain (chapter 16). | Soft chord. |

**Narration (ch15-voice.json):**

1. Here's what our simulator says. Every run starts two point eight meters back, a bit to the side, already moving.
2. A success means the robot says it's done, and it's truly within two centimeters and one degree.
3. Driving to the map: fifty seven, thirty one, fifteen percent, as the simulated world gets messier.
4. Tracking the tag in the wheel world, without calibration: one hundred, seventy two, forty six.
5. Add per-camera calibration and the retuned drive loop: one hundred, ninety nine, ninety eight.
6. On the hard world: no calibration, sixty six. Teaching the goal spot instead, sixty four. Fixing each camera, one hundred.
7. One center camera: seventy three to seventy eight percent. Two corner cameras: one hundred.
8. Frame rate, pixel noise, and handoff speed barely mattered. These are simulator numbers with guessed errors. Robot tests will replace them.


# Part 4: The code

Every code chapter uses the same traced approach as Part 3. Ghost values (debugger-style grey text at
the end of a line, in the quantity's colour) show photo 2's loop unless the row says "photo 3's loop"
or "calibration". The line-number gutter stays on screen the whole time.

## Chapter 16: DriveToTag, the whole file (lines 27–77) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–9 s | Code | WIDE: the field from above, the robot mid-approach; the camera pushes into its "brain" and an editor panel unfolds: `DriveToTag.java`, highlight on line 27; a small video-2 loop icon hangs off `extends DriveToPose`. | **DriveToTag.java** | The highlight steps to line 28. | Panel-unfold whoosh. |
| 2 | 9–15 s | Code | Lines 28–32 light one by one; ghosts: `tagId` 1, `tagToGoal` (0.60, 0.00, 180°) (green, purple), `tracker` TagTracker, `mapGoal` null. | **FIELDS** | The constructor. | One tick per line. |
| 3 | 15–22 s | Code | Lines 42–49; line 48 glows and a small magnifier icon (TagTracker) pops out, wired to two camera icons. | **ONE TRACKER, EVERY CAMERA** | `initialize()`. | Pop. |
| 4 | 22–28 s | Code | Line 53 `tracker.reset()`; ghost: "estimate: none, count 0". | **START FRESH** | Lines 55–58. | Wipe sound. |
| 5 | 28–38 s | Code | Lines 55–58; ghosts: `getTagPose(1)` → (14.27, 4.17, 180°) (example map); `transformBy` (= ⊕) → (13.67, 4.17, 0°). Inset: the dashed map-goal marker on the field. | **mapGoal = MAP TAG ⊕ tagToGoal** | Line 58's `orElse`. | Two plinks. |
| 6 | 38–45 s | Code | Line 58 `.orElse(drivetrain.getPose())` and the comment on line 54 glow together. | **NO MAP ENTRY? HOLD STILL** | Line 59. | Soft tone. |
| 7 | 45–53 s | Code | Line 59 `super.initialize()`; then lines 63–65: line 64 lights, then 65; badge: "ORDER: photos first". | **UPDATE THE TRACKER, THEN DRIVE** | `goal()`. | Two clicks. |
| 8 | 53–62 s | Code | Line 70; two ghost rows: photo 2's loop → `Optional.empty` → mapGoal (13.67, 4.17, 0°); photo 3's loop → (13.697, 4.199, −0.2°). → EXTREME CLOSE-UP on the ghost digits flipping; inset: the goal marker hops 4 cm. | **THE HANDOFF IN ONE LINE** | `isFinished()`. | Hop "boing". |
| 9 | 62–68 s | Code | Line 75 `tracker.hasLock() && super.isFinished()`; ghost: false → true near the end. | **DONE ONLY WHEN LOCKED** | The panel slides to `TagTracker.java` (chapter 17). | Lock click. |

**Narration (ch16-voice.json):**

1. Now the real Java. Drive To Tag, line twenty seven, extends Drive To Pose, so it inherits the whole driving loop from video two.
2. It keeps the drivetrain, the tag's ID, tag to goal, one tag tracker, and the map goal.
3. The constructor saves them; line forty eight builds a tag tracker fed by every camera that might see it.
4. Initialize runs once at the start. Line fifty three forgets any old estimate, so every approach starts fresh.
5. Lines fifty five to fifty eight: the map goal is the tag's map pose composed with tag to goal. Here, thirteen point six seven, four point one seven.
6. No map entry? The goal is wherever the robot is, so it holds still until the cameras lock.
7. Execute runs fifty times a second: line sixty four feeds new photos to the tracker first, then line sixty five drives.
8. Goal, line seventy, is the whole handoff: the tag's goal if the tracker has one, otherwise the map goal. After the lock, the goal hops four centimeters.
9. And line seventy five: finished only when locked and arrived. The robot never finishes on the map's guess.

**Code on screen:**

`DriveToTag.java`

```java
 27  public class DriveToTag extends DriveToPose {
 28    private final DriveMechanism drivetrain;
 29    private final int tagId;
 30    private final Transform2d tagToGoal;
 31    private final TagTracker tracker;
 32    private Pose2d mapGoal;
...
 42    public DriveToTag(
 43        DriveMechanism drivetrain, int tagId, Transform2d tagToGoal, Limelight... cameras) {
 44      super("DriveToTag", drivetrain);
 45      this.drivetrain = drivetrain;
 46      this.tagId = tagId;
 47      this.tagToGoal = tagToGoal;
 48      this.tracker = new TagTracker(drivetrain, tagId, cameras);
 49    }
...
 51    @Override
 52    protected void initialize() {
 53      tracker.reset();
 54      // Approach goal from the field map. No map entry: hold still until the cameras lock on.
 55      mapGoal =
 56          AllianceFlip.getTagPose(tagId)
 57              .map(tag -> tag.transformBy(tagToGoal))
 58              .orElse(drivetrain.getPose());
 59      super.initialize();
 60    }
 61
 62    @Override
 63    protected void execute() {
 64      tracker.update();
 65      super.execute();
 66    }
 67
 68    @Override
 69    protected Pose2d goal() {
 70      return tracker.getFieldGoal(tagToGoal).orElse(mapGoal);
 71    }
 72
 73    @Override
 74    protected boolean isFinished() {
 75      return tracker.hasLock() && super.isFinished();
 76    }
 77  }
```

## Chapter 17: TagTracker: the rules, the memory, and a new photo (lines 37–119) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–9 s | Code | `TagTracker.java`, lines 37–44; the Board checklist from chapter 11 flies in and lands on these lines. → Ghosts light in turn on lines 38–44: 3.5 m (green), 0.3, 0.25 s (yellow), 0.25 m (green), 3, 3. | **THE RULES: 3.5 m · 0.3 · 0.25 s · 0.25 m · 3 · 3** | Lines 47–53. | Whoosh as the checklist lands; six ticks. |
| 2 | 9–17 s | Code | Lines 47–53: the σ formulas and the drift lift off the Board and land here; ghosts 0.005, 0.01, 0.5°, 1.5°, 0.002, 0.1°. | **σ AND DRIFT** | The memory fields. | Write-on. |
| 3 | 17–25 s | Code | Lines 62–66; ghosts before photo 2: `tagPose` (2.97, 0.57, 179°), `variance` {0.002029, 0.002029, 0.003735}, `goodFrames` 1, `jumpsInARow` 0. → Lines 77–78 flash: ghost `corrections` {(0, 0, 0°), (0, 0, 0°)}, tiny glasses icons. | **THE MEMORY · ONE FIX PER CAMERA** | `reset()`. | Ticks; pop. |
| 4 | 25–30 s | Code | Lines 85–91 `reset()`; ghosts: `tagPose` null, `goodFrames` 0, `lastFrameIndex` {−1, −1}. | **reset(): FORGET** | `update()`. | Wipe. |
| 5 | 30–36 s | Code | Lines 96–102: six empty lists, dimmed, tagged "for the log". → Line 104: ghost `i` = 0. Inset WIDE: the field, both camera cones. | **update(): EVERY LOOP** | Line 105. | Soft tick. |
| 6 | 36–45 s | Code | Lines 105–109; ghost `frame.frameIndex` 4182 vs `lastFrameIndex[0]` 4181 → new; then stored. | **NEW PHOTO?** | Line 111. | Click. |
| 7 | 45–53 s | Code | Line 111 → the highlight jumps to `findTag`, lines 197–204, loops over the targets, returns the one with `fiducialId` 1 → back to 112. | **IS OUR TAG IN IT?** | Line 115. | "Blip-blip-ding". |
| 8 | 53–61 s | Code | Line 115 ghost: `tagInRobot3d` (2.00, 0.10, …, yaw 180°). → Line 116 ghost: `distance` 2.0025 (green). EXTREME CLOSE-UP on the digits 2.0025. | **distance = 2.0025 m** | Line 117. | Plink. |
| 9 | 61–69 s | Code | Lines 117–119; ghost "2.0025: not ≈ 0, not > 3.5 ✓". | **NO 3D ANSWER OR TOO FAR? SKIP** | Line 121 (chapter 18). | Tick. |

**Narration (ch17-voice.json):**

1. Tag Tracker opens with the rules: three point five meters, ambiguity zero point three, a quarter second, twenty five centimeters, three to restart, three to lock.
2. Lines forty seven to fifty three hold the sigma formula's numbers and the drift: two millimeters and a tenth of a degree.
3. Lines sixty two to sixty six are its memory: the tag pose, three variances, good photos, and jumps in a row.
4. Reset, lines eighty five to ninety one, forgets the tag pose and the counts.
5. Update runs every loop. Line one oh four loops over the cameras; we follow camera zero.
6. Line one oh five grabs its newest frame, one photo. Lines one oh six to one oh nine skip a frame number we've already seen.
7. Line one eleven calls find tag, at line one ninety seven, which looks through every tag in the photo for ours.
8. Line one fifteen gets the tag's 3D pose in robot space. Line one sixteen takes its floor distance: two point zero zero two five meters.
9. Line one seventeen skips the photo if that distance is almost zero, meaning no 3D answer, or beyond three point five meters.

**Code on screen:**

`TagTracker.java`

```java
 37    // Frames outside these limits are skipped. TODO: tune from logs.
 38    private static final double MAX_TAG_DISTANCE_METERS = 3.5;
 39    private static final double MAX_AMBIGUITY = 0.3;
 40    private static final double MAX_FRAME_AGE_SECONDS = 0.25;
 41    // A frame this far from the estimate is a wrong tag or a flipped solve - unless 3 in a row agree.
 42    private static final double JUMP_METERS = 0.25;
 43    private static final int JUMPS_TO_RESTART = 3;
 44    private static final int FRAMES_TO_LOCK = 3;
 45
 46    // Per-frame noise (1 sigma), growing with distance d: xy = floor + k * d^2, yaw = floor + k * d.
 47    private static final double XY_NOISE_FLOOR = 0.005; // m
 48    private static final double XY_NOISE_PER_M2 = 0.01; // m per m^2
 49    private static final double YAW_NOISE_FLOOR = Math.toRadians(0.5);
 50    private static final double YAW_NOISE_PER_M = Math.toRadians(1.5);
 51    // How far the tag may appear to move per frame (odometry drift); keeps the filter listening.
 52    private static final double XY_DRIFT = 0.002; // m
 53    private static final double YAW_DRIFT = Math.toRadians(0.1);
...
 62    private Pose2d tagPose; // odometry frame; null until the first good frame
 63    private final double[] variance = new double[3]; // x, y, yaw of tagPose
 64    private int goodFrames;
 65    private int jumpsInARow;
 66    private double lastSeenTime;
 67    private int approach; // counts reset() calls, so offline replay knows where each approach starts
...
 77      this.corrections =
 78          Arrays.stream(cameras).map(CameraCorrections::get).toArray(Transform2d[]::new);
...
 84    /** Forgets the estimate. Call when starting a new approach. */
 85    public void reset() {
 86      tagPose = null;
 87      goodFrames = 0;
 88      jumpsInARow = 0;
 89      approach++;
 90      Arrays.fill(lastFrameIndex, -1);
 91    }
...
 96    public void update() {
 97      List<Pose2d> tagInRobotLog = new ArrayList<>();
 98      List<Pose2d> robotAtCaptureLog = new ArrayList<>();
 99      List<Double> frameAgeLog = new ArrayList<>();
100      List<Double> ambiguityLog = new ArrayList<>();
101      List<Double> distanceLog = new ArrayList<>();
102      List<Double> cameraLog = new ArrayList<>();
103
104      for (int i = 0; i < cameras.length; i++) {
105        LimelightResults frame = cameras[i].getLatestResults();
106        if (frame.frameIndex == lastFrameIndex[i]) {
107          continue; // nothing new from this camera
108        }
109        lastFrameIndex[i] = frame.frameIndex;
110
111        FiducialTarget tag = findTag(frame);
112        if (tag == null) {
113          continue;
114        }
115        Pose3d tagInRobot3d = tag.getTargetPose_RobotSpace();
116        double distance = tagInRobot3d.getTranslation().toTranslation2d().getNorm();
117        if (distance < 0.01 || distance > MAX_TAG_DISTANCE_METERS) {
118          continue; // ~0 means the camera sent no 3D solve for this tag
119        }
...
197    private FiducialTarget findTag(LimelightResults frame) {
198      for (FiducialTarget target : frame.fiducialTargets) {
199        if (target.fiducialId == tagId) {
200          return target;
201        }
202      }
203      return null;
204    }
```

## Chapter 18: update(): rewind and place (lines 121–138; WheelOdometry) (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–8 s | Code | Lines 122–124; ghosts: `getAgeSeconds()` 0.015, `captureLatencyMillis` 8, `targetingLatencyMillis` 12 (yellow). | **frameAge = ARRIVAL AGE + CAMERA TIME** | The sum. | Three clicks. |
| 2 | 8–15 s | Code | Ghost `frameAge` 0.035 (yellow); the Board's age bars from chapter 08 slide onto it. → Line 125: 0.035 ≤ 0.25 ✓. | **frameAge = 0.035 s** | Lines 128–129. | Bell. |
| 3 | 15–23 s | Code | Lines 128–129; ghost: 12.500 − 0.035 = 12.465 → `getOdometryPoseAt(12.465)`. | **REWIND TO 12.465 s** | The panel slides to `WheelOdometry.java`. | Rewind chirp. |
| 4 | 23–31 s | Code | `WheelOdometry.java` lines 23–24; ghost "keeps 1.5 s". | **WheelOdometry: 1.5 s OF HISTORY** | Lines 34–43. | Page flip. |
| 5 | 31–38 s | Code | Lines 34–43; line 42 pulses 250 times a second. → WIDE (inset): the field from above, the trail of dots growing behind the robot. | **A DOT EVERY 0.004 s** | Lines 56–58. | Fast ticking. |
| 6 | 38–45 s | Code | Lines 56–58; ghost: `getSample(12.465)` blends 12.464 s (0.9985) and 12.468 s (1.0045) → (1.0000, 0.50, 0°). EXTREME CLOSE-UP on the two dots and the blend point. | **robotAtCapture = (1.00, 0.50, 0°)** | `synchronized` glows. | Bell. |
| 7 | 45–50 s | Code | The word `synchronized` glows on lines 34, 46 and 56; two lanes animate: the wheel thread writing, the main loop reading. | **TWO THREADS, ONE MEMORY** | Back to `TagTracker.java` line 130. | Two-lane whoosh. |
| 8 | 50–56 s | Code | `TagTracker.java` lines 130–132; ghost "present ✓". | **NO MEMORY YET? SKIP** | Line 134. | Tick. |
| 9 | 56–64 s | Code | Line 134; ghost `rawTagInRobot` (2.00, 0.10, 180°); a 3D tag flattens onto the floor. | **FLATTEN TO THE FLOOR** | Line 136. | Plink. |
| 10 | 64–72 s | Code | Line 136; chapter 14's **A ⊕ measured** lands on it; ghost `corrections[0]` (0, 0, 0°) → `tagInRobot` (2.00, 0.10, 180°). | **A ⊕ MEASURED** | Line 137. | The formula "lands". |
| 11 | 72–80 s | Code | Line 137 ghost: `ambiguity` 0.1 ≤ 0.3 ✓. → Line 138: chapter 07's compose lands; ghost (3.00, 0.60, 180°), 2.0025 → `fuse(...)`. | **robotAtCapture ⊕ tagInRobot = (3.00, 0.60, 180°)** | Lines 141–159 dim (chapter 19). | Chord. |

**Narration (ch18-voice.json):**

1. Lines one twenty two to one twenty four compute the frame age: time since it arrived, plus the camera's own milliseconds.
2. That's zero point zero three five seconds, like the board. Line one twenty five skips anything older than a quarter second.
3. Lines one twenty eight and one twenty nine rewind: the wheel pose at now minus frame age, twelve point four six five seconds.
4. That lands in Wheel Odometry. Lines twenty three and twenty four create its history buffer, one and a half seconds long.
5. Its update runs two hundred fifty times a second; line forty two stores each pose with its timestamp.
6. Get pose at, line fifty seven, blends the two samples around our time: one, zero point five, zero degrees.
7. Each method is synchronized, because the wheel thread writes while the main loop reads.
8. Back in Tag Tracker, line one thirty skips the photo if the memory has no answer yet.
9. Line one thirty four flattens the 3D pose onto the floor: two point zero zero, zero point one zero, one eighty degrees.
10. Line one thirty six applies the camera's fix, A composed with measured. Our fix is still zero, so nothing changes yet.
11. Line one thirty seven checks ambiguity, zero point one, fine. Line one thirty eight composes robot and tag, three, zero point six, and calls fuse.

**Code on screen:**

`TagTracker.java`

```java
121        // How long ago the picture was taken: time since it arrived + the camera's own latency.
122        double frameAge =
123            frame.getAgeSeconds()
124                + (frame.captureLatencyMillis + frame.targetingLatencyMillis) / 1000.0;
125        if (frameAge > MAX_FRAME_AGE_SECONDS) {
126          continue;
127        }
128        Optional<Pose2d> robotAtCapture =
129            drivetrain.getOdometryPoseAt(Utils.getCurrentTimeSeconds() - frameAge);
130        if (robotAtCapture.isEmpty()) {
131          continue;
132        }
133
134        Pose2d rawTagInRobot = tagInRobot3d.toPose2d();
135        // This camera's calibration fix first, then place the tag where the robot was.
136        Pose2d tagInRobot = Pose2d.kZero.plus(corrections[i]).plus(asTransform(rawTagInRobot));
137        if (tag.ambiguity <= MAX_AMBIGUITY) { // logged either way, so replay can try keeping them
138          fuse(robotAtCapture.get().plus(asTransform(tagInRobot)), distance);
```

`WheelOdometry.java`

```java
 23    private final TimeInterpolatableBuffer<Pose2d> history =
 24        TimeInterpolatableBuffer.createBuffer(1.5); // seconds kept
...
 33    /** Adds one odometry sample. Called on the odometry thread - keep it fast. */
 34    public synchronized void update(SwerveDriveState state) {
 35      if (state.ModulePositions == null) {
 36        return;
 37      }
 38      if (odometry == null) {
 39        odometry = new SwerveDriveOdometry(kinematics, state.RawHeading, state.ModulePositions);
 40      }
 41      latest = odometry.update(state.RawHeading, state.ModulePositions);
 42      history.addSample(state.Timestamp, latest);
 43    }
 44
 45    /** The newest wheel-only pose. */
 46    public synchronized Pose2d getPose() {
 47      return latest;
 48    }
...
 50    /**
 51     * The wheel-only pose at a past time, interpolated between samples.
 52     *
 53     * @param timestampSeconds time in the Phoenix timebase ({@code Utils.getCurrentTimeSeconds()})
 54     * @return empty until the first sample arrives
 55     */
 56    public synchronized Optional<Pose2d> getPoseAt(double timestampSeconds) {
 57      return history.getSample(timestampSeconds);
 58    }
```

## Chapter 19: The log, then fuse() line by line (lines 141–242) (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–8 s | Code | Lines 141–146, then 150–159 scroll by, dimmed with a "LOG" tag; the key names glow in turn: TagInRobot, RobotAtCapture, FrameAgeSeconds, Ambiguity, Distance, Camera, Approach, Locked, SecondsSinceSeen, TagPose. | **EVERYTHING GOES IN THE LOG** | `TagPose` stays lit. | Tape-recorder whir. |
| 2 | 8–13 s | Code | `TagPose` on line 159 gets a bookmark: "spin test". → WIDE (inset): the robot spinning on the field, a still tag marker. | **REMEMBER: TagTracker/1/TagPose** | The highlight jumps to line 207. | Bookmark "snap". |
| 3 | 13–19 s | Code | Line 207; ghosts `measured` (3.00, 0.60, 180°), `distance` 2.0025. | **fuse(): THE SMART AVERAGE** | Lines 208–210. | Motif sting. |
| 4 | 19–28 s | Code | Lines 208–210; chapter 09's σ formulas land on 208–209; ghosts `xyNoise` 0.0451, `yawNoise` 3.50° (0.0612 rad), `noise` {0.002034, 0.002034, 0.003740}. | **NOISE = σ²** | Lines 212–215. | Two plinks. |
| 5 | 28–36 s | Code | Lines 212–215; ghost `tagPose` (2.97, 0.57, 179°) ≠ null → skip; a faded "photo 1" card shows the earlier `restart`. | **NO ESTIMATE YET? RESTART** | Lines 216–218. | Tick. |
| 6 | 36–44 s | Code | Lines 216–218; ghosts `dx` 0.030, `dy` 0.030 (green), `dYaw` 1.0° (purple). | **dx, dy, dYaw** | Line 219. | Three plinks. |
| 7 | 44–52 s | Code | Lines 219–225; ghost `hypot` 0.042 ≤ 0.25 → the jump branch dims. | **4 cm < 25 cm: NO JUMP** | Line 226. | Fence hum. |
| 8 | 52–58 s | Code | Line 226: `jumpsInARow` 0. → Line 228: ghost `drift` {0.000004, 0.000004, 0.000003}. | **AGREED · DRIFT** | The loop. | Tick. |
| 9 | 58–67 s | Code | Lines 230–234: chapter 10's three formulas land on 231, 232 and 233; ghosts for axis 0: 0.002033 → `gain` 0.4999 → 0.001017. EXTREME CLOSE-UP on `gain[axis]` ticking to 0.4999. | **ADD DRIFT · GAIN · SHRINK** | Lines 235–239. | Three write-ons. |
| 10 | 67–76 s | Code | Lines 235–239; ghost new `tagPose` (2.985, 0.585, 179.5°); inset: the white marker springs halfway. | **tagPose = (2.985, 0.585, 179.5°)** | Lines 240–241. | Slide. |
| 11 | 76–83 s | Code | Line 240 `goodFrames` 1 → 2; line 241 `lastSeenTime` 12.500. | **goodFrames = 2** | `restart()` (chapter 20). | Counter click. |

**Narration (ch19-voice.json):**

1. Lines one forty one to one fifty nine log the raw inputs, even unsure photos, so other filters can be replayed later.
2. Remember the log key Tag Pose. You'll watch it in the spin test.
3. Now fuse, line two oh seven: the smart average, with our measured pose and the distance.
4. Lines two oh eight to two ten: the sigma formulas, squared into noise. Zero point zero zero two zero three four for x and y.
5. Lines two twelve to two fifteen restart from this photo if there's no estimate yet. Photo one did that. This time we go on.
6. Lines two sixteen to two eighteen measure the differences: dx zero point zero three, dy zero point zero three, and one degree.
7. Line two nineteen is the jump check. Four centimeters is well inside twenty five, so the jump branch is skipped.
8. Line two twenty six clears the jump counter, and line two twenty eight builds the drift, squared.
9. Lines two thirty to two thirty four run the board formulas per axis: add drift, compute the gain, shrink the variance. Gain: zero point five.
10. Lines two thirty five to two thirty nine move the estimate by gain times difference, to two point nine eight five, zero point five eight five.
11. Line two forty counts one more good photo, now two. Line two forty one notes when we last saw the tag.

**Code on screen:**

`TagTracker.java`

```java
141        tagInRobotLog.add(rawTagInRobot);
142        robotAtCaptureLog.add(robotAtCapture.get());
143        frameAgeLog.add(frameAge);
144        ambiguityLog.add(tag.ambiguity);
145        distanceLog.add(distance);
146        cameraLog.add((double) i);
147      }
148
149      // Raw inputs too, so other filters can be replayed offline (tools/alignsim/replay.py).
150      Logger.recordOutput(logPrefix + "TagInRobot", tagInRobotLog.toArray(Pose2d[]::new));
151      Logger.recordOutput(logPrefix + "RobotAtCapture", robotAtCaptureLog.toArray(Pose2d[]::new));
152      Logger.recordOutput(logPrefix + "FrameAgeSeconds", toArray(frameAgeLog));
153      Logger.recordOutput(logPrefix + "Ambiguity", toArray(ambiguityLog));
154      Logger.recordOutput(logPrefix + "Distance", toArray(distanceLog));
155      Logger.recordOutput(logPrefix + "Camera", toArray(cameraLog));
156      Logger.recordOutput(logPrefix + "Approach", approach);
157      Logger.recordOutput(logPrefix + "Locked", hasLock());
158      Logger.recordOutput(logPrefix + "SecondsSinceSeen", getSecondsSinceSeen());
159      getTagPose().ifPresent(pose -> Logger.recordOutput(logPrefix + "TagPose", pose));
...
206    /** Folds one measurement into the estimate - a tiny Kalman filter per axis (x, y, yaw). */
207    private void fuse(Pose2d measured, double distance) {
208      double xyNoise = XY_NOISE_FLOOR + XY_NOISE_PER_M2 * distance * distance;
209      double yawNoise = YAW_NOISE_FLOOR + YAW_NOISE_PER_M * distance;
210      double[] noise = {xyNoise * xyNoise, xyNoise * xyNoise, yawNoise * yawNoise};
211
212      if (tagPose == null) {
213        restart(measured, noise);
214        return;
215      }
216      double dx = measured.getX() - tagPose.getX();
217      double dy = measured.getY() - tagPose.getY();
218      double dYaw = measured.getRotation().minus(tagPose.getRotation()).getRadians();
219      if (Math.hypot(dx, dy) > JUMP_METERS) {
220        jumpsInARow++;
221        if (jumpsInARow >= JUMPS_TO_RESTART) {
222          restart(measured, noise); // the old estimate was the wrong one
223        }
224        return;
225      }
226      jumpsInARow = 0;
227
228      double[] drift = {XY_DRIFT * XY_DRIFT, XY_DRIFT * XY_DRIFT, YAW_DRIFT * YAW_DRIFT};
229      double[] gain = new double[3];
230      for (int axis = 0; axis < 3; axis++) {
231        variance[axis] += drift[axis];
232        gain[axis] = variance[axis] / (variance[axis] + noise[axis]);
233        variance[axis] *= 1.0 - gain[axis];
234      }
235      tagPose =
236          new Pose2d(
237              tagPose.getX() + gain[0] * dx,
238              tagPose.getY() + gain[1] * dy,
239              tagPose.getRotation().plus(Rotation2d.fromRadians(gain[2] * dYaw)));
240      goodFrames++;
241      lastSeenTime = Utils.getCurrentTimeSeconds();
242    }
```

## Chapter 20: restart, lock, and the goal (lines 162–250; Vision) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–8 s | Code | Lines 244–250; ghosts from photo 1's earlier call: `tagPose` (2.97, 0.57, 179°), `variance` = noise {0.002029, 0.002029, 0.003735}, `goodFrames` 1, `jumpsInARow` 0. | **restart(): START FROM THIS PHOTO** | A badge on line 247. | Reset whoosh. |
| 2 | 8–16 s | Code | Badge on line 247: "count = 1 → lock lost". Inset: DriveToTag's goal falls back to the dashed map marker for a moment. | **A RESTART DROPS THE LOCK** | `hasLock()`. | Low tone. |
| 3 | 16–23 s | Code | Lines 163–165; ghosts in photo 3's loop: `tagPose` ≠ null ✓, `goodFrames` 3 ≥ 3 ✓ → true. | **hasLock() = true** | The comment on line 162. | Lock click. |
| 4 | 23–29 s | Code | The comment on line 162 glows: "Stays true while odometry coasts." → WIDE (inset): the tag leaves both camera cones; the robot glides on. | **STAYS LOCKED WHILE COASTING** | `getFieldGoal`. | Glide. |
| 5 | 29–34 s | Code | Lines 181–184; ghost in photo 2's loop: not locked → `Optional.empty()`. | **NO LOCK → NO TAG GOAL** | Line 185 (photo 3's loop). | Tick. |
| 6 | 34–43 s | Code | Line 185; chapter 12's step 1 lands; ghost `goalInOdometry` (2.397, 0.599, −0.2°). | **STEP 1: TAG ⊕ tagToGoal** | Line 186. | Write-on. |
| 7 | 43–51 s | Code | Line 186; ghosts `getOdometryPose()` (2.04, 0.50, 0°) → `robotToGoal` (0.357, 0.099, −0.2°); a green tape inset. | **STEP 2: SEEN FROM THE ROBOT** | Line 187. | Tape "zip". |
| 8 | 51–59 s | Code | Line 187; ghosts `getPose()` (13.34, 4.10, 0°) → (13.697, 4.199, −0.2°). EXTREME CLOSE-UP on the result digits. | **STEP 3: HANG IT ON THE FIELD POSE** | `Vision.java` slides in beside. | Bell. |
| 9 | 59–69 s | Code | `Vision.java` slides in (lines 100–112 summarised: every frame from every camera becomes one field pose estimate). → Lines 114–119: the "ORDER MATTERS" comment glows; estimate cards sort oldest first, then drop into `addVisionMeasurement`. | **Vision.java: EVERY PHOTO, OLDEST FIRST** | Split view. | Page flip; sorting clicks. |
| 10 | 69–75 s | Code | Split view: Vision feeding the pose estimator; TagTracker touching only `getPose()` on line 187. | **TagTracker ONLY BORROWS THE FIELD POSE, AT THE END** | Calibration code (chapter 21). | Calm chord. |

**Narration (ch20-voice.json):**

1. Restart, line two forty four: the estimate becomes this photo, its variance this photo's noise, and the count goes back to one.
2. A restart also drops the lock, so Drive To Tag falls back to the map goal until three photos agree again.
3. Has lock, line one sixty four: an estimate exists, and at least three good photos. After photo three, it's true.
4. It stays true when the tag leaves view. The wheels carry the estimate the rest of the way.
5. Get field goal, line one eighty one, returns nothing until we're locked.
6. Line one eighty five is step one: tag pose, composed with tag to goal. Two point three nine seven, zero point five nine nine.
7. Line one eighty six is step two: goal minus the wheel pose now, meaning seen from the robot. Thirty six centimeters ahead, ten left.
8. Line one eighty seven is step three: hang it on the field pose. Thirteen point six nine seven, four point one nine nine.
9. For contrast, Vision dot java feeds the field pose. Lines one fourteen to one sixteen add its photos oldest first, since adding one discards newer ones.
10. Tag Tracker never touches that estimator. It only borrows the field pose at the very last step.

**Code on screen:**

`TagTracker.java`

```java
162    /** True once a few frames agree on where the tag is. Stays true while odometry coasts. */
163    public boolean hasLock() {
164      return tagPose != null && goodFrames >= FRAMES_TO_LOCK;
165    }
...
181    public Optional<Pose2d> getFieldGoal(Transform2d tagToGoal) {
182      if (!hasLock()) {
183        return Optional.empty();
184      }
185      Pose2d goalInOdometry = tagPose.transformBy(tagToGoal);
186      Transform2d robotToGoal = goalInOdometry.minus(drivetrain.getOdometryPose());
187      return Optional.of(drivetrain.getPose().transformBy(robotToGoal));
188    }
...
244    private void restart(Pose2d measured, double[] noise) {
245      tagPose = measured;
246      System.arraycopy(noise, 0, variance, 0, 3);
247      goodFrames = 1;
248      jumpsInARow = 0;
249      lastSeenTime = Utils.getCurrentTimeSeconds();
250    }
```

`Vision.java`

```java
100      List<PoseEstimate> estimates = new ArrayList<>();
101      for (Limelight camera : cameras) {
102        for (LimelightResults frame : camera.readResultsQueue()) {
103          // Try MegaTag1 first (2+ tags); fall back to MegaTag2 for a lone tag.
104          PoseEstimate estimate = camera.getPoseEstimate(frame, PoseEstimateType.MT1_WPIBLUE);
105          if (!estimate.isValid() && !spinningTooFast) {
106            estimate = camera.getPoseEstimate(frame, PoseEstimateType.MT2_WPIBLUE);
107          }
108          if (estimate.isValid()) {
109            estimates.add(estimate);
110          }
111        }
112      }
113
114      // ORDER MATTERS: adding a measurement throws away any stored ones that are newer, so feed
115      // both cameras' frames oldest first.
116      estimates.sort(Comparator.comparingDouble(estimate -> estimate.timestampSeconds));
117      for (PoseEstimate estimate : estimates) {
118        drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, estimate.stdDevs);
119      }
```

## Chapter 21: The eye test in code (CameraCorrections, CalibrateCamerasOpMode) (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | Code | `CameraCorrections.java` lines 18–22; ghosts: every entry (0, 0, 0°); the TODO on line 18 glows. | **ONE FIX PER CAMERA NAME** | Line 28. | Page flip. |
| 2 | 7–13 s | Code | Line 28 `getOrDefault(..., new Transform2d())`; ghost "not calibrated → no change". | **NOT CALIBRATED? NO CHANGE** | `CalibrateCamerasOpMode.java`. | Tick. |
| 3 | 13–22 s | Code | `CalibrateCamerasOpMode.java` lines 44–50; ghosts `TAG_ID` 1, `TAG_TO_PARKED` (0.6, 0, 180°), 0.15 m/s (blue), 4.0 s (yellow), 0.25 s. → Lines 76–82, summarised: ghost `away` (−0.15, 0.00) m/s; WIDE (inset): the robot reversing from the element. | **BACK STRAIGHT AWAY: 0.15 m/s × 4 s = 0.6 m** | `start()`. | Ticks; reverse beeps. |
| 4 | 22–29 s | Code | Line 87; chapter 14's first formula lands; ghosts: `getOdometryPose()` (4.00, 1.00, 0°), `TAG_TO_PARKED.inverse()` (0.6, 0, 180°) → `expectedTag` (4.60, 1.00, 180°). | **expectedTag = (4.60, 1.00, 180°)** | Lines 88–95, then `periodic()`. | Bell. |
| 5 | 29–39 s | Code | Lines 88–95 (clear sums, frame indexes, start time, schedule the back-off). → Lines 99–113: `addFrame` per camera; lines 103–108 glow at 4.0 s (cancel the back-off, send the zero-velocity `stop`, lines 55–59); ghost "print at 4.5 s" on line 109. | **COLLECT · STOP AT 4 s · PRINT AT 4.5 s** | `addFrame`. | Clock tick; a soft brake hiss. |
| 6 | 39–45 s | Code | Lines 121–135: the same "new frame? our tag?" checks as TagTracker, with a ghost "same as TagTracker 105–114". | **SAME CHECKS AS TagTracker** | Lines 136–146. | Quick ticks. |
| 7 | 45–52 s | Code | Lines 136–146; ghosts (calibration, 0.30 m backed off): `measured` (0.8964, −0.0113, 179.0°), `frameAge` 0.035, `robotAtCapture` (3.70, 1.00, 0°). | **MEASURED + REWOUND POSE** | Line 148. | Rewind chirp. |
| 8 | 52–58 s | Code | Line 148 `expectedTag.relativeTo(robotAtCapture)` ("seen from"); ghost `expected` (0.90, 0.00, 180°). | **EXPECTED = THE TAG SEEN FROM THE ROBOT** | Lines 149–151. | Plink. |
| 9 | 58–68 s | Code | Lines 149–151; chapter 14's **A = expected ⊕ measured⁻¹** lands; ghost `correction` (0.0035, −0.0043, 1.00°). EXTREME CLOSE-UP on "1.00°". | **A = EXPECTED ⊕ MEASURED⁻¹** | Lines 152–160. | The formula "lands". |
| 10 | 68–77 s | Code | Lines 152–156: sums of x, y, sin, cos and the count; ghost count 1, 2, 3 … (at most about 200). → Lines 158–160: each sample logged with its distance; inset: a flat line plot. | **SUM, AND LOG EACH SAMPLE** | `printCorrections`. | Counter ticking. |
| 11 | 77–85 s | Code | Lines 163–180; ghost on line 178: `atan2(sum sin, sum cos)` → 1.00°; a console prints `"limelight-bl", new Transform2d(0.0035, -0.0043, Rotation2d.fromDegrees(1.00)),` (example values). | **PASTE INTO CameraCorrections** | The printed line flies out. | Typewriter. |
| 12 | 85–93 s | Code | The printed line flies into `CameraCorrections.java` line 22, then lights `TagTracker.java` line 136. A greyed console line (lines 167–171) shows the other case: "limelight-br saw the tag only 12 times - check TAG_ID and the camera". | **EVERY PHOTO FROM THAT CAMERA, FIXED** | Pull out of the brain to the whole field (chapter 22). | Resolve chord. |

**Narration (ch21-voice.json):**

1. Camera Corrections is tiny. Lines nineteen to twenty two map each camera's name to its fix. Right now, all zero.
2. Line twenty eight returns the fix, or no change for a camera never calibrated.
3. The calibration op mode sets the tag, the parked spot, and a straight back-off from the tag, at zero point one five meters per second for four seconds.
4. Start, line eighty seven: the parked pose, composed with tag to parked inverted. Four point six, one, one eighty.
5. Then it backs off. Periodic adds a frame from each camera every loop, stops the robot at four seconds, and prints at four and a half.
6. Add frame, line one twenty one, starts with Tag Tracker's checks: a new frame, with our tag.
7. Lines one thirty six to one forty six get the measured pose, its age, and the rewound wheel pose.
8. Line one forty eight: the tag seen from the robot at capture. Zero point nine, zero, one eighty.
9. Lines one forty nine to one fifty one: expected, composed with measured inverse. The fix: three and a half millimeters, minus four point three, one degree.
10. Lines one fifty two to one fifty six add it to the sums, using sine and cosine for the angle. Then each sample is logged.
11. Print corrections averages the sums; line one seventy eight turns sine and cosine back into an angle. Paste the printed line into Camera Corrections.
12. From then on, line one thirty six in Tag Tracker fixes every photo from that camera. Under twenty photos? It warns instead.

**Code on screen:**

`CameraCorrections.java`

```java
 18    // TODO: run "Calibrate Cameras" and paste its printed lines here. Re-run after a camera moves.
 19    private static final Map<String, Transform2d> BY_CAMERA_NAME =
 20        Map.of(
 21            "limelight-br", new Transform2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
 22            "limelight-bl", new Transform2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
...
 26    /** The camera's correction, or no change if it hasn't been calibrated. */
 27    public static Transform2d get(Limelight camera) {
 28      return BY_CAMERA_NAME.getOrDefault(camera.getName(), new Transform2d());
 29    }
```

`CalibrateCamerasOpMode.java`

```java
 43    // TODO: the tag on the element you park against.
 44    private static final int TAG_ID = 1;
 45    // The scoring spot in the tag's frame, from CAD - the same tagToGoal DriveToTag uses.
 46    private static final Transform2d TAG_TO_PARKED =
 47        new Transform2d(0.6, 0.0, Rotation2d.fromDegrees(180));
 48    private static final double BACK_OFF_SPEED = 0.15; // m/s
 49    private static final double BACK_OFF_SECONDS = 4.0;
 50    private static final double MAX_FRAME_AGE_SECONDS = 0.25;
...
 52    private final DriveMechanism drivetrain;
 53    private final Limelight[] cameras;
 54    private final Command backOff;
 55    // A command that ends sends nothing, so the back-off would keep rolling: stop it ourselves.
 56    private final SwerveRequest.ApplyRobotVelocity stop =
 57        new SwerveRequest.ApplyRobotVelocity()
 58            .withDriveRequestType(DriveRequestType.Velocity)
 59            .withVelocity(new ChassisVelocities());
...
 75      // Robot-relative, straight away from the tag.
 76      Translation2d tagInRobot = TAG_TO_PARKED.inverse().getTranslation();
 77      Translation2d away = tagInRobot.div(-tagInRobot.getNorm()).times(BACK_OFF_SPEED);
 78      SwerveRequest.ApplyRobotVelocity request =
 79          new SwerveRequest.ApplyRobotVelocity()
 80              .withDriveRequestType(DriveRequestType.Velocity)
 81              .withVelocity(new ChassisVelocities(away.getX(), away.getY(), 0.0));
 82      backOff = drivetrain.applyRequest(() -> request).withTimeout(Seconds.of(BACK_OFF_SECONDS));
...
 85    @Override
 86    public void start() {
 87      expectedTag = drivetrain.getOdometryPose().transformBy(TAG_TO_PARKED.inverse());
 88      for (double[] sum : sums) {
 89        Arrays.fill(sum, 0.0);
 90      }
 91      Arrays.fill(lastFrameIndex, -1);
 92      startTime = Utils.getCurrentTimeSeconds();
 93      printed = false;
 94      stopped = false;
 95      Scheduler.getDefault().schedule(backOff);
 96    }
 97
 98    @Override
 99    public void periodic() {
100      for (int i = 0; i < cameras.length; i++) {
101        addFrame(i);
102      }
103      double elapsed = Utils.getCurrentTimeSeconds() - startTime;
104      if (!stopped && elapsed >= BACK_OFF_SECONDS) {
105        stopped = true;
106        Scheduler.getDefault().cancel(backOff);
107        drivetrain.setControl(stop);
108      }
109      if (!printed && elapsed > BACK_OFF_SECONDS + 0.5) {
110        printed = true;
111        printCorrections();
112      }
113    }
...
121    private void addFrame(int i) {
122      LimelightResults frame = cameras[i].getLatestResults();
123      if (frame.frameIndex == lastFrameIndex[i]) {
124        return;
125      }
126      lastFrameIndex[i] = frame.frameIndex;
127      FiducialTarget tag = null;
128      for (FiducialTarget target : frame.fiducialTargets) {
129        if (target.fiducialId == TAG_ID) {
130          tag = target;
131        }
132      }
133      if (tag == null) {
134        return;
135      }
136      Pose2d measured = tag.getTargetPose_RobotSpace().toPose2d();
137      double frameAge =
138          frame.getAgeSeconds()
139              + (frame.captureLatencyMillis + frame.targetingLatencyMillis) / 1000.0;
140      Optional<Pose2d> robotAtCapture =
141          drivetrain.getOdometryPoseAt(Utils.getCurrentTimeSeconds() - frameAge);
142      if (measured.getTranslation().getNorm() < 0.01
143          || frameAge > MAX_FRAME_AGE_SECONDS
144          || robotAtCapture.isEmpty()) {
145        return;
146      }
147      // correction + measured = expected, so correction = expected - measured.
148      Pose2d expected = expectedTag.relativeTo(robotAtCapture.get());
149      Pose2d correction =
150          expected.transformBy(
151              new Transform2d(measured.getTranslation(), measured.getRotation()).inverse());
152      sums[i][0] += correction.getX();
153      sums[i][1] += correction.getY();
154      sums[i][2] += correction.getRotation().getSin();
155      sums[i][3] += correction.getRotation().getCos();
156      sums[i][4] += 1.0;
157      // Per-frame samples: if they drift with distance, one correction can't fix that camera.
158      String key = "CalibrateCameras/" + cameras[i].getName() + "/";
159      Logger.recordOutput(key + "Sample", correction);
160      Logger.recordOutput(key + "Distance", measured.getTranslation().getNorm());
161    }
162
163    private void printCorrections() {
164      System.out.println("Paste into CameraCorrections:");
165      for (int i = 0; i < cameras.length; i++) {
166        double[] sum = sums[i];
167        if (sum[4] < 20) {
168          System.out.printf(
169              "  %s saw the tag only %d times - check TAG_ID and the camera%n",
170              cameras[i].getName(), (int) sum[4]);
171          continue;
172        }
173        System.out.printf(
174            "          \"%s\", new Transform2d(%.4f, %.4f, Rotation2d.fromDegrees(%.2f)),%n",
175            cameras[i].getName(),
176            sum[0] / sum[4],
177            sum[1] / sum[4],
178            Math.toDegrees(Math.atan2(sum[2], sum[3])));
179      }
180    }
```


# Part 5: Recap

## Chapter 22: Recap and check yourself (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–4 s | World | WIDE: one full approach replays at speed, labels popping as each idea kicks in: MAP GOAL → TAG SEEN → PHOTOS FIXED, REWOUND, PLACED → SMART AVERAGE → LOCKED → GOAL HOPS 4 cm → RE-ANCHORED EACH LOOP → TAG OUT OF VIEW → WHEELS FINISH → INSIDE 2 cm. | (the labels as listed) | The robot stops on the mark. | The series theme, one hit per label. |
| 2 | 4–9 s | Board | The camera tilts up to the Board; card 1 writes on. | **1. FAR: THE MAP. NEAR: THE TARGET'S OWN TAG.** | Card 2. | Chime. |
| 3 | 9–15 s | Board | Card 2 writes on, with a small rewind icon and the glass sheet. | **2. PLACE EACH PHOTO WHERE THE ROBOT WAS, IN THE WHEEL WORLD.** | Card 3. | Chime. |
| 4 | 15–22 s | Board | Card 3 writes on, with the bell curves shrinking. | **3. A SMART AVERAGE: TRUST CLOSE PHOTOS, IGNORE JUMPS AND UNSURE ONES.** | Card 4. | Chime. |
| 5 | 22–28 s | Board | Card 4 writes on, with the glasses icon. | **4. FIX EACH CAMERA ONCE.** | The cards stack; question 1. | Chime; the motif resolves. |
| 6 | 28–34 s | Board | Question card 1, then a pause card: a 3 s countdown ring. | **Q1: WHY DOES THE TAG LIVE IN THE WHEEL WORLD?** | The answer flips over. | Ticking countdown. |
| 7 | 34–42 s | World | Answer: the still tag on the glass sheet; robot and goal jumping together on the field pose. | **A1: NO JUMPS THERE · THE GOAL MOVES WITH THE ROBOT** | Question 2. | Answer chime. |
| 8 | 42–51 s | Board | Question card 2 and the pause ring. | **Q2: 2 m/s, PHOTO 0.05 s OLD. ERROR WITHOUT THE REWIND?** | The answer. | Countdown. |
| 9 | 51–55 s | Board | 2 × 0.05 = 0.10 m (blue × yellow = green). | **A2: v × age = 10 cm** | Question 3. | Chime. |
| 10 | 55–62 s | Board | Question card 3 and the pause ring; two equal bells. | **Q3: VARIANCE = NOISE. GAIN? HOW FAR DOES THE GUESS MOVE?** | The answer. | Countdown. |
| 11 | 62–67 s | Board | K = ½; the marker springs halfway. | **A3: K = 0.5 → HALFWAY** | Question 4. | Chime. |
| 12 | 67–75 s | Board | Question card 4 and the pause ring; a camera turned 2°. | **Q4: CAMERA 2° OFF, TAG 1.5 m AWAY. SIDEWAYS ERROR?** | The answer. | Countdown. |
| 13 | 75–80 s | Board | 1.5 × tan 2° = 1.5 × 0.035 = 0.052 m. EXTREME CLOSE-UP on the thin triangle's tip. | **A4: ≈ 5 cm** | Question 5. | Chime. |
| 14 | 80–84 s | Board | Question card 5 and the pause ring. | **Q5: WHY CAN'T MORE PHOTOS FIX THAT?** | The answer. | Countdown. |
| 15 | 84–91 s | Board | The bias bar stays full height while the noise bar shrinks; a glasses icon. | **A5: SAME ERROR EVERY PHOTO → CALIBRATE** | A terminal panel (chapter 23). | Chime; the motif resolves. |

**Narration (ch22-voice.json):**

1. Let's put it all together, with every idea switched on.
2. One. Far away, use the map. Near the target, measure the target's own tag.
3. Two. Place every photo where the robot was when it was taken, in the smooth wheel world.
4. Three. Blend photos into a smart average that trusts close photos, and ignores jumps and unsure photos.
5. Four. Fix each camera once, because a crooked camera is wrong the same way every time.
6. Check yourself. Question one: why does the tag live in the wheel world? Pause and think.
7. The wheel pose never jumps, so the tag stays still. And the goal is measured from the robot each loop, so jumps cancel.
8. Question two: the robot drives two meters per second, and a photo is zero point zero five seconds old. Without the rewind, how far off?
9. Speed times age: two times zero point zero five. Ten centimeters.
10. Question three: our variance equals the photo's noise. What's the gain, and how far does the guess move?
11. The gain is one half, so the guess moves halfway to the photo.
12. Question four: a camera is turned two degrees, and the tag is one and a half meters away. How far sideways does it look?
13. One and a half times the tangent of two degrees. About five centimeters.
14. Question five: why can't averaging more photos fix that error?
15. Every photo is off the same way. The average of the same mistake is the same mistake. Calibrate instead.

## Chapter 23: Try it, and what's next (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–8 s | Code | A terminal panel: `cd tools` then `python3 -m alignsim.replay --synthetic out.wpilog`. | **TRY IT: THE REPLAY TOOL** | The output prints. | Keyboard clicks. |
| 2 | 8–16 s | Code | The output table prints: TagTracker.java (as shipped), no camera correction, keep ambiguous frames, keep ambiguous frames + inflate their yaw noise, near frames only, more forgetting (drift ×4), geo filter. | **SAME PHOTOS, SEVEN FILTERS** | One row highlights. | Printer chatter. |
| 3 | 16–23 s | Code | Highlight "more forgetting (drift ×4)": spread 0.9 → 2.0 mm against the shipped row, in our run. EXTREME CLOSE-UP on the two spread numbers. | **MORE DRIFT = MORE WOBBLE** | The real robot in the shop. | Wobble sound. |
| 4 | 23–33 s | World | The hero robot spins in place on carpet with a tag in view; beside it, a log plot of `TagTracker/1/TagPose`. | **SPIN TEST: ≈1 rad/s, TAG IN VIEW** | The plot reacts. | Spin hum. |
| 5 | 33–42 s | Board | Three mini plots: flat (good); swinging with the rotation (timestamps off); tracing a circle (camera mounting pose off). | **STILL ✓ · SWINGS = TIMING · CIRCLE = CAMERA MOUNT** | A "what's next" card. | Three tones. |
| 6 | 42–50 s | World | What's next: a robot shoves ours near the target; all four wheels slide sideways together; the dashed wheel pose doesn't move. | **WHAT'S NEXT: COLLISIONS** | A red badge. | Crash thud. |
| 7 | 50–58 s | Board | Badge: **32% FALSE "DONE"** after a 300–800 N push near the target, in our simulator. | **IN OUR SIMULATOR: 32% FALSE "DONE"** | Inside the gyro chip. | Alarm blip. |
| 8 | 58–66 s | World | Inside the gyro chip: its accelerometer's proof mass shifts; the wheels' story says "no change"; the mismatch flags a skid. | **PLAN: GYRO-CHIP ACCELEROMETER vs WHEELS (95% CAUGHT IN SIM) · NOT IN THE CODE YET** | The robot backs off. | Detector "beep". |
| 9 | 66–73 s | World | After the flagged skid, the robot backs off 30 cm and re-locks (counter 1, 2, 3, padlock). | **AFTER A SKID: BACK OFF 30 cm, RE-LOCK** | WIDE pull-back. | Reverse beeps, lock click. |
| 10 | 73–81 s | World | WIDE: the whole field, the robot on the mark inside 2 cm; an end card: three lessons (physics, driving, the tag) and frc5712.com. | **frc5712.com** | — | Series theme; success chime; out. |

**Narration (ch23-voice.json):**

1. Try it. From the tools folder, run the replay tool with dash dash synthetic. It simulates one approach and writes a robot-style log.
2. Then it replays those photos through Tag Tracker and six other filter versions, and compares each one with the truth.
3. Compare more forgetting with the shipped filter. In our run, four times the drift made the estimate wobble about twice as much.
4. On the real robot, try the spin test. Turn in place at about one radian per second with a tag in view, and watch Tag Pose.
5. It should stay still. If it swings back and forth, the timestamps are off. If it traces a circle, the camera's mounting pose is off.
6. What's next? Collisions. A push that slides all four wheels together fools the wheel pose, so the tag's place is suddenly wrong.
7. In our simulator, a hard push near the target fooled the robot into a false done thirty two percent of the time.
8. The plan: compare the gyro chip's accelerometer with the wheels. In our simulator, that caught ninety five percent of skids. Not in the code yet.
9. After a skid, the robot would stop trusting its lock, back off thirty centimeters, and lock on again.
10. That's how a robot hits a two centimeter target. The lessons behind it are at f r c fifty seven twelve dot com.


## Fact check

All arithmetic below was recomputed in Python for this brief. "Study" means the simulation reports in
[`docs/alignment/studies/`](../../studies/) and [`tools/alignsim/studies/tag/`](../../../../tools/alignsim/studies/tag/);
none of these results is robot-tested. WPILib line numbers are from the alpha-6 `wpimath` source.

| Claim | Basis |
| --- | --- |
| Drawings allow ±½ in (1.3 cm) and 1°; FIRST warned events can be worse | README §1: "Arena drawings carry ±½ in and 1°"; FIRST 2025 Team Update 12 warned event variation "will exceed" the 0.2–2.9 in difference between the two tag maps. ½ in = 1.27 cm. |
| Goal: within 2 cm and 1° | The simulator's success rule (README §5; tag-tracking study, Setup). |
| Driving to the map: 57% / 31% / 15% | tag-tracking study, bottom line 1 ("global pose, all tags"); `final_results.md` headline tables 57.0 / 31.0 / 15.0. |
| Everything on: 100% / 99% / 98% | tag-tracking study, bottom line 1 (ships) and 4; `final_results.md` "which filter": SHIPPED per-axis KF + per-camera cal 100.0 / 99.0 / 98.0. |
| Tag in the wheel world, no calibration: 100% / 72% / 46% | tag-tracking study, bottom line 1 ("odom frame, measured yaw (first `TagTracker.java`)"). |
| A perfect field pose still drives to where the element should be | README §1 ("a robot that knows its field pose perfectly still ends up a few cm off"); `DriveToTag.java:55–58` (map goal = map tag ⊕ tagToGoal). |
| The camera reports the tag in the robot's frame; facing the tag = 180° | `TagTracker.java:115` (`getTargetPose_RobotSpace`); game-info skill: +X out of the tag face, a robot facing the tag has yaw 180°. |
| tagToGoal = (0.60, 0, 180°) | `TeleopOpMode.java:36–37`, `DriveThenAlignOpMode.java:36–37`, `CalibrateCamerasOpMode.java:46–47`. |
| Switch to the tag only when locked (3 good photos) | `DriveToTag.java:70`; `TagTracker.java:44, 164, 182–184`. |
| Field pose = wheels + gyro + every tag; jumps when vision corrects it | `Vision.java:18–25, 117–119`; `DriveMechanism.java:57–68`; README §3 (REP-105 `map` frame "jumps"). |
| Wheel pose = wheels + gyro only, never jumps; ROS calls it "odom" | `WheelOdometry.java:14–16`; `DriveMechanism.java:62–68`. |
| Wheel pose drifted ≈ 2 mm over a 4 m approach | skid-odometry study, bottom line: "Over a whole 4 m approach it is 0.2 cm (p95 0.4)". |
| Goal measured from the robot each loop, hung on the field pose; jumps cancel | `TagTracker.java:172–188` (doc comment and code). |
| A photo's trip ≈ 35 ms (example); 8 ms capture + 12 ms processing | Example numbers. 8 and 12 ms are the simulator's assumed Limelight latencies (`tools/alignsim/params.py`: `capture_latency` 0.008, `pipeline_latency` 0.012); the 15 ms since arrival is illustrative. The real values come from the log (`FrameAgeSeconds`). |
| ≈ 5 cm moved during the trip | 1.5 m/s × 0.035 s = 0.0525 m. 1.5 m/s is the simulator's handoff speed (tag-tracking study, Setup). |
| 250 dots a second, 1.5 s kept (375 dots) | `WheelOdometry.java:18, 24`; README §4 (250 Hz with a CANivore). 250 × 1.5 = 375. |
| Photos older than 0.25 s are skipped | `TagTracker.java:40, 125–127`. |
| Up to 50 photos a second per camera | `TagTracker.update()` reads the newest frame per camera each 50 Hz loop (`TagTracker.java:105`); README §6 known limits ("≤ 50 fps"). |
| Simulated photo scatter ≈ 2 mm (depth) at 1 m, ≈ 2 cm at 3 m, ≈ 1 mm sideways | alignsim camera model, re-measured for this brief (2,000 photos per distance, corner camera, robot facing the tag): depth σ 1.5 mm at 1 m, 7.4 mm at 2 m, 17 mm at 3 m; sideways 0.4–1.4 mm. |
| Jump > 25 cm ignored; 3 jumps in a row restart | `TagTracker.java:42–43, 219–225`. |
| Unsure (ambiguous) photos skipped, still logged | `TagTracker.java:39, 137` (comment: "logged either way"), `141–146`. Ambiguity: LimelightLib `FiducialTarget.ambiguity` "Lower is better"; the simulator computes it as best ÷ second-best reprojection error, so 0 = sure, near 1 = a coin flip (`tools/alignsim/camera.py`). |
| Head-on, a tag can look tilted either way | tag-tracking study, bottom line 5 ("the single-tag yaw solve is ambiguous" head-on); the camera's tag solve returns two solutions (`camera.py`, IPPE). |
| The tag leaves view in the last few cm; the wheels finish | skid-odometry study, Setup ("leaves both cameras' view about 3 cm from the goal"); `CalibrateCamerasOpMode.java:39`; `TagTracker.java:162` ("Stays true while odometry coasts"). |
| 1° → 1.7 cm per metre | tan 1° = 0.017455 → 1.75 cm at 1 m, 3.49 cm at 2 m, 5.24 cm at 3 m (the repo's docs round to 1.7). |
| Same error every photo, so averaging can't fix it | tag-tracking study, bottom line 2. Average of (truth + bias + noise) → truth + bias as the noise averages out. |
| Calibrate: jig, back off 0.6 m slowly; a fix per camera that carries to every tag | `CalibrateCamerasOpMode.java:33–39, 48–49` (0.15 m/s × 4 s = 0.6 m); tag-tracking study, bottom line 3 ("a property of the camera, so it carries to every tag"). |
| Two corner cameras: 100%; one centre camera: 73–78%; one corner camera: 98–100% | `conditions_results.md`, camera layout (n = 60): centre 73.3 / 78.3, corners 100, one corner 98.3 / 100. |
| Compose = rotate by the heading, then add; headings add | WPILib `Pose2d.transformBy` (`Pose2d.java:216–219`); `Pose2d.plus` calls it (`Pose2d.java:110–112`). |
| (1.00, 0.50, 0°) ⊕ (2.00, 0.10, 180°) = (3.00, 0.60, 180°) | 1 + 2 = 3; 0.5 + 0.1 = 0.6; 0° + 180° = 180°. |
| At 30°: arrow (1.68, 1.09), tag (2.68, 1.59, 210°); 1 m off without the rotate | x = 2 cos 30° − 0.1 sin 30° = 1.7321 − 0.05 = 1.6821; y = 2 sin 30° + 0.1 cos 30° = 1.0866; + (1.00, 0.50) = (2.6821, 1.5866); 30° + 180° = 210°. √(0.318² + 0.987²) = 1.04 m. |
| Inverse = the trip backwards; "seen from" | `Transform2d.inverse` (`Transform2d.java:243–250`); `Pose2d.relativeTo` and `Pose2d.minus` (`Pose2d.java:120–123, 232–235`). |
| Capture time = now − frame age, one clock | `TagTracker.java:122–124, 129`; LimelightLib `getAgeSeconds()` = NT now − receive time; README §4 (shared clock on SystemCore). 12.500 − 0.035 = 12.465 s. |
| Dots at 12.464 s (0.9985 m) and 12.468 s (1.0045 m) blend to 1.0000 m | Robot at 1.5 m/s: 1.00 − 1.5 × 0.001 = 0.9985; 1.00 + 1.5 × 0.003 = 1.0045. (12.465 − 12.464) ÷ 0.004 = 0.25; 0.9985 + 0.25 × 0.0060 = 1.0000. |
| A straight-line blend; WPILib's tiny arc is the same over 4 ms | `TimeInterpolatableBuffer.getSample` blends the samples either side with fraction (t − t₀)/(t₁ − t₀); `Pose2d.interpolate` follows a twist (arc), `Pose2d.java:311–321`. Over a 6 mm step turning even 2 rad/s × 0.004 s = 0.008 rad, arc and chord differ by about 6 mm × 0.008 ÷ 8 ≈ 6 µm. |
| Timing errors barely hurt a straight approach; turning matters more | tag-tracking study, bottom line 8 (no compensation: median 0.4 → 0.8 cm, still 98–100%; 100 ms too early costs 7%); `conditions_results.md` clock-bias tables; README §7 step 4 (spin test). |
| σ_xy = 0.005 + 0.01·d²: 1.5 / 4.5 / 9.5 cm at 1 / 2 / 3 m; 12.75 cm at 3.5 m | `TagTracker.java:47–48, 208`. 0.005 + 0.01 = 0.015; 0.005 + 0.04 = 0.045; 0.005 + 0.09 = 0.095; 0.005 + 0.1225 = 0.1275. |
| Twice as far → half as big: 122 px at 1 m, 61 px at 2 m | Simulated Limelight 4 focal length 740 px (`params.py`) × 0.1651 m tag (`FieldParams.tag_size`) ÷ d = 122.2 and 61.1 px. |
| σ_yaw = 0.5° + 1.5°·d: 2° at 1 m, 3.5° at 2 m | `TagTracker.java:49–50, 209`. |
| The code's σ are cautious first guesses | Simulated depth scatter 1.5 mm at 1 m vs 15 mm assumed (above); README §6 ("Every limit, noise number, camera correction and `tagToGoal` is a first guess"). |
| Variance = σ²: 0.045² = 0.002025 | Arithmetic; `TagTracker.java:210`. |
| Photo 1: tag (2.97, 0.57, 179°), d 2.0012 m, variance 0.0020294 | (0.97, 0.50, 0°) ⊕ (2.00, 0.07, 179°) = (2.97, 0.57, 179°); d = √(4 + 0.0049) = 2.0012; σ = 0.005 + 0.01 × 4.0049 = 0.045049; σ² = 0.0020294; restart sets variance = noise and count 1 (`TagTracker.java:212–214, 244–250`). |
| Photo 2: noise 0.002034, K = 0.50, estimate (2.985, 0.585, 179.5°), σ 3.2 cm | d = √4.01 = 2.0025; σ = 0.005 + 0.0401 = 0.0451, σ² = 0.0020340. Drift: 0.0020294 + 0.002² = 0.0020334. K = 0.0020334 ÷ (0.0020334 + 0.0020340) = 0.49993. x = 2.97 + 0.49993 × 0.03 = 2.9850; y = 0.57 + 0.49993 × 0.03 = 0.5850. Yaw: σ = 0.5° + 1.5° × 2.0025 = 3.5037° (0.061152 rad, σ² 0.0037396); prior 0.0037355 + (0.1°)² 0.0000030 = 0.0037385; K = 0.49993; 179° + 0.49993 × 1° = 179.50°. Variance after: 0.0020334 × 0.50007 = 0.0010169 → σ = 3.19 cm. Count 2 (`TagTracker.java:228–240`). |
| Equal photos give the ordinary average (1, ½, ⅓, ¼) | With equal noise R and no drift: P₁ = R, K₂ = R ÷ 2R = ½, P₂ = R/2, K₃ = (R/2) ÷ (3R/2) = ⅓, and so on. The 2 mm drift makes the real gains a hair larger. |
| "A tiny Kalman filter per axis" | `TagTracker.java:206` (doc comment). |
| Drift is added only when a photo is fused | `TagTracker.java:228–231` (inside `fuse`, after the jump check), so a blocked stretch adds one drift step, not one per loop. |
| Photo 3 (1 m): noise 0.000228, K = 0.82 (0.33 from 2 m), estimate (2.997, 0.597, 179.8°), 4 mm from the true tag, σ 1.4 cm, locked | (2.00, 0.50, 0°) ⊕ (1.00, 0.10, 180°) = (3.00, 0.60, 180°); d = √1.01 = 1.0050; σ = 0.005 + 0.0101 = 0.0151, σ² = 0.00022801. Prior 0.0010169 + 0.000004 = 0.0010209; K = 0.0010209 ÷ (0.0010209 + 0.0002280) = 0.8174; from 2 m: 0.0010209 ÷ (0.0010209 + 0.0020340) = 0.334. x = 2.985 + 0.8174 × 0.015 = 2.99726; y = 0.59726. Yaw: σ = 2.0075°, σ² 0.0012276; prior 0.0018695 + 0.0000030 = 0.0018726; K = 0.6040; 179.5° + 0.604 × 0.5° = 179.80°. Error √2 × 2.74 mm = 3.9 mm. Variance 0.00018638 → σ = 1.37 cm. Count 3 → `hasLock()` (`TagTracker.java:164`). |
| Gain over a whole approach: 1, 0.50, 0.34 … ≈ 0.05 near 2 m, ≈ 0.14 at 1 m, ≈ 0.2 at the end | Illustration computed with `TagTracker`'s formulas on a made-up straight approach (2.8 m → 0.6 m, speed min(1.5, √(2·6·(d − 0.6)), 6·(d − 0.6)) m/s, two photos per 0.02 s, all passing): K = 1, 0.500, 0.343, 0.256 …; minimum 0.050 at 2.2 m; 0.054 at 2.0 m; 0.136 at 1.0 m; 0.203 at 0.62 m. Not a simulator result. |
| Drift keeps the filter listening | `TagTracker.java:51` ("keeps the filter listening"). |
| Photo 2 passes every rule | 2.0025 ≤ 3.5 m; 0.035 ≤ 0.25 s; 0.1 ≤ 0.3; √(0.03² + 0.03²) = 0.0424 ≤ 0.25 m. |
| Re-anchor: goal (2.40, 0.60, 0°) → seen from the robot (0.36, 0.10) → field goal (13.70, 4.20) | Rounded tag: (3.00, 0.60, 180°) ⊕ (0.60, 0, 180°): (0.60, 0) rotated 180° = (−0.60, 0) → (2.40, 0.60); 180° + 180° = 0°. Exact (code ghosts): (2.99726, 0.59726, 179.80°) ⊕ → (2.39726, 0.59933, −0.20°); minus wheel pose (2.04, 0.50, 0°) → (0.35726, 0.09933, −0.20°); field pose (13.34, 4.10, 0°) ⊕ → (13.69726, 4.19933, −0.20°). `TagTracker.java:185–187`. |
| A +3 cm vision jump moves the goal by the same 3 cm | Field pose (13.34, 4.13, 0°) ⊕ (0.35726, 0.09933) = (13.69726, 4.22933): both y values +0.03, gap unchanged. |
| The drive loop only looks at goal − pose | `DriveToPose.java:164–166` (`toGoal = goal − pose`). |
| Map goal 4 cm from the measured goal (example) | Example map tag (14.27, 4.17, 180°) ⊕ (0.60, 0, 180°) = (13.67, 4.17, 0°); √(0.02726² + 0.02933²) = 0.0400 m. Field numbers are illustrative, not the real tag 1. |
| Tag turned 1° → the goal 0.6 m out swings ≈ 1 cm, robot faces 1° wrong | 0.6 × tan 1° = 0.6 × 0.017455 = 0.0105 m; the goal's heading comes from the tag's (`TagTracker.java:185`). |
| One error at a time: element 4 cm / 2° → 100%; camera mount yaw 0.8° → 92–95%; tag-solve yaw bias 1° → 78–83% | tag-tracking study, bottom line 2 (ablation, n = 60). "About ninety three" and "about eighty" are the middles of those ranges. |
| Calibration: parked (4.00, 1.00, 0°) → tag (4.60, 1.00, 180°) | Inverse of (0.6, 0, 180°): translation −R(−180°)·(0.6, 0) = (0.6, 0), rotation −180° = 180°; (4.00, 1.00, 0°) ⊕ (0.6, 0, 180°) = (4.60, 1.00, 180°). `CalibrateCamerasOpMode.java:87`. |
| After 0.30 m: expected (0.90, 0.00, 180°) | Robot (3.70, 1.00, 0°); tag seen from it: (4.60 − 3.70, 0) = (0.90, 0, 180°). `CalibrateCamerasOpMode.java:148` (`relativeTo`). |
| Measured (0.8964, −0.0113, 179.0°), ≈ 1.2 cm off | A camera at (0.25, 0.20) truly turned +1° but believed straight puts every tag rotated −1° about the camera: Python gives (0.8964, −0.0113, 179.0°); √(0.0036² + 0.0113²) = 1.19 cm. |
| A = expected ⊕ measured⁻¹ = (0.0035, −0.0043, 1.00°), the same at 0, 0.3 and 0.6 m | `CalibrateCamerasOpMode.java:147–151`; tag-tracking study "How the calibration works". A is the 1° turn about the camera point: c − R(1°)c = (0.25 − 0.24647, 0.20 − 0.20433) = (0.0035, −0.0043). Python: identical A at 0, 0.3, 0.6 m; A ⊕ measured = (0.9000, 0.0000, 180.00°). |
| A flat Sample-vs-Distance line is a good sign | `CalibrateCamerasOpMode.java:157–160`; README §7 step 5. |
| Angles averaged with sine and cosine | `CalibrateCamerasOpMode.java:61, 154–155, 178`. 179° and −179°: plain mean 0°, atan2(Σsin, Σcos) = 180°. |
| Up to about 200 photos per camera | 4 s × 50 loops, one `addFrame` per camera per loop (`CalibrateCamerasOpMode.java:100–102`); fewer, since the tag is out of view at the start. |
| What calibration absorbs, and what it also absorbs | tag-tracking study, "How the calibration works". |
| Simulator starts 2.8 m back, up to 0.8 m sideways, moving | tag-tracking study, Setup (±0.8 m, ±25°, already moving). |
| Hard world: no calibration 66%, taught goal 64%, per-camera fix 100% | `bias_results.md`, hard world, n = 80: 66.2 / 63.8 / 100.0 (rows run with the sim's geo filter; README decision 3 quotes the same numbers). |
| Frame rate, pixel noise, handoff speed, 100 Hz wheel updates barely matter | tag-tracking study, bottom line 6; `conditions_results.md` (all 98–100%). |
| DriveToTag code facts | `DriveToTag.java:27–32, 42–49, 53–59, 63–66, 69–71, 74–76`. `getTagPose` gives the map pose (`AllianceFlip.java:48–53`). |
| Execute runs fifty times a second | The 50 Hz (20 ms) robot loop (README §5 "the 50 Hz robot loop"; video 2). |
| TagTracker code facts | `TagTracker.java` lines as quoted in chapters 17–20; `findTag` 197–204; `reset` 85–91. Frame number 4182 is illustrative. |
| WheelOdometry runs on the odometry thread, synchronized | `WheelOdometry.java:18–19, 34, 42, 46, 56`; `DriveMechanism.java:39–40`. |
| `getSample` blends the samples around the time | WPILib `TimeInterpolatableBuffer.getSample` (`TimeInterpolatableBuffer.java:109–140`). |
| A restart drops the lock; DriveToTag falls back to the map goal | `TagTracker.java:247` (count 1) → `hasLock()` false (164) → `getFieldGoal` empty (182–184) → `DriveToTag.java:70` uses `mapGoal`. |
| Vision adds frames oldest first; TagTracker never touches the estimator | `Vision.java:114–119`; README decision 10; `TagTracker.java:187` only reads `getPose()`. |
| CameraCorrections are zero until calibrated; uncalibrated = no change | `CameraCorrections.java:18–22, 28`. |
| Back-off velocity (−0.15, 0) m/s | `CalibrateCamerasOpMode.java:76–81`: tag in robot (0.6, 0) ÷ −0.6 × 0.15. |
| Stops at 4 s with a zero-velocity request; prints at 4.5 s; under 20 photos warns; printed format | `CalibrateCamerasOpMode.java:55–59, 103–112, 167–178` (the stop was added in commit f3ca19d, "Stop with zero velocity, not Idle"). The printed numbers shown are this brief's example sample, not a measured correction. |
| Recap Q2: 10 cm; Q3: K = 0.5; Q4: ≈ 5 cm | 2 × 0.05 = 0.10 m; P ÷ (P + P) = 0.5; 1.5 × tan 2° = 1.5 × 0.03492 = 0.0524 m. |
| Replay tool: `python3 -m alignsim.replay --synthetic out.wpilog`, seven filter variants | `tools/alignsim/README.md` ("Replaying robot logs"); `tools/alignsim/replay.py` `variants()` (shipped + 6 others). |
| More forgetting wobbled about twice as much in our run | Ran for this brief from `tools/` (output written to a scratch folder): spread over the last 0.5 s 0.9 mm / 0.07° for the shipped filter vs 2.0 mm / 0.20° for "more forgetting (drift ×4)". One synthetic run. |
| Spin test: still = good; swings = timestamps; circle = camera mounting pose | README §7, step 4. |
| Collisions: 32% false "done" after a 300–800 N push near the target | skid-odometry study, bottom line; README decision 11. |
| Gyro-chip (Pigeon) accelerometer vs wheels caught 95%; not in the code yet | skid-odometry study ("catches 95% (99% of those over 1 cm) with zero clean-run false alarms"); README decision 11 ("Not in code yet"), decision 13 (Pigeon 2). |
| After a skid: back off 30 cm and re-lock | README decision 11; skid-odometry study, mitigation (c). |
| frc5712.com | teaching skill (team workshop site). |
