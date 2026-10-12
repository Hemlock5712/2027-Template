# Video 3: "Trust the tag, not the map" (driving to a tag: `DriveToTag` and `TagTracker`)

An in-depth lesson in six chapters, built like videos 1 and 2: each chapter is its own short animate
piece with its own voice-over file. The code it explains:
[`DriveToTag.java`](../../../../src/main/java/frc/robot/commands/DriveToTag.java),
[`TagTracker.java`](../../../../src/main/java/frc/robot/subsystems/vision/TagTracker.java),
[`CalibrateCamerasOpMode.java`](../../../../src/main/java/frc/robot/opmodes/CalibrateCamerasOpMode.java).
The study behind it: [tag-tracking.md](../../studies/tag-tracking.md).

```
/animate chapter 1 of the explainer in brief.md, use ch1-voice.json as the voice-over
```

Beat times in the tables are a guide: with a voice-over, animate times each beat to its narration line, so the chapter lengths below come from the narration (about 3 words a second).

## Intake answers (same for every chapter)

| Question | Answer |
| --- | --- |
| **Subject** | How the robot lines up within 2 cm of a real field element using its AprilTag: why the map isn't enough, the two "where am I" answers, old photos, combining photos, and calibrating the cameras. |
| **The one true thing** | Near the target, measure the target itself, place every photo where the robot was when it was taken, and fix each camera once. |
| **Must include** | The map vs the real element; field pose vs wheel pose; the photo's age and the rewind; combining many photos; 1° camera error → 1.7 cm per metre; the calibration drive; corner cameras. |
| **Must avoid** | Claiming robot-tested accuracy (simulation only so far; say "in our simulator"). The words "Kalman", "PnP", "odometry" without a picture first. |
| **Formats and length** | 16:9 first, 9:16 second. 20–45 s per chapter, about 3.6 min for the whole video. |
| **Voice** | Narrated voice-over (one JSON per chapter). |
| **Look** | Math style, matching videos 1 and 2. Top-down field, the same robot hero; the AprilTag drawn as a black-and-white square. |
| **Hero** | The same top-down robot, now with two cameras on its front corners. TODO: team colours and number. |
| **Music** | Composed in code; a "detective" motif (measuring, checking clues) with a resolve when the robot locks on. |
| **Story format** | F4 mission: the robot wants to stop on the real scoring spot and solves one problem per chapter. Chapter 6 is a recap that replays the whole approach. |

## Numbers on screen

| Thing | Value | Source |
| --- | --- | --- |
| Field tolerance | about ±1 cm and 1° on drawings, often worse at events | arena drawings; FIRST 2025 Team Update 12 |
| Map-based success in the sim | 57% / 31% / 15% (default / hard / very hard field) | tag-tracking study |
| Measuring the tag, cameras calibrated | 100% / 99% / 98% | tag-tracking study |
| Wheel-pose history kept | last 1.5 s, 250 times a second | `WheelOdometry` |
| Photo age | a few hundredths of a second; photos older than 0.25 s are skipped | `TagTracker` |
| Photo error | about 2 mm at 1 m, about 2 cm at 3 m (depth); about 1 mm sideways | camera model in the simulator |
| Jump check | a photo 25 cm from the estimate is ignored, unless 3 in a row agree | `TagTracker` |
| Locked | after 3 agreeing photos | `TagTracker` |
| Camera turned 1° | tag looks 1.7 cm off per metre away | geometry |
| Calibration | park on the spot, back away 0.6 m at 0.15 m/s | `CalibrateCamerasOpMode` |
| Hard field: no calibration / taught goal / per-camera fix | 66% / 64% / 100% | bias study |
| One centre camera vs two corner cameras | 73–78% vs 100% | conditions study |

## Chapter 1: The map lies (about 35 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–10 s | A blueprint field map with a scoring spot. The real paper field appears on top, and the element sits a little off its blueprint outline. | **MAP ≠ FIELD** | Zoom to the gap: a ruler shows a few cm and a small angle. | Paper rustle. |
| 2 | 10–22 s | Three fields from the simulator side by side: slightly off, more off, very off. A robot that trusts the map lands outside the 2 cm circle more and more often: 57%, 31%, 15% hits. | **DRIVING TO THE MAP: 57% → 15%** | The tag on the element starts to glow. | Three "miss" thuds, getting heavier. |
| 3 | 22–36 s | The robot drives to the map spot until it's close, then a line from its camera locks onto the tag and the goal marker jumps onto the real element. | **FAR: MAP · NEAR: TAG** | The same three fields again; now the robot hits 100%, 99%, 98%. | Lock "click", then three hit chimes. |
| 4 | 36–50 s | Question marks pop up over the robot: "Which way am I facing? How old is this photo? Is my camera straight?" | **3 PROBLEMS** | Cut to the robot's "thought bubble" (chapter 2). | Detective motif. |

**Narration (ch1-voice.json):**

1. The field map says exactly where every target is. Real fields are built by people, so the real target is often a few centimeters off.
2. In our simulator, a robot that drives to the map hits a two centimeter target only fifty seven percent of the time. On a messier field, fifteen.
3. So when it gets close, our robot stops trusting the map, and measures the target's own AprilTag instead.
4. Measuring well takes solving three problems. Let's take them one at a time.

## Chapter 2: Two answers to "where am I?" (about 40 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–12 s | Two robot icons in a thought bubble. "Field pose" (blue): uses wheels plus every tag on the field; it jumps a little whenever a camera corrects it. "Wheel pose" (orange): only wheels and the gyro; smooth, drifts slowly. | **FIELD POSE: CORRECT BUT JUMPY · WHEEL POSE: SMOOTH** | The blue one twitches; the orange one glides. | Two different blips. |
| 2 | 12–28 s | The tag is drawn in the orange (wheel) world. As the blue pose jumps, the tag in the orange world stays perfectly still. | **TRACK THE TAG IN THE SMOOTH WORLD** | A goal marker is drawn next to the tag, in the orange world. | Calm pad. |
| 3 | 28–44 s | Each loop the goal is carried from the orange world to the blue world by measuring it from the robot: "the goal is 1.2 m ahead and 0.1 m left of me". When the blue world jumps, robot and goal jump together, so the drive doesn't flinch. | **ROBOT AND GOAL MOVE TOGETHER** | A camera shutter clicks. | Two synced "boop"s. |
| 4 | 44–55 s | The orange robot keeps a short trail behind it: where it was over the last 1.5 s, 250 dots a second. | **IT REMEMBERS THE LAST 1.5 s** | The trail glows (chapter 3 needs it). | Ticking clock. |

**Narration (ch2-voice.json):**

1. The robot keeps two answers to "where am I". The field pose uses every tag it sees, so it's right, but it jumps whenever a camera corrects it.
2. The wheel pose only uses the wheels and the gyro. It drifts a little over a long drive, but over the last few seconds it's smooth and steady.
3. So we place the tag in the smooth wheel world. Then, every loop, we measure the goal from the robot, so if the field pose jumps, the robot and the goal jump together.
4. And the wheel pose remembers where the robot was for the last second and a half. That's about to matter.

## Chapter 3: Every photo is old (about 30 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–12 s | The camera takes a photo (flash). The photo travels along a little conveyor: exposure → processing → network. A timer counts a few hundredths of a second. | **CAPTURE → PROCESS → SEND** | The robot keeps driving during the trip. | Conveyor rattle. |
| 2 | 12–24 s | When the photo arrives, the robot is about 5 cm further on. Drawing the tag from where the robot is *now* puts it 5 cm wrong (red ghost tag). | **PHOTO IS OLD → TAG IN THE WRONG PLACE** | Rewind symbol. | "Wrong" buzz. |
| 3 | 24–40 s | The photo carries its age. The robot rewinds its trail from chapter 2 to that moment, stands a ghost robot there, and draws the tag from the ghost. The tag lands in the right place (green). | **REWIND TO WHEN IT WAS TAKEN** | The ghost fades. | Tape rewind, then a click. |
| 4 | 40–50 s | A photo older than 0.25 s gets a "too old" stamp and is thrown away. | **TOO OLD? SKIP IT** | Many photos start arriving (chapter 4). | Paper toss. |

**Narration (ch3-voice.json):**

1. A camera photo takes time: take it, process it, send it over the network. A few hundredths of a second.
2. At driving speed, the robot moves about five centimeters in that time. Draw the tag from where the robot is now, and it lands five centimeters wrong.
3. So every photo carries its age. The robot rewinds its memory to the moment the photo was taken, and draws the tag from there.
4. Photos older than a quarter of a second are thrown away.

## Chapter 4: Many photos, one answer (about 45 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–12 s | Dots land around the true tag position, one per photo, about 30 a second from two cameras. Far away the dots are spread out (about 2 cm); close up they're tight (about 2 mm). | **FAR: FUZZY · CLOSE: SHARP** | A weighted average marker appears. | Rain-like taps. |
| 2 | 12–28 s | The estimate marker moves toward each new dot, a lot for close (sharp) dots, a little for far (fuzzy) ones. A "trust" bar beside each dot shows its weight. | **TRUST CLOSE PHOTOS MORE** | One dot lands far away. | Gentle pulls. |
| 3 | 28–42 s | A wild dot 30 cm off is ignored (bounces off a 25 cm fence). Three wild dots in a row in the same place: the estimate restarts there (the old one was wrong). A blurry, two-faced photo (head-on, "flip") is skipped. | **IGNORE JUMPS · SKIP UNSURE PHOTOS** | Third good photo arrives. | Bounce, then a reset whoosh. |
| 4 | 42–52 s | After 3 agreeing photos, a padlock closes: LOCKED. | **LOCKED** | The robot drives in close. | Lock click, motif resolves. |
| 5 | 52–60 s | The last few cm: the tag leaves the camera's view; the robot keeps going on its smooth wheel pose and stops on the mark. | **TAG OUT OF VIEW? KEEP GOING ON THE WHEELS** | Zoom into a camera (chapter 5). | Quiet glide. |

**Narration (ch4-voice.json):**

1. Two cameras send dozens of photos a second. Each one is a little off. Far away, by a couple of centimeters. Up close, by a couple of millimeters.
2. So the robot keeps one estimate and nudges it toward each new photo: a lot for close, sharp photos, and only a little for far, fuzzy ones.
3. A photo that jumps more than twenty five centimeters is ignored, unless three in a row agree. And unsure photos, where the tag could be facing two ways, are skipped.
4. Three photos that agree, and the robot is locked on.
5. In the last few centimeters the tag leaves the camera's view, so the robot finishes on its smooth wheel pose.

## Chapter 5: Fix the camera, not the goal (about 40 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–14 s | A camera rotates by 1°. A thin wedge grows from it: 1.7 cm wide at 1 m, 3.4 cm at 2 m. Every photo from it is off the same way, so averaging doesn't help. | **1° → 1.7 cm PER METRE** | The wedge freezes. | Low "uh-oh" note. |
| 2 | 14–34 s | The calibration drive: the robot is pushed into a jig on the real scoring spot, then backs straight away 0.6 m, slowly. Since it knows where it started, it knows where the tag *should* look; each camera compares and gets its own small fix sticker. | **PARK · BACK AWAY · EACH CAMERA LEARNS ITS FIX** | Stickers glow. | Reverse beeps, then two fix chimes. |
| 3 | 34–46 s | Simulator results on a messy field: no calibration 66%, teaching the goal 64%, per-camera fix 100%. | **PER-CAMERA FIX: 66% → 100%** | The two cameras on the robot highlight. | Three result tones. |
| 4 | 46–60 s | One centre camera sees the tag head-on: the tag could be tilted either way (two ghost tags). Two corner cameras see it from the sides: one answer. 73–78% vs 100%. | **TWO CORNER CAMERAS** | Fade to the recap (chapter 6). | Two ghosts merge into one. |

**Narration (ch5-voice.json):**

1. One more problem. If a camera is mounted just one degree crooked, every photo puts the tag almost two centimeters off. Averaging can't fix that, because every photo is wrong the same way.
2. So we calibrate each camera once. Park the robot exactly on the scoring spot, then let it back away slowly. It knows where the tag should appear, so each camera learns its own small fix.
3. On a messy field in our simulator, that took the robot from sixty six percent to a hundred.
4. And use two cameras on the front corners. Straight on, a tag can look tilted either way. From the sides, there's only one answer.

## Chapter 6: Putting it together (about 25 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–30 s | Full approach in one shot, with labels popping as each idea kicks in: map goal far away → cameras see the tag → photos rewound and placed → estimate tightens → LOCKED → goal jumps to the real spot → DriveToPose's loop (video 2) brings it in → tag leaves view → finish on wheels → inside 2 cm. | (labels as listed) | The robot stops on the mark. | The series' main theme, one hit per label. |
| 2 | 30–45 s | The ruler from the trailer shows the piece inside 2 cm. Then a final card with the three lessons: physics (video 1), driving (video 2), the tag (video 3). | **frc5712.com** | — | Success chime, music out. |

**Narration (ch6-voice.json):**

1. Far away, drive to the map. As soon as the cameras lock on, switch to the spot measured from the tag.
2. Every photo is rewound to when it was taken, combined with the others, and corrected for its camera.
3. The same driving loop brings the robot in, and the wheels finish the last few centimeters.
4. That's how a robot hits a two centimeter target.

## Fact check

| Claim | Basis |
| --- | --- |
| Map-based 57 / 31 / 15%; tag + calibration 100 / 99 / 98% | tag-tracking study, headline table (simulation). |
| Field pose jumps; wheel pose smooth | REP-105 map vs odom frames; `Drivetrain/Pose` vs `Drivetrain/OdometryPose`. |
| Goal measured from the robot each loop | `TagTracker.getFieldGoal`: odometry goal re-anchored on the field pose. |
| 1.5 s history at 250 Hz | `WheelOdometry` buffer, CTRE odometry thread. |
| ~5 cm moved during a photo's trip | ~30–35 ms × ~1.5 m/s. |
| Photos older than 0.25 s skipped | `TagTracker.MAX_FRAME_AGE_SECONDS`. |
| ~2 mm at 1 m, ~2 cm at 3 m depth | Simulated Limelight 4 noise; replace with test-plan step 3 measurements. |
| Trust close photos more | Noise grows with distance² in `TagTracker.fuse`. |
| 25 cm jump check, 3 in a row restart, lock after 3 | `JUMP_METERS`, `JUMPS_TO_RESTART`, `FRAMES_TO_LOCK`. |
| Unsure (head-on) photos skipped | `MAX_AMBIGUITY` 0.3. |
| 1° → 1.7 cm per metre | tan(1°) = 0.0175. |
| Calibration drive | `CalibrateCamerasOpMode`: 0.15 m/s for 4 s. |
| 66% / 64% / 100% | bias study, hard field. |
| Centre camera 73–78% vs corners 100% | conditions study, camera layout. |
