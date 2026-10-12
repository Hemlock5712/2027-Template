# Video 0 (trailer): "How a robot hits a 2 cm target"

The one-minute overview of the series: it touches every idea that videos 1-3 explain in depth. A
ready-made brief for the [animate](https://github.com/cth9191/animate) Claude Code skill. Copy this
folder into the animate project, then run:

```
/animate the narrated explainer in brief.md, use voice-script.json as the voice-over
```

It answers the skill's intake questions up front, so you only approve the story, the look and the
storyboard. The narration is already in the skill's `voice/script.json` format
([voice-script.json](voice-script.json)).

## Intake answers

| Question | Answer |
| --- | --- |
| **Subject** | How an FRC robot drives to a scoring spot and lines up within 2 cm of a real field element. |
| **The one true thing** | To be precise, the robot stops trusting the map and measures the target itself, then drives as fast as it can while it can still stop. |
| **Must include** | The map vs. the real field; the AprilTag; "the photo is old" (latency); camera calibration; the speed rule; tire grip and weight shift. |
| **Must avoid** | Claiming tested accuracy numbers or success rates (this is the 2027 plan, tested in simulation, not on a robot yet). Code on screen. Jargon without a picture (no "Kalman", "odometry", "PnP"). |
| **Formats and length** | 16:9 for the website first, 9:16 second. About 60–65 s. |
| **Voice** | Narrated voice-over (script below). |
| **Look** | Cut paper (the skill's proven style, friendly for students). Alternative: math, if you want it to feel like a lesson diagram. |
| **Hero** | A small square swerve robot with bumpers. TODO: team colours and number on the bumpers. |
| **Music** | Composed in code (no track), light and curious, a build-up into the final recap. |
| **Story format** | F4 Mission: the robot wants one thing and solves one problem at a time, then a 4-shot rush recap. |

## Story: beat table

The voice is the clock; times are approximate (~3.2 words per second).

| # | Time | What happens (picture) | On-screen text | Bridge into next beat | Sound's role |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | The robot holds a game piece in front of a paper field element. A tiny target on the element glows; a ruler shows its width next to a thumb. | **± 2 cm** | Camera pulls back to show the whole field. | Soft tick, then music starts. |
| 2 | 5–12 s | A blueprint "map" (dashed outline) lies over the field. The robot drives to the dashed spot and lands a few cm off the real paper element, which sits slightly shifted from its outline. | **MAP ≠ FIELD** | The dashed outline peels away like tape. | Record scratch on the miss. |
| 3 | 12–20 s | Close-up of the black-and-white AprilTag on the element. A line shoots from the robot's camera to the tag; the goal marker jumps from the map spot to the real spot. | **MEASURE THE TARGET** | Snapshot flash: the picture freezes into a photo. | Camera shutter, then a "lock" click. |
| 4 | 20–29 s | The photo floats toward the robot, but the robot keeps rolling; a translucent ghost robot stays where the photo was taken, a gap opening between them. A rewind clock spins and the robot lines the photo up with its ghost. | **THE PHOTO IS 0.03 s OLD** | The ghost fades; zoom into the camera on the robot. | Whoosh, then a tape-rewind sound. |
| 5 | 29–37 s | The camera tilts a tiny bit; a thin wedge grows from it, and at the tag the wedge is 2 cm wide. Then the robot parks on the real spot, backs away slowly, and each camera gets a small "fix" sticker. | **1° → 2 cm** | The robot backs out into an open field. | Low "uh-oh" note, then a bright fix chime per camera. |
| 6 | 37–45 s | Two robots race at the target. The ghost one slows only when close and sails past; ours watches a speed bar that drops just in time and stops on the mark. | **CAN I STILL STOP?** | Zoom into our robot's wheels. | Engine-like rise; a "boing" for the overshoot. |
| 7 | 45–53 s | Cutaway of the robot speeding up: weight arrows grow on the back wheels and shrink on the front; a front wheel spins and sparks. Then it accelerates a bit gentler and all four wheels grip. | **DON'T ASK FOR MORE THAN GRIP** | Quick cuts begin (the rush). | Tire squeal, then a solid grip thud. |
| 8 | 53–59 s | Rush recap, one shot each: map peels off → tag line → rewind clock → camera fix sticker → speed bar → gripping tires. | (icons only) | Final shot: the robot scores. | Build-up on the beat, one hit per shot. |
| 9 | 59–64 s | The piece drops onto the target; the ruler from beat 1 shows it inside 2 cm. End card. | **frc5712.com** | — | Success chime, music out. |

## Narration (also in voice-script.json)

1. Our robot has to drop this piece within two centimeters. About the width of your thumb.
2. Easy: drive to where the map says. But real fields are built by people, so the target is a few centimeters off.
3. So near the end, the robot stops trusting the map. It reads the target's own AprilTag and measures where it really is.
4. But every photo is a little old. By then, the robot moved about five centimeters. So it remembers where its wheels were when the photo was taken.
5. A camera turned just one degree sees the tag almost two centimeters off. So each camera is calibrated once, on the real spot.
6. Now, the drive. A robot that slows down only when it's close overshoots. Ours asks fifty times a second: can I still stop?
7. And it never asks the tires for more than they can grip. Speeding up shifts weight back, so the front wheels slip first.
8. Measure the target. Rewind the photo. Fix the camera. Brake on time. Respect the tires.
9. That's how a robot hits a two-centimeter target.

## Fact check (for whoever edits the script)

Every claim above, with where it comes from. Keep the wording if you change the pictures.

| Claim | Basis |
| --- | --- |
| Target within 2 cm | The design goal in `docs/alignment/README.md`. |
| Real fields are a few cm off the map | FIRST's own warning (2025 Team Update 12) and arena tolerances; README section 1. |
| The photo is ~0.03 s old; ~5 cm at speed | Limelight capture plus processing plus network is a few tens of ms; at 1.5 m/s, 35 ms is about 5 cm. |
| Remembers where its wheels were | `TagTracker` places each frame with the wheel-odometry pose at the capture time. |
| 1° camera turn → almost 2 cm | 1.7 cm per metre of distance; the tag is often a metre or more away. |
| Calibrated once, on the real spot | The "Calibrate Cameras" OpMode. |
| P-only slows too late and overshoots | Controller study: P-on-distance alone overshoots unless it's tuned slow. |
| Fifty times a second | `DriveToPose` runs in the 50 Hz robot loop. |
| Weight shifts back; front wheels slip first | Controller study Q6 (weight transfer); basic physics. |

## Other episodes this could become

- **"Why not just a P controller?"** (30 s, F4): the overshoot race from beat 6, slowed down, with the
  speed bar explained.
- **"The rewind trick"** (30 s, F1 Machine): a photo travels through stations (shutter → processor →
  network → robot) while a clock counts, then the rewind lines it up.
- **"Grip"** (30 s, F5 fixed hero): the robot stays centred while arrows show weight moving between
  wheels as it speeds up, brakes and turns.
