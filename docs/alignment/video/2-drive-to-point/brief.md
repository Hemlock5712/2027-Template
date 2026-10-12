# Video 2: "Fifty decisions a second" (driving to a point: `DriveToPose`)

An in-depth lesson in six chapters, built like video 1: each chapter is its own short animate piece
with its own voice-over file. The code it explains:
[`DriveToPose.java`](../../../../src/main/java/frc/robot/commands/DriveToPose.java) (video 1 covers the
physics it hands off to, [`AccelerationLimiter`](../../../../src/main/java/frc/robot/utils/AccelerationLimiter.java)).

```
/animate chapter 1 of the explainer in brief.md, use ch1-voice.json as the voice-over
```

Beat times in the tables are a guide: with a voice-over, animate times each beat to its narration line, so the chapter lengths below come from the narration (about 3 words a second).

## Intake answers (same for every chapter)

| Question | Answer |
| --- | --- |
| **Subject** | How `DriveToPose` decides, every loop, which way and how fast to drive: the speed rule, turning, corners, and goals that move. |
| **The one true thing** | There is no plan or timer: every 20 ms the robot asks "how far is the goal, and what's the fastest speed I can still stop from?", so the goal is allowed to move. |
| **Must include** | The four questions of the loop; the three speeds and "pick the smallest"; the overshoot race against a P-only robot; the corner rule; moving goals (tag, game piece). |
| **Must avoid** | Claiming robot-tested results (simulation only so far). Naming other teams' code as "wrong": say "a simpler rule" instead. |
| **Formats and length** | 16:9 first, 9:16 second. 20–45 s per chapter, about 2.7 min for the whole video. |
| **Voice** | Narrated voice-over (one JSON per chapter). |
| **Look** | Math style, matching video 1. Top-down field, the same robot hero. |
| **Hero** | The same top-down robot as video 1. TODO: team colours and number. |
| **Music** | Composed in code; a steady 50 Hz-ish tick motif under the loop, a race theme for chapter 3. |
| **Story format** | F4 mission for chapters 1–3 (the robot wants to stop exactly on a mark), F1-style machine for the loop diagram, F5 fixed hero for chapters 4–6. |

## Numbers on screen (from `DriveToPose`)

| Thing | Value | Constant |
| --- | --- | --- |
| Loop rate | 50 times a second (every 20 ms) | robot loop |
| Top speed | 4 m/s | `MAX_SPEED` |
| Planned braking | 6 m/s² | `DECEL` |
| Stopping from 4 m/s | 1.33 m, 0.67 s | 4² ÷ (2 × 6) |
| Brake early by | 0.05 s (0.2 m at 4 m/s) | `REACTION_TIME` |
| Gentle zone | last 0.33 m (speed = 6 × distance) | `KP` |
| "Close enough, stop" | 3 mm | `STOP_DEADBAND` |
| Turning | max 6 rad/s, brakes at 6 rad/s², gentle × 5, stops within 0.3° | `MAX_OMEGA`, `ANGULAR_DECEL`, `KP_HEADING`, `HEADING_DEADBAND` |
| Corner hand-over | 0.4 m or 40% of the shorter leg | `MAX_SWITCH_RADIUS` |
| Corner speeds (r = 0.4 m) | 45°: 2.4 m/s · 90°: 1.55 m/s · 135°: 1.0 m/s | √(6 × 0.4 ÷ tan(turn ÷ 2)) |
| Done | within 2 cm and 2°, slower than 5 cm/s | `isFinished` |

## Chapter 1: Fifty questions a second (about 20 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–8 s | The robot on a field, a target mark 3 m away. A stopwatch shows 20 ms ticking over and over. | **EVERY 20 ms** | The four questions appear around the robot. | Tick motif starts. |
| 2 | 8–28 s | Four cards light up in a loop: "Where is the goal?" (a pin), "How far?" (a ruler), "How fast?" (a speedometer), "How much may it change?" (a padlock with "video 1"). An arrow leaves the robot each tick. | **WHERE · HOW FAR · HOW FAST · HOW MUCH CHANGE** | The loop spins faster and faster. | One soft note per card. |
| 3 | 28–45 s | Contrast: a "schedule" robot follows a timeline and keeps going when the pin moves; our robot re-asks and turns toward the moved pin. | **NO SCHEDULE: JUST "WHERE IS IT NOW?"** | Zoom into the speedometer card (chapter 2). | A "boing" for the schedule robot, a click for ours. |

**Narration (ch1-voice.json):**

1. Every twenty milliseconds, our robot asks four questions.
2. Where is the goal? How far is it? How fast should I go? And how much may my speed change, which is the physics from video one.
3. There's no schedule. If the goal moves, the next answer just points somewhere new.

## Chapter 2: How fast? Three speeds, pick the smallest (about 35 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–10 s | Three speedometers side by side over the robot's path. Needle 1, "top speed", sits at 4 m/s. | **1. TOP SPEED: 4 m/s** | The second gauge lights up. | Gauge chime. |
| 2 | 10–28 s | Gauge 2, "can I still stop?": a braking ramp is drawn backwards from the target; its height at the robot's spot is the speed. At 1.33 m out it equals 4 m/s; closer, it drops. A small gap before the ramp is labelled "brake 0.05 s early". | **2. CAN I STILL STOP? √(2 × 6 × d)** | The ramp glows; the third gauge lights up. | Rising then falling tone along the ramp. |
| 3 | 28–42 s | Gauge 3, "gentle at the end": a straight line, speed = 6 × distance, which is lower than the ramp in the last 33 cm. | **3. GENTLE: 6 × d** | All three needles overlay. | Soft glide. |
| 4 | 42–52 s | The robot drives; at each moment the lowest of the three needles is highlighted. The speed trace draws: flat, then the ramp, then the line. | **PICK THE SMALLEST** | The robot reaches the mark. | Tick motif speeds up and settles. |
| 5 | 52–60 s | The final 3 mm: a microscope view; the robot stops wiggling inside the circle. | **WITHIN 3 mm: STOP** | Fade to the race (chapter 3). | Quiet "settle" note. |

**Narration (ch2-voice.json):**

1. To answer "how fast", the robot works out three speeds and picks the smallest.
2. One: its top speed, four meters per second.
3. Two: the fastest speed it can still stop from in the distance left. From four meters per second, stopping takes one point three meters. It even brakes a twentieth of a second early, to cover its own reaction time.
4. Three: near the end, slow down gently, in a straight line, so it doesn't wobble.
5. Smallest of the three, every loop. And inside three millimeters, it just stops.

## Chapter 3: Why not just slow down when you're close? (about 25 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–8 s | Two robots on parallel lanes, each with a target mark. Lane A: "speed = 5 × distance". Lane B: ours. | **A RACE** | Countdown. | Race theme, three beeps. |
| 2 | 8–24 s | Both go. Robot A is fast but starts braking too late: its speed rule only drops once it's close, its brakes can't keep up, and it sails 2 m past and comes back. Robot B brakes on its ramp and stops on the mark. | **OVERSHOOT: 2 m** | Rewind; lane A gets a gentler rule. | "Boing" for A, a clean click for B. |
| 3 | 24–38 s | Rematch: A uses "speed = 2 × distance". It stops cleanly but crawls in; B finishes 0.8 s sooner. | **SAFE BUT 0.8 s SLOWER** | Both lanes freeze; B's ramp is highlighted. | Clock tick for the wait. |
| 4 | 38–50 s | The single line that makes the difference appears: √(2 × braking × distance). | **ONE LINE: "CAN I STILL STOP?"** | Fade to turning (chapter 4). | Sting. |

**Narration (ch3-voice.json):**

1. A simpler rule: speed equals distance times a number. Slow down only when you're close.
2. Make it quick, and it starts braking too late. In our simulator it overshot by two meters.
3. Make it gentle, and it stops cleanly, but it arrives almost a second later.
4. Asking "can I still stop?" gives us both: full speed for longer, then a stop right on the mark.

## Chapter 4: Turning uses the same three ideas (about 20 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–12 s | The robot needs to rotate 90°. Three dials again, now for spin: max 6 rad/s; "can I still stop turning?"; gentle near the end. | **SAME THREE, FOR TURNING** | The robot rotates while driving. | Gauge chimes. |
| 2 | 12–25 s | It drives and turns at the same time, both finishing together at the mark. | **DRIVE AND TURN AT ONCE** | Zoom on the heading arrow. | Smooth whirr. |
| 3 | 25–35 s | The last 0.3°: the heading arrow stops twitching. | **WITHIN 0.3°: STOP TURNING** | Fade to a route with corners (chapter 5). | Settle note. |

**Narration (ch4-voice.json):**

1. Turning uses the same three ideas: a top spin rate, the fastest spin it can still stop, and gentle near the end.
2. So it can drive and turn at the same time, and both finish together.
3. Within a third of a degree, it stops turning.

## Chapter 5: Corners without stopping (about 30 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–10 s | A route with two waypoints. A stop-and-go robot halts at each corner. | **STOP · GO · STOP · GO** | Rewind. | Stuttering beat. |
| 2 | 10–24 s | Our robot: a circle appears around the corner (0.4 m, or 40% of the shorter leg). The robot hands over to the next leg when it reaches the circle. | **HAND OVER EARLY** | Corner angles appear. | Swoosh at the hand-over. |
| 3 | 24–42 s | Three corners side by side: 45°, 90°, 135°. Each gets a speed: 2.4, 1.55, 1.0 m/s. A curved arc fits inside each corner. | **SHARPER TURN → SLOWER CORNER** | Arcs glow. | Three rising notes, descending in pitch. |
| 4 | 42–55 s | The full route plays: smooth curves, never stopping until the end. In our simulator, routes like this were 11–26% faster than stop-and-go. | **11–26% FASTER (SIM)** | Fade to a moving target (chapter 6). | Flowing version of the race theme. |

**Narration (ch5-voice.json):**

1. Stopping at every corner wastes time.
2. Instead, the robot hands over to the next leg a little before the corner, inside a circle up to forty centimeters wide.
3. How fast through the corner? The sharper the turn, the slower: about two and a half meters per second for a gentle bend, one for a hairpin.
4. In our simulator, routes like this were up to a quarter faster than stopping at every point.

## Chapter 6: A goal that moves (about 30 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–12 s | The pin on the field slides as the robot drives; each tick, the arrow re-aims. | **THE GOAL CAN MOVE** | The pin becomes an AprilTag on a field element. | Ticks continue. |
| 2 | 12–26 s | DriveToTag: far away, the goal comes from the field map; once the cameras lock on, it jumps to the spot measured from the tag. | **MAP FIRST, THEN THE TAG** | The tag becomes a game piece. | Lock "click". |
| 3 | 26–38 s | DriveToPiece: the goal is the piece seen by the camera; it rolls and the robot follows. | **SAME LOOP, NEW GOAL** | The finish check appears. | Rolling sound. |
| 4 | 38–50 s | Done check: three ticks: within 2 cm, within 2°, almost stopped. Then a peek at video 3. | **DONE = 2 cm · 2° · STOPPED** | End card: "Next: driving to a tag" (video 3). | Success sting. |

**Narration (ch6-voice.json):**

1. Because the robot re-asks "where is the goal" every loop, the goal is allowed to move.
2. Driving to a tag, it starts with the field map, then switches to the spot it measures from the tag itself.
3. Driving to a game piece, the goal is wherever the camera sees it.
4. Same loop, every time. It's done when it's within two centimeters, within two degrees, and nearly stopped. Next video: how it measures that tag.

## Fact check

| Claim | Basis |
| --- | --- |
| Every 20 ms | `DriveToPose.execute` runs in the 50 Hz robot loop. |
| Stopping from 4 m/s takes 1.33 m | v² ÷ (2 × `DECEL`) = 16 ÷ 12. |
| Brakes 0.05 s early | `REACTION_TIME`: braking distance is reduced by speed × 0.05 s. |
| Gentle zone 0.33 m | `KP` × d is lower than √(2 × 6 × d) when d < 12 ÷ 36 = 0.33 m. |
| P-only overshot 2 m; gentle P was 0.8 s slower | Controllers study §7: P 5 with accel limit 3 overshot 205 cm (p95); P 2 reached 5 cm in 2.20 s vs 1.39 s for `DriveToPose`. |
| Corner speeds | √(`DECEL` × r ÷ tan(turn ÷ 2)) with r = 0.4 m, `DECEL` = 6. |
| 11–26% faster | Controllers study §3 (stop-and-go vs pass-through routes, original tune). |
| Map first, then the tag | `DriveToTag.goal()`: the map goal until `TagTracker` locks. |
| Done check | `DriveToPose.isFinished`: 2 cm, 2°, 0.05 m/s. |
