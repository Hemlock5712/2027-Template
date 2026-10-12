# Video 2: "Fifty decisions a second" (driving to a point: `DriveToPose`)

A long-form lesson in 25 chapters (about 22 minutes of narration, about 26 with pauses) on how
[`DriveToPose`](../../../../src/main/java/frc/robot/commands/DriveToPose.java) drives the robot to a
field pose fast and stops exactly on it: within 2 cm and 2°, from any starting speed, even when the
goal moves. It walks the series path from [../STYLE.md](../STYLE.md): **Part 1** the goal (and why not
a timer or a timed path), **Part 2** the idea in plain words (four questions every 20 ms, three speeds
and pick the smallest, braking early, turning, corners, moving goals, the finish line), **Part 3** the
maths derived from pictures with this robot's numbers (stopping distance, reaction time, the gentle
line, the P-only race, other teams' rules, heading, the corner rule, margins), **Part 4** the real
Java in file order with one traced example in ghost values, and **Part 5** a recap with check-yourself
questions and a "try it". Video 1 covers the physics it hands off to
([`AccelerationLimiter`](../../../../src/main/java/frc/robot/utils/AccelerationLimiter.java)); video 3
covers how the tag goal is measured ([`DriveToTag`](../../../../src/main/java/frc/robot/commands/DriveToTag.java)).

Each chapter is its own animate piece with its own voice-over file. In the animate project, run:

```
/animate chapter 1 of the explainer in brief.md, use ch01-voice.json as the voice-over
```

Then chapter 2 with `ch02-voice.json`, and so on to chapter 25 with `ch25-voice.json`. Render
chapters one at a time and join them.

- **Beats match narration lines one to one.** With a voice-over, animate times each beat to its
  narration line; the times in the tables assume about 3 words a second plus a 0.6 s gap.
- **Voice files** are generated from the numbered lists (format in [../README.md](../README.md):
  ids `c01-1`, `c01-2`, …). In chapter 24, give every line that starts with "Question" `"after": 4.0`
  so the pause card can hold. The `ch1`–`ch6` JSON files in this folder belong to the old short
  version; regenerate them from this brief.
- **Ghost values** in Code mode come from the traced example below; the other numbers on screen come
  from the "Numbers on screen" table.

## Intake answers (same for every chapter)

| Question | Answer |
| --- | --- |
| **Subject** | How `DriveToPose` decides, every 20 ms, which way and how fast to drive and turn: the speed rule (three speeds, pick the smallest), braking early for delay, the gentle line, turning, the corner rule, moving goals and the finish check; derived with real numbers, then read line by line in the real Java. |
| **The one true thing** | There is no plan or clock: every 20 ms the robot asks "how far is the goal, and what's the fastest speed I can still stop from?", so it can start at any speed and the goal is allowed to move. |
| **Must include** | The four questions; v = √(2·a·d) derived from the speed-time triangle; reaction time; the gentle line and where it crosses (⅓ m); the P-only race and *why* P alone brakes late (d = v ÷ P vs v² ÷ 2a); heading with the same three ideas; the corner rule with its drawing (R = r ÷ tan(turn ÷ 2)); the done check; every line of `execute()` with one traced example in ghost values; other teams' simpler rules, fairly. |
| **Must avoid** | Claiming robot-tested results (simulation only so far: say "in our simulator"). Calling other teams' code "wrong" (say "a simpler rule"). Re-teaching video 1's physics in depth (one short chapter of numbers only). Long equations on screen at once: build each one from its picture. |
| **Formats and length** | 16:9 first, 9:16 second. 25 chapters of about 1–2 min: about 22 min of narration (3,999 words at 3 words a second), about 26 min with pauses. |
| **Voice** | Narrated voice-over, one JSON per chapter (`chNN-voice.json`). |
| **Look** | `"style": "field-lab"`, the custom style in [../STYLE.md](../STYLE.md): World (isometric field and robot), Board (maths on the navy ground), Code (editor panel, real line numbers, ghost values). Mode changes are camera moves, never cuts. Use the colour code: distance green, velocity blue, acceleration orange, heading purple, force red, time yellow, current teal, tag white with a team-colour outline. |
| **Hero** | The series robot: square frame, four swerve modules, team-colour bumpers, two corner cameras with lens LEDs. TODO: team colours and number. Same model as videos 0, 1 and 3. |
| **Music** | Composed in code. A steady tick motif (the 20 ms loop) under Parts 1–2; calm and curious under the maths with a small sting at each result; a race theme for chapter 09; quiet and sparse under the code; a resolve at the recap. |
| **Story format** | F4 mission for Parts 1–2 (the robot wants to stop exactly on a mark). F1 machine for the four-question loop (chapter 03). F5 fixed hero for Parts 3–4: the robot stays in frame (as an inset in Code mode) while graphs and code change around it. Part 5 replays the whole drive as a recap. |

## Numbers on screen (from `DriveToPose` and this robot)

| Thing | Value | Constant or source |
| --- | --- | --- |
| Loop rate | 50 times a second (every 20 ms) | robot loop |
| Finish tolerances | 2 cm, 2° | `positionTolerance`, `headingTolerance` |
| "Stopped" | measured speed under 0.05 m/s (5 cm/s) | `SETTLED_SPEED` |
| Top speed | 4 m/s (about 88% of the 4.53 m/s free speed) | `MAX_SPEED` |
| Planned braking | 6 m/s² | `DECEL` |
| Stopping from 4 m/s | 1.33 m, 0.67 s | 4² ÷ (2 × 6); 4 ÷ 6 |
| Fastest speed you can still stop from | 1 m: 3.46 m/s · 0.5 m: 2.45 m/s | √(2 × 6 × d) |
| Brake early by | 0.05 s (0.2 m at 4 m/s; braking starts 1.53 m out) | `REACTION_TIME` |
| Gentle line | speed = 6 × distance; wins inside ⅓ m (2 m/s there) | `KP` |
| At 1 mm out | curve 0.11 m/s, line 0.006 m/s | √(12 × 0.001), 6 × 0.001 |
| "Close enough, stop" | 3 mm | `STOP_DEADBAND` |
| P-only race (sim) | P 5 · limit 3: 205 cm overshoot (worst 5%) · P 5 · limit 10: 20 cm · P 2: 0.1 cm but 2.20 s vs 1.39 s to 5 cm | controllers study §7 |
| P-only braking point | d = v ÷ P: 0.8 m for P 5 at 4 m/s; needs 2.67 m at 3 m/s², 0.8 m at 10 m/s² | arithmetic |
| Turning | max 6 rad/s, brakes at 6 rad/s², gentle × 5, brakes 0.05 s early, stops within 0.3° | `MAX_OMEGA`, `ANGULAR_DECEL`, `KP_HEADING`, `HEADING_REACTION_TIME`, `HEADING_DEADBAND` |
| Stopping a turn from 6 rad/s | 3 rad (about 172°) | 6² ÷ (2 × 6) |
| Heading gentle line wins | inside 0.48 rad (about 27°) | 12 ÷ 25 |
| Limiter (video 1) | grip 10 m/s², turning 8 rad/s², 150 A per motor model | `TRACTION_ACCEL`, `ANGULAR_ACCEL`, `MODEL_CURRENT` |
| Switch radius | 0.4 m, or 40% of the shorter leg if smaller | `MAX_SWITCH_RADIUS` |
| Corner speeds (r = 0.4 m) | 45°: 2.41 m/s · 90°: 1.55 m/s · 135°: 1.00 m/s; under about 17°: 4 m/s | √(6 × 0.4 ÷ tan(turn ÷ 2)) |
| Corner rule vs stop-and-go (sim) | 11–26% faster; within 13–21 cm of the drawn lines | controllers study §3 |
| Waypoint finishes | inside max(switch radius, 5 cm) | `isFinished` |
| Margins (sim) | plan 4 with limit 3: 67 cm overshoot | controllers study §5 |

**The traced example** (Part 3 chapter 11, Part 4 chapters 18–21): the robot is at (0, 0, 0°), the goal
is (3.0, 0, 90°), last loop's command was (2.0 m/s, 0, 0 rad/s), dt = 0.02 s, a stopping leg
(`endSpeed` 0, `switchRadius` 0).

| Line | Quantity | Ghost value |
| --- | --- | --- |
| 161 | `dt` | 0.02 |
| 164 | `pose` | (0.00, 0.00, 0°) |
| 165 | `goal` | (3.00, 0.00, 90°) |
| 166 | `toGoal` | (3.00, 0.00) |
| 167 | `distance` | 3.00 |
| 168 | `headingError` | 1.571 (90°) |
| 172 | `speedNow` | 2.00 |
| 173 | `brakingDistance` | 2.90 |
| 174–175 | `speed` | min(4.0, √34.8 = 5.90) = 4.00 |
| 177 | `speed` | min(4.00, 18.0) = 4.00 |
| 182–183 | `vx`, `vy` | 4.00, 0.00 |
| 185–186 | `brakingAngle` | 1.571 |
| 187–192 | `turnSpeed` | min(6, 4.342, 7.854) = 4.342 |
| 193–194 | `omega` | +4.342 |
| 197 | `commanded` (after `AccelerationLimiter.limit`) | (2.199, 0.000, 0.060) |
| 214–216 | `isFinished()` | false (3.00 m away) |

Inside `limit` for that loop: wanted change ax = 100 m/s², spin change 217 rad/s² capped to 8; motors
at 2 m/s can add 26.3 m/s² (150 A each), so ax = 26.3; grip: √(26.3² + (8 × 0.359)²) = 26.46 > 10, so
everything scales by 0.378, giving ax = 9.94 m/s², alpha = 3.02 rad/s²; × 0.02 s gives +0.199 m/s and
+0.060 rad/s.

**The traced corner** (chapters 12, 13, 22, 23): start (0, 0), waypoint (2, 0), next (2, 1.5), which are the
template auto's two legs with the first made pass-through. Cross 3, dot 0, turn 90°; shorter leg
1.5 m × 0.4 = 0.6 → r = 0.4 m; end speed √(6 × 0.4 ÷ tan 45°) = √2.4 = 1.55 m/s.

# Part 1: The goal

## Chapter 01: The mission (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | World | WIDE: the whole field in 2:1 isometric on the navy ground, carpet sheen moving. The hero robot sits near the blue wall, modules twitching. A white mark (ring and crosshair) glows 3 m ahead; a green ruler draws between them. | **3 m** | The camera glides down toward the robot. | Soft pad; the tick motif starts quietly. |
| 2 | 6–10 s | World | A ghost of the robot slides to the mark, swings to face a purple arrow, and stops; a check pops. | **DRIVE · FACE · STOP** | The ghost fades; the camera pushes into the mark. | Three soft clicks. |
| 3 | 10–15 s | World | EXTREME CLOSE-UP: the crosshair. A green 2 cm circle and a purple 2° wedge draw on; a cartoon thumb slides in beside the circle for scale. | **2 cm · 2°** | Pull back to the robot. | Tiny "tink". |
| 4 | 15–18 s | World | A yellow autonomous clock appears in the corner and ticks down; a scoring icon greys out each second it waits. | **AND FAST** | The clock shrinks into the corner. | Clock ticks. |
| 5 | 19–22 s | World | The robot is already rolling: its blue velocity arrow points sideways, then swings to point away from the mark. | **ANY STARTING SPEED** | The mark starts to slide. | Whoosh. |
| 6 | 23–28 s | World | A white AprilTag (team-colour outline) on a field element nudges the mark 10 cm; the robot's blue arrow re-aims. | **THE GOAL MAY MOVE** | The camera pushes into the robot's "brain". | Shimmer. |
| 7 | 28–32 s | Code | The code panel unfolds out of the robot: file tab `DriveToPose.java`, line 34 highlighted (`public class DriveToPose ...`), gutter visible. | **DriveToPose** | The panel folds back into the robot. | Paper unfold. |
| 8 | 32–38 s | World | A pin on the field with green x and y rulers from the blue corner origin and a purple heading arrow. | **POSE = (x, y, θ)** | The floor tilts flat into the board. | Chime. |
| 9 | 39–44 s | Board | Five stepping stones draw on: GOAL · IDEA · MATHS · CODE · RECAP. The first three light in turn. | **1 GOAL · 2 IDEA · 3 MATHS** | The fourth stone shows a line-number gutter. | Three rising notes. |
| 10 | 45–50 s | Board | Stones 4 and 5 light; the robot icon hops onto stone 1. | **4 CODE · 5 RECAP** | A stopwatch rolls in (chapter 02). | Resolve chord. |

**Narration (ch01-voice.json):**

1. Here's our robot, and a mark on the field, three meters away.
2. The job: drive to the mark, face the right way, and stop.
3. Stop exactly: within two centimeters, about a thumb's width, and within two degrees.
4. And fast, because every second of autonomous counts.
5. It might already be moving: sideways, or even the wrong way.
6. And the mark might move, as the robot learns where the target really is.
7. One piece of code handles all of this: Drive To Pose.
8. A pose is a spot on the field plus a direction to face: x, y, and heading.
9. We'll go in order: the goal, the idea in plain words, then the maths with real numbers.
10. Then the real Java, line by line, until you know why every line is there.

## Chapter 02: Why not a timer, or a timed path? (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–8 s | World | A "timer robot" with a yellow stopwatch on its roof drives at 2 m/s; a green ruler counts 0 to 3 m as the watch counts to 1.5 s. | **2 m/s × 1.5 s = 3 m** | The battery icon starts to drain. | Stopwatch tick. |
| 2 | 8–13 s | World | Three replays: low battery (it stops short), a spinning wheel with a smoke puff (it stops short), a bump from another robot (it ends off to the side). Each time the watch says "done". | **THE TIMER NEVER NOTICES** | A timetable unrolls over the field. | Three "boing"s. |
| 3 | 14–19 s | Board | A train timetable: "0.5 s: here", "1.0 s: here", "1.5 s: here"; on the field below, yellow ghost dots along a path with time labels. | **A TIMED PATH = A TIMETABLE** | The ghost dots start to move. | Station chime. |
| 4 | 20–24 s | World | WIDE: a long curvy route across the field; the robot follows its yellow ghost dots smoothly. | **GREAT FOR LONG ROUTES** | A wheel slips. | Smooth whirr. |
| 5 | 24–30 s | World | The robot slips and falls behind; its yellow ghost dot keeps going without it. Then the goal slides and the path's end is stranded; the path crumples. | **BEHIND? THE PLAN KEEPS GOING · GOAL MOVED? START OVER** | The crumpled path blows away. | Paper crumple. |
| 6 | 31–38 s | World | Our robot, no clock. Two thought bubbles refresh with each tick: a robot outline ("where am I?") and a pin ("where is the goal?"). | **NO CLOCK: TWO QUESTIONS** | A phone slides in. | Tick motif. |
| 7 | 38–42 s | World | EXTREME CLOSE-UP: a phone map. The blue dot steps off the route and the route line instantly redraws from the dot. | **RE-ROUTE FROM WHERE YOU ARE** | Pull back to the field. | Map "re-route" ding. |
| 8 | 42–45 s | Board | Two columns: TIME-BASED (clock icon) and POSITION-BASED (pin icon); the pin column glows. | **POSITION-BASED** | The floor rises back into the world. | Chime. |
| 9 | 45–50 s | World | A push, a slip, a sliding mark: each time the blue arrow re-aims from where the robot really is. | **EVERY LOOP: A FRESH START** | Zoom into the robot's loop (chapter 03). | Three clean clicks. |

**Narration (ch02-voice.json):**

1. The simplest idea is a timer: drive at two meters per second for one and a half seconds. Three meters.
2. But with a weak battery, a slipping wheel, or a bump, the timer never notices.
3. A timed path is smarter: where to be at every moment, like a train timetable.
4. They're great for long, planned routes. But they run on a clock.
5. If the robot falls behind, the plan keeps going. If the goal moves, the plan must start over.
6. Drive To Pose has no clock. Every loop it asks: where am I now, and where is the goal now?
7. Like a phone map that re-routes from wherever you really are.
8. We call this position-based, instead of time-based.
9. So a push, a slip, or a moving goal is just the next starting point.

# Part 2: The idea

## Chapter 03: Four questions, fifty times a second (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–6 s | World | WIDE: the field. A yellow stopwatch over the robot ticks 20 ms at a time; a counter runs 1 to 50 in one second. | **50 TIMES A SECOND · EVERY 20 ms** | Four cards fly in and orbit the robot. | Tick motif at a steady pulse. |
| 2 | 7–9 s | Board | F1 machine: four numbered cards on a loop track around a robot icon; a token runs round the track. | **4 QUESTIONS** | Card 1 lights. | Machine hum. |
| 3 | 10–15 s | World | Card 1, a white pin: "Where is the goal?" On the field, the pin twitches a little each tick. | **1 · WHERE IS THE GOAL?** | Card 2 lights. | Note 1. |
| 4 | 16–20 s | World | Card 2, a green ruler: a straight green arrow grows from the robot to the goal, labelled d. | **2 · HOW FAR? d** | Card 3 lights. | Note 2. |
| 5 | 20–24 s | World | Card 3, a blue speedometer with a "?" spinning on its needle. | **3 · HOW FAST?** | Card 4 lights. | Note 3. |
| 6 | 25–29 s | World | Card 4, an orange padlock stamped "video 1"; a spring holds the speedometer needle back. | **4 · HOW MUCH CHANGE?** | The four cards fold together. | Note 4. |
| 7 | 30–34 s | World | The cards fold into one blue arrow leaving the robot's centre. | **→ ONE VELOCITY ARROW** | The arrow flashes and resets. | Whoosh. |
| 8 | 35–38 s | World | The cards blank and refill each tick; old answers drop into a small bin. | **THROW AWAY · ASK AGAIN** | Push into the robot's brain. | Tick. |
| 9 | 39–43 s | World | EXTREME CLOSE-UP: inside the brain, one memory slot labelled "last command" holds a blue arrow; every other shelf is empty. | **ONLY MEMORY: THE LAST COMMAND** | Pull out to the field. | Soft "thunk". |
| 10 | 44–49 s | World | The robot drives to the mark: the blue arrow is long far away, shrinks as it closes, and vanishes on the mark. | **LONG · SHORT · GONE** | Zoom into the speedometer card (chapter 04). | The tick slows and stops. |

**Narration (ch03-voice.json):**

1. The robot's code runs in a loop, fifty times a second: once every twenty milliseconds.
2. Each time, Drive To Pose asks four questions.
3. One: where is the goal? It asks every loop, because the goal may have moved.
4. Two: how far is it? A straight arrow from robot to goal.
5. Three: how fast should I go? The heart of this video.
6. Four: how much may my velocity change this loop? That's physics, from video one.
7. The answers become one arrow: the velocity the robot is told to drive.
8. Twenty milliseconds later, it throws them away and asks again.
9. No plan is stored. Its only memory is the last velocity it asked for.
10. Far away the arrow is long. Close in, it shrinks. On the mark, it's gone.

## Chapter 04: How fast? Three speeds, pick the smallest (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | World | WIDE: a simple slab-style car on a road approaches a white stop line. | **HOW FAST?** | Three road signs pop up. | Engine hum. |
| 2 | 6–11 s | World | Three signs by the road: a speed-limit sign; "can I stop?" with brake marks; a snail sign right at the line. | **LIMIT · CAN I STOP? · CREEP** | The car morphs into our robot, the road into the field. | Three pops. |
| 3 | 12–15 s | World | Morph: car to robot, road to field, stop line to the mark. Three blue gauges appear above the path. | **THE SAME THREE** | Gauge 1 lights. | Morph swoosh. |
| 4 | 15–18 s | World | Gauge 1's needle sits at 4 m/s; a flat blue line runs along the whole path. | **1 · TOP SPEED: 4 m/s** | Gauge 2 lights. | Gauge chime. |
| 5 | 19–23 s | World | Gauge 2: a blue braking ramp draws backwards from the mark, tall far away, zero at the mark. | **2 · CAN I STILL STOP?** | Gauge 3 lights. | Rising then falling tone. |
| 6 | 24–30 s | World | Gauge 3: a straight blue line rising from the mark; it only dips under the ramp in the last ⅓ m (green bracket). | **3 · GENTLE: KP × d** | All three overlay. | Soft glide. |
| 7 | 30–33 s | World | The three overlay; at every point the lowest glows and the others dim. | **PICK THE SMALLEST** | The robot launches. | Chord. |
| 8 | 33–36 s | World | The robot cruises; gauge 1 glows; a blue speed trace draws flat. | **CRUISE** | The ramp approaches. | Steady tick. |
| 9 | 37–42 s | World | The robot reaches the ramp; gauge 2 glows; the speed trace curves down. | **BRAKE** | The line approaches. | Falling tone. |
| 10 | 42–47 s | World | The last stretch: gauge 3 glows; the robot glides onto the mark. | **GLIDE** | A driver's foot appears. | Soft settle. |
| 11 | 47–51 s | World | A cartoon foot over a brake pedal; a yellow timer shows a small gap before it presses; the car rolls on meanwhile. | **REACTION TIME** | Push into the robot's brain. | Clock tick. |
| 12 | 52–57 s | World | Inside the robot: a position reading with an old yellow timestamp; a teal packet crawls along a CAN wire to a motor. | **OLD READINGS · SLOW WIRES** | Pull back to the ramp. | Packet blips. |
| 13 | 58–61 s | World | The braking ramp slides back by a small yellow gap labelled 0.05 s. | **BRAKE 0.05 s EARLY** | Push into the mark. | Click. |
| 14 | 62–66 s | World | EXTREME CLOSE-UP: the robot's centre a few millimetres from the crosshair; a green 3 mm ring next to two stacked pennies; the blue arrow snaps to zero. | **INSIDE 3 mm: ZERO** | The purple heading arrow lights (chapter 05). | Settle note. |

**Narration (ch04-voice.json):**

1. Picture a driver nearing a stop line. How fast should they go?
2. They weigh three things: the speed limit, "can I still stop?", and creeping in at the end.
3. Our robot asks the same three, every loop.
4. Speed one: top speed, four meters per second.
5. Speed two: the fastest speed it can still stop from, in the distance left.
6. Speed three: gentle, a number times the distance. It only matters in the last third of a meter.
7. Then take the smallest of the three.
8. Far away, top speed is smallest, so it cruises.
9. Then "can I still stop" takes over, and it brakes along a smooth curve.
10. At the very end, the gentle line takes over, and it glides in.
11. One more trick. A driver needs a moment to react before braking.
12. So does our robot: its readings are slightly old, and commands take time to reach the motors.
13. So it brakes a twentieth of a second early.
14. And within three millimeters, about two stacked pennies, it asks for zero.

## Chapter 05: Turning, and corners without stopping (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–6 s | World | The robot on the mark but facing wrong: its purple heading arrow and a purple target arrow 90° apart. | **FACE THE RIGHT WAY** | Three purple dials appear. | Chime. |
| 2 | 6–12 s | World | Three purple dials over the robot: top spin; "can I stop spinning?"; gentle near the end. | **TOP SPIN · CAN I STOP? · GENTLE** | The heading arrow starts to swing. | Three gauge chimes, lower pitch. |
| 3 | 13–18 s | World | EXTREME CLOSE-UP: the heading arrow's tip slows early (a yellow tick) and settles inside a 0.3° purple wedge. | **INSIDE 0.3°: STOP TURNING** | Pull back. | Settle note. |
| 4 | 18–24 s | World | Two arcs: the long way round (350°) dashed and crossed out, the short way (10°) bright. Then the robot drives and turns at once onto the mark. | **SHORT WAY · DRIVE AND TURN TOGETHER** | WIDE pull-back to a route. | Smooth whirr. |
| 5 | 25–31 s | World | WIDE: a route with one corner. A robot drives to the corner, stops, then sets off on the next leg. | **STOP-AND-GO** | It repeats on a second corner. | Stuttering beat. |
| 6 | 31–34 s | World | A yellow clock adds lost time at each stop. | **STOP · GO · STOP · GO** | Rewind. | Rewind zip. |
| 7 | 34–39 s | World | Our robot rounds the corner on a smooth curve without stopping. | **PASS THROUGH** | A circle draws at the corner. | Swoosh. |
| 8 | 40–44 s | World | A green circle around the corner, radius 0.4 m; a baton passes from leg 1 to leg 2 as the robot crosses it. | **HAND OVER, AT MOST 0.4 m OUT** | Three corners appear. | Baton click. |
| 9 | 45–50 s | World | A car takes three bends: gentle (fast), right angle, hairpin (slow); its blue speed arrow shrinks bend by bend. | **SHARPER → SLOWER** | Arcs glow inside each corner. | Three notes, falling. |
| 10 | 50–54 s | World | An arc fits snugly inside each corner; a "Part 3" stamp lands on them. | **THE FASTEST CURVE THAT FITS** | A pin starts to slide (chapter 06). | Sting. |

**Narration (ch05-voice.json):**

1. The robot must also face the right way. Turning uses the same three ideas.
2. A top spin rate, the fastest spin it can still stop from, and gentle near the end.
3. It brakes its turn early too, and stops turning within a third of a degree.
4. It turns the short way, and since it's a swerve robot, it drives and turns at once.
5. Now a route with a corner. The simple way: stop at the corner, then start the next leg.
6. Stop, go, stop, go. Precise, but slow.
7. Drive To Pose can pass through a waypoint instead, handing over just before the corner.
8. The hand-over happens on a circle at most forty centimeters from the corner.
9. How fast through the corner? Like a car: a gentle bend quickly, a hairpin slowly.
10. It picks the fastest speed whose curve fits inside the corner.

## Chapter 06: A goal that moves, and when it's done (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–6 s | World | WIDE: the field. The pin slides slowly while the robot drives; every tick the blue arrow re-aims at it. | **ASKED EVERY LOOP** | The pin becomes a field element with a tag. | Ticks. |
| 2 | 6–12 s | World | A field element with a white AprilTag (team-colour outline). A dashed "map" copy of it sits 5 cm off; the robot aims at the dashed one. | **DRIVETOTAG: THE MAP FIRST** | The camera LEDs pulse. | Low hum. |
| 3 | 12–19 s | World | EXTREME CLOSE-UP: a camera's lens LED pulses; white brackets lock onto the tag's corners; the goal jumps to the real spot. A "video 3" tag hangs on it. | **THEN THE TAG ITSELF** | The tag dissolves into a game piece. | Lock click. |
| 4 | 20–25 s | World | `DriveToPiece`: a game piece rolls; the goal, a spot just short of it, slides with it; the robot follows. | **DRIVETOPIECE: FOLLOW THE PIECE** | The floor tilts into the board. | Rolling sound. |
| 5 | 26–31 s | Board | The four-question loop again; only card 1 changes face: map, tag, piece. | **SAME LOOP · NEW GOAL** | Three checkboxes appear. | Card flips. |
| 6 | 31–35 s | World | Three empty checkboxes beside the robot near the mark. | **DONE?** | They tick one by one. | Pen scratch. |
| 7 | 35–41 s | World | Ticks: a green 2 cm circle; a purple 2° wedge; a blue speedometer under 5 cm/s, labelled "measured". | **2 cm · 2° · UNDER 5 cm/s** | A car pulls into a parking space. | Three ticks. |
| 8 | 41–45 s | World | The car is still rolling in the space: a red cross on "parked?"; it stops: green tick. | **STOPPED = PARKED** | Back to the corner circle. | Handbrake click. |
| 9 | 46–49 s | World | A waypoint robot crosses its green hand-over circle still moving; a tick appears as it crosses. | **WAYPOINT: DONE AT THE CIRCLE** | The floor tilts flat into the board (Part 3). | Sting. |

**Narration (ch06-voice.json):**

1. Where is the goal? It's asked every loop, so the goal may move.
2. Drive To Tag uses this. Far away, it drives to the scoring spot on the field map.
3. Once the cameras lock on to the AprilTag, the goal jumps to the spot measured from the tag. Video three.
4. Drive To Piece chases a game piece the camera sees. If the piece rolls, the robot follows.
5. Same loop, same speed rule. Only the answer to "where is the goal" changes.
6. So when is the job done? Three checks, all at once.
7. Within two centimeters, within two degrees, and truly stopped: measured speed under five centimeters per second.
8. Like parking a car. You're not parked until the car stops.
9. A pass-through waypoint is done at its hand-over circle, still rolling.

# Part 3: The maths

## Chapter 07: Stopping distance, from a triangle (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | World | WIDE: the robot brakes to a stop on the field; then the floor tilts flat and its edges become axes. | **HOW FAR TO STOP?** | Axes draw on. | Pen-draw sound. |
| 2 | 5–9 s | Board | Axes: speed up the side (blue), time along the bottom (yellow). The field stays faintly visible below. | **speed ↑ · time →** | A dot appears on the speed axis. | Soft note. |
| 3 | 9–13 s | Board | A blue dot at 4 on the speed axis, a small robot icon beside it. | **v = 4 m/s** | An orange slope triangle appears. | Pluck. |
| 4 | 14–20 s | Board | An orange slope triangle: down 6 for every 1 s across. | **a = 6 m/s²** | The line draws down. | Pluck. |
| 5 | 21–29 s | Board | A straight blue line from 4 down to 0; the yellow time span under it is labelled. A live readout counts the speed down. | **t = v ÷ a = 4 ÷ 6 = 0.67 s** | The area under the line fills. | Falling tone. |
| 6 | 29–35 s | Board | The area under the line fills green; a "distance" readout counts up as it fills. | **AREA = DISTANCE** | A rectangle example slides in. | Fill shimmer. |
| 7 | 35–41 s | Board | A rectangle, 2 m/s tall and 3 s wide, fills to 6 m; then our triangle is shown as half of a rectangle. | **2 × 3 = 6 m · TRIANGLE = ½ × height × width** | Symbols slide together. | Two notes. |
| 8 | 42–48 s | Board | ½ · v · (v ÷ a) collapses into one fraction. EXTREME CLOSE-UP: the small "²" writes on. | **d = v² ÷ (2a)** | Numbers drop in. | Written-on scratch. |
| 9 | 49–53 s | Board | 16 ÷ 12 = 1.33; below, on the faint field, a green 1.33 m bracket ends at the mark. | **4² ÷ (2 × 6) = 1.33 m** | The formula turns around. | Chime. |
| 10 | 54–59 s | Board | The formula rotates: d is now given (green), v is the unknown (blue "?"). | **d LEFT → FASTEST v?** | v is solved for. | Soft note. |
| 11 | 60–64 s | Board | v = √(2 · a · d) writes on. | **v = √(2 · a · d)** | Its curve draws. | Sting. |
| 12 | 65–72 s | Board | New axes: distance left (green) across, speed (blue) up. The curve √(12d) draws; dots at 1 m (3.46) and 0.5 m (2.45); it reaches zero at the goal. | **1 m: 3.46 m/s · 0.5 m: 2.45 m/s · 0: 0** | The first triangle returns. | Three plucks. |
| 13 | 72–77 s | Board | The triangle's tip is cut off at an end speed: it becomes a trapezoid (green area). | **WAYPOINT: BRAKE TO AN END SPEED** | The rule writes on. | Soft note. |
| 14 | 78–84 s | Board | v² = v_end² + 2·a·d writes on, lifts off the board and flies to a dim preview of code line 175, then returns. | **v² = v_end² + 2 · a · d** | The robot reappears with a yellow clock (chapter 08). | Whoosh. |

**Narration (ch07-voice.json):**

1. Now the maths. How far does it take to stop?
2. Draw a graph: speed up the side, time along the bottom.
3. The robot starts at v, its speed: four meters per second.
4. It brakes at a, its deceleration: six meters per second squared. It loses six meters per second, every second.
5. So speed falls in a straight line. The time it takes, t, is v over a. Four over six: zero point six seven seconds.
6. Here's the trick: on a speed-time graph, the area under the line is the distance travelled.
7. Two meters per second for three seconds is a rectangle: six meters. A triangle is half height times width.
8. Half of v, times v over a. So d, the stopping distance, equals v squared over two a.
9. Sixteen over twelve: one point three three meters to stop from top speed.
10. Now flip it. With d meters left, what's the fastest speed I can still stop from?
11. Solve for v: v equals the square root of two times a times d.
12. One meter out: three point four six meters per second. Half a meter: two point four five. At the goal: zero.
13. A waypoint brakes to an end speed, not zero, so the triangle becomes a trapezoid.
14. Then v squared equals end speed squared, plus two a d. That's the exact rule in the code.

## Chapter 08: Brake early, then glide in (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–4 s | World | WIDE: the robot at speed on the field; a yellow clock labelled 0.05 s floats over it. | **REACTION TIME: 0.05 s** | Push into the robot. | Tick. |
| 2 | 5–11 s | World | EXTREME CLOSE-UP: inside, a position reading stamped with an old yellow time; a 20 ms gate; a teal packet crawling along a CAN wire to a motor. | **OLD READINGS · LOOP WAIT · CAN WIRES** | Pull back. | Packet blips. |
| 3 | 11–16 s | World | The robot keeps rolling during the yellow clock; a green 0.2 m bracket draws behind it. | **4 m/s × 0.05 s = 0.2 m** | The floor tilts into the board. | Soft roll. |
| 4 | 16–21 s | Board | On the distance-left axis, "d − v × 0.05" writes on; the braking curve slides away from the goal by the gap. | **d − v × 0.05** | Two markers appear. | Click. |
| 5 | 21–27 s | Board | Two green markers at 4 m/s: 1.33 m (old) and 1.53 m (new). | **1.33 + 0.2 = 1.53 m** | The gentle line enters. | Chime. |
| 6 | 28–32 s | Board | A straight blue line, speed = 6 × d, draws up from the origin. | **v = KP × d · KP = 6** | It meets the curve. | Pen draw. |
| 7 | 33–38 s | Board | Both lines; the crossing blinks; the equation writes beside it. | **6d = √(12d)** | Both sides are squared. | Soft note. |
| 8 | 39–46 s | Board | 36d² = 12d, divide by d: d = ⅓; the crossing labelled (0.33 m, 2 m/s). | **d = ⅓ m · v = 2 m/s** | The region to the left glows. | Two notes. |
| 9 | 47–50 s | Board | Left of ⅓ m, the line runs under the curve and glows; right of it, the curve glows. | **CLOSER THAN ⅓ m: THE LINE WINS** | Zoom into the origin. | Soft note. |
| 10 | 51–54 s | Board | EXTREME CLOSE-UP of the origin: the √ curve stands almost straight up. | **STEEP AT ZERO** | Readouts appear. | Low tone. |
| 11 | 55–60 s | Board | At d = 1 mm: curve 0.11 m/s, line 0.006 m/s, as two blue bars. | **1 mm: 11 cm/s vs 0.6 cm/s** | The field fades in below. | Two plucks. |
| 12 | 60–64 s | World | The measured position wiggles by a hair; with the curve alone, the robot lurches back and forth. | **WIGGLE → LURCH** | A sim card slides in. | Rattle. |
| 13 | 65–71 s | Board | Sim card: "curve only, 1 cm of noise: about 9 cm/s of jitter". | **IN OUR SIMULATOR: ~9 cm/s JITTER** | A KP slider appears. | Soft note. |
| 14 | 71–76 s | Board | A KP slider moves 6 → 8: a tiny wobble appears at the finish; back to 6: still. | **KP 8+: TINY WOBBLE (SIM) · KP 6: STILL** | Pull out to the field. | Wobble, then quiet. |
| 15 | 77–81 s | World | The robot settles on the mark; a green 3 mm ring; the blue speed readout reads 0. | **STOP_DEADBAND: 3 mm** | Two lanes draw on (chapter 09). | Settle note. |

**Narration (ch08-voice.json):**

1. Now the reaction time: zero point zero five seconds.
2. Readings are a little old, and each command waits for the loop, then travels over the CAN wires.
3. Meanwhile it keeps rolling: at four meters per second, zero point two meters.
4. So it subtracts speed times zero point zero five from the distance first.
5. From top speed, braking now starts at one point five three meters, not one point three three.
6. Next, the gentle line: speed equals K P, a gain of six, times distance.
7. Where does it cross the braking curve? Where six d equals the square root of twelve d.
8. Square both sides: thirty-six d squared equals twelve d. So d is a third of a meter, at two meters per second.
9. Closer than that, the line is lower, so it wins.
10. Why bother? Near zero, the curve is almost vertical.
11. One millimeter out, the curve asks for eleven centimeters per second. The line, under one.
12. Measured positions always wiggle. A steep curve turns every wiggle into a lurch.
13. In our simulator, with a centimeter of noise, the curve alone jittered about nine centimeters per second.
14. A K P of eight or more also wobbled at the finish. Six didn't.
15. Last: inside three millimeters, the stop deadband sets speed to exactly zero.

## Chapter 09: The race: why not just slow down when close? (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | World | WIDE: two parallel lanes, each with a mark. Lane A's robot wears a sign "speed = P × d"; lane B's robot is ours. | **A SIMPLER RULE** | A "SIM" badge drops on the scene. | Race theme starts. |
| 2 | 7–10 s | World | Countdown lights; a "1 to 6 m drives · our simulator" plate under the lanes. | **RACE · IN OUR SIMULATOR** | Lights go green. | Three beeps. |
| 3 | 11–16 s | World | Lane A's sign updates: P 5 · limit 3 m/s². Both robots launch. | **ROUND 1: P 5 · LIMIT 3** | Both reach top speed. | Race theme. |
| 4 | 16–20 s | World | Robot A sails about 2 m past its mark and creeps back; robot B stops on its mark. | **OVERSHOOT ≈ 2 m (WORST RUNS)** | Freeze; the floor tilts into the board. | Boing. |
| 5 | 21–27 s | Board | v = P × d; a marker where P × d drops below the speed; d = v ÷ P writes on. | **BRAKES WHEN P × d < v → d = v ÷ P** | Numbers drop in. | Soft note. |
| 6 | 28–31 s | Board | 4 ÷ 5 = 0.8; a green marker at 0.8 m on the distance axis. | **4 ÷ 5 = 0.8 m** | An orange bracket grows. | Pluck. |
| 7 | 32–37 s | Board | An orange bracket for the stopping distance, 16 ÷ 6 = 2.67 m, dwarfs the green 0.8 m; the gap is hatched. | **NEEDS 4² ÷ (2 × 3) = 2.67 m** | Rewind the lanes. | Error buzz. |
| 8 | 37–43 s | World | Round 2: lane A's sign reads P 5 · limit 10. Inset: "needs 16 ÷ 20 = 0.8 m · starts at 0.8 m". | **ROUND 2: P 5 · LIMIT 10** | Both launch. | Race theme. |
| 9 | 44–48 s | World | EXTREME CLOSE-UP: robot A's front bumper slides 20 cm past the line. | **ZERO MARGIN · 20 cm OVER** | Rewind. | Small boing. |
| 10 | 49–54 s | World | Round 3: P 2. A green marker shows braking starting 2 m out; robot A stops cleanly. | **ROUND 3: P 2 · BRAKES AT 2 m** | Robot A's last stretch slows. | Clean click. |
| 11 | 55–58 s | World | Robot A creeps the last stretch; its speed trace flattens toward zero; a snail icon. | **CREEPS IN** | Two timers appear. | Slow ticking. |
| 12 | 58–63 s | World | Two timers to "within 5 cm": A 2.20 s, B 1.39 s. | **2.20 s vs 1.39 s** | The floor tilts into the board. | Clock tick. |
| 13 | 64–68 s | Board | A see-saw: "FAST + OVERSHOOT" on one end, "SAFE + SLOW" on the other. | **P ALONE MUST CHOOSE** | Our curve draws. | See-saw creak. |
| 14 | 69–75 s | Board | Our braking curve; a green marker at 1.53 m where braking starts from 4 m/s. | **OURS: BRAKES AT 1.53 m** | Orange bars rise. | Pluck. |
| 15 | 75–81 s | Board | Orange bars: plan 6 under limit 10; the last ⅓ m highlighted on the gentle line. | **PLAN 6 · LIMIT 10 · GENTLE LAST ⅓ m** | Back to the lanes. | Soft note. |
| 16 | 82–86 s | World | Lane B's robot finishes first, dead on its mark; a "0 cm overshoot" tag. | **FAST AND EXACT** | Other robots roll in (chapter 10). | Win sting. |

**Narration (ch09-voice.json):**

1. Here's a simpler rule: speed equals P, just a number, times the distance, capped at top speed.
2. Let's race it against ours, in our simulator.
3. Round one: P is five, with an acceleration limit of three meters per second squared.
4. The simple robot overshoots. In the worst runs, by about two meters.
5. Why? It only slows once P times d drops below its speed: at d equals v over P.
6. Four over five: it starts braking zero point eight meters out.
7. But stopping from four at three needs sixteen over six: two point six seven meters.
8. Round two: a limit of ten. Stopping takes zero point eight meters, exactly where it starts braking.
9. Zero margin. Any delay, and it slides past: twenty centimeters, in our simulator.
10. Round three: a gentle P of two. It brakes from two meters out and stops cleanly.
11. But its speed shrinks with distance, so it creeps in.
12. It reached five centimeters in two point two zero seconds. Ours: one point three nine.
13. So P alone must choose: fast and overshoot, or safe and slow.
14. Ours asks "can I still stop?", so from top speed it brakes at one point five three meters.
15. It plans six while the limit allows ten; the gentle line takes the last third of a meter.
16. Full speed for longer, no overshoot. All from one square root.

## Chapter 10: Other ways teams do this (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | World | WIDE: several generic robots (plain bumpers, no team logos) each driving to a mark, each with a small "P × d" tag. | **OTHER WAYS** | A card flips up. | Friendly theme. |
| 2 | 6–11 s | Board | Card "B-Line": a polyline path; the green distance left is measured along the path; speed = P × that. | **B-LINE: P × PATH LEFT** | A robot drifts off the line. | Card flip. |
| 3 | 12–17 s | World | A robot drifts sideways; a small arrow pushes it back to the line; a dial limits how fast its blue arrow changes. | **+ PUSH BACK TO THE LINE · + RATE LIMIT** | An editor window opens. | Soft note. |
| 4 | 18–23 s | World | An editor window with a many-point path; the robot runs it, its purple heading arrow turning smoothly along. | **GAINS: MANY POINTS · EDITOR · SMOOTH HEADING** | A tag slides; the path stays put. | Pluck. |
| 5 | 23–30 s | World | A tag goal slides; the drawn path stays frozen with a padlock "fixed for this run"; P's slider is held low. | **GIVES UP: MOVING GOALS · P STAYS GENTLE** | A second card flips up. | Lock click. |
| 6 | 30–35 s | Board | Card "2056 OP Robotics": speed = P × d, with a cap. | **2056: P × d, CAPPED** | Push into its camera. | Card flip. |
| 7 | 35–41 s | World | EXTREME CLOSE-UP: a camera view of the scoring spot; the end-point marker slides to the measured spot. | **END POINT FROM THE CAMERA** | Two columns draw. | Lock click. |
| 8 | 42–46 s | Board | Two columns: GAINS (simple, a few lines, tune by hand) · GIVES UP (brake late or creep in). | **SIMPLE · TUNED PER ROBOT** | A stamp lands. | Soft note. |
| 9 | 47–51 s | Board | A stamp: "our simulator copies, not their robots". | **OUR COPIES, NOT THEIR ROBOTS** | Our formula slides in. | Stamp thump. |
| 10 | 51–55 s | Board | Our speed rule with the √ term glowing as "+ 1 line". | **SAME IDEA + STOPPING DISTANCE** | The board rotates to a heading dial (chapter 11). | Chime. |

**Narration (ch10-voice.json):**

1. Many good teams use P on distance, carefully tuned. Here are two.
2. B-Line is a small, student-run path library. Speed is P times the distance left along the path.
3. It adds a push back toward the line, and a limit on how fast speed changes.
4. It gains many-point paths from an editor, with heading turning smoothly along the way.
5. It gives up moving goals, since the path is fixed once it starts. And its P must stay gentle.
6. Team twenty fifty-six, OP Robotics, uses P on distance with a speed cap.
7. To line up, they measure the scoring spot from the robot and move the end point to match.
8. It gains simplicity. It gives up the same choice: brake late, or creep in.
9. We tested simple copies of these rules, not their real robots.
10. Ours keeps the same position-based idea, plus one line: the stopping distance.

## Chapter 11: Turning, by the numbers (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–6 s | Board | Two columns swap words: distance d (green) becomes angle θ (purple); speed v (blue) becomes spin ω (purple). | **d → θ · v → ω** | A circle draws round the robot. | Soft note. |
| 2 | 6–12 s | World | A purple circle round the robot, −π and +π meeting behind it; the short arc to the goal heading glows, the long arc is dashed. | **−π TO π: THE SHORT WAY** | The floor tilts into the board. | Pluck. |
| 3 | 12–19 s | Board | A dial at 6 rad/s, nearly one turn per second; an orange-purple slope labelled α = 6 rad/s². | **ω max 6 rad/s · α 6 rad/s²** | The stopping formula writes on. | Gauge chime. |
| 4 | 19–25 s | Board | ω² ÷ 2α = 36 ÷ 12 = 3 rad; a purple wedge opens to 172°. | **3 rad ≈ 172°** | A yellow tick appears. | Pluck. |
| 5 | 25–30 s | Board | 6 × 0.05 = 0.3 rad, a yellow tick on the wedge. | **BRAKE EARLY: 0.3 rad AT TOP SPIN** | The gentle line draws. | Click. |
| 6 | 31–37 s | Board | The line 5θ and the curve √(12θ) cross at 0.48 rad; label "≈ 27°". | **CROSS AT 0.48 rad ≈ 27°** | Zoom to the arrow tip. | Two notes. |
| 7 | 38–41 s | World | EXTREME CLOSE-UP: the heading arrow's tip inside a 0.3° wedge; ω reads 0. | **UNDER 0.3°: ω = 0** | Pull back to the board. | Settle note. |
| 8 | 42–47 s | Board | The traced turn, 90° = 1.571 rad: three readouts, curve 4.34, line 7.85, cap 6. | **4.34 · 7.85 · 6** | The smallest is circled. | Three plucks. |
| 9 | 48–53 s | Board | 4.34 rad/s circled, with a small tag "code line 191". | **ω = 4.34 rad/s** | WIDE pull-back to the field. | Sting. |
| 10 | 54–57 s | World | WIDE: the robot turning on the field; two orange-purple bars: plan 6, limit 8. | **PLAN 6 < LIMIT 8** | A route with a corner draws (chapter 12). | Soft note. |

**Narration (ch11-voice.json):**

1. Turning uses the same maths. Distance becomes angle; speed becomes omega, the spin rate.
2. The heading error stays between minus pi and pi radians, so it turns the short way.
3. Top spin: six radians per second, almost a turn a second. Spin braking, alpha: six radians per second squared.
4. Stopping from top spin: omega squared over two alpha. Three radians, about one hundred seventy-two degrees.
5. Braking early at top spin: zero point zero five seconds is zero point three radians.
6. The gentle line, five times the angle, meets the curve at zero point four eight radians, about twenty-seven degrees.
7. Inside zero point three degrees, the spin command is zero.
8. For a ninety degree turn: curve four point three four, line seven point eight five, cap six.
9. Smallest: four point three four radians per second. It shows up again in the code.
10. And the margin rule: plan six, below the limiter's eight.

## Chapter 12: The corner rule (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | World | WIDE: a route on the field from (0, 0) to (2, 0), then to (2, 1.5); the 90° corner is marked. | **2 m · THEN 1.5 m LEFT · 90°** | The floor tilts into the board, the route kept. | Soft note. |
| 2 | 8–14 s | Board | The rule writes on next to the corner. | **r = min(0.4 m, 40% × shorter leg)** | Numbers drop in. | Pen draw. |
| 3 | 14–19 s | Board | 0.4 × 1.5 = 0.6; min(0.4, 0.6) = 0.4. Inset: a short 0.5 m leg gives r = 0.2. | **r = 0.4 m** | A circle draws. | Pluck. |
| 4 | 20–26 s | Board | The turning circle draws, touching leg 1 at 0.4 m before the corner and leg 2 at 0.4 m after (two green tick marks). | **TOUCHES BOTH LEGS** | A right triangle highlights. | Soft note. |
| 5 | 27–34 s | Board | Right triangle: corner, touch point, circle centre. The two radii meet at the turn angle, so the angle at the centre is half the turn; corner to touch point is r, touch point to centre is R (both green). | **tan(turn ÷ 2) = r ÷ R → R = r ÷ tan(turn ÷ 2)** | Zoom to the angle. | Two notes. |
| 6 | 35–40 s | Board | EXTREME CLOSE-UP: the "45°" angle mark at the centre; tan 45° = 1 writes on; R = 0.4 m. | **tan 45° = 1 → R = 0.4 m** | Pull back; a robot circles. | Pluck. |
| 7 | 40–45 s | World | The robot drives round the circle; an orange arrow points sideways to the centre, labelled v² ÷ R. | **SIDEWAYS ACCEL = v² ÷ R** | The floor tilts into the board. | Whirr. |
| 8 | 45–51 s | Board | v² ÷ R ≤ 6; solve: v = √(6 × R). | **v² ÷ R ≤ DECEL → v = √(DECEL × R)** | The two formulas merge. | Soft note. |
| 9 | 52–57 s | Board | R slides into the speed formula. | **v = √(DECEL × r ÷ tan(turn ÷ 2))** | Numbers drop in. | Sting. |
| 10 | 57–63 s | Board | √(6 × 0.4 ÷ 1) = √2.4 = 1.55. | **90°: 1.55 m/s** | Two more corners appear. | Pluck. |
| 11 | 63–69 s | Board | Corners side by side: 45° with a big arc (2.41), 135° with a tiny arc (1.00). | **45°: 2.41 m/s · 135°: 1.00 m/s** | A bar chart grows. | Three notes, falling. |
| 12 | 69–75 s | Board | Bars of corner speed by turn angle; every bar under about 17° is cut at 4 m/s; a "< 1°: straight" note. | **UNDER ~17°: 4 m/s · UNDER 1°: STRAIGHT** | Rise into the world. | Soft note. |
| 13 | 75–80 s | World | Two robots on a zigzag route: stop-and-go versus corner rule; yellow timers, the corner-rule robot wins. | **11–26% FASTER (SIM)** | The route lines overlay. | Race theme, short. |
| 14 | 81–85 s | World | The corner-rule path overlaid on the drawn lines, cutting each corner by a little; a green "≤ 21 cm" ruler at the worst spot. | **WITHIN ~20 cm OF THE LINES (SIM)** | Two orange bars appear (chapter 13). | Settle note. |

**Narration (ch12-voice.json):**

1. Now the corner rule, on a route: two meters ahead, then one and a half left. Ninety degrees.
2. Step one: the switch radius, r. Forty centimeters, or forty percent of the shorter leg, whichever is smaller.
3. Forty percent of one and a half is sixty centimeters, so r is forty.
4. Step two: the turning circle touches the first leg r before the corner, and the second leg r after.
5. Its radius is big R. The right triangle at the corner gives: big R equals r over tangent of half the turn.
6. Half of ninety is forty-five, whose tangent is one. So big R is forty centimeters.
7. Step three: driving around a circle takes sideways acceleration: v squared over big R.
8. Keep that within our planned braking, six. So v is the square root of six times big R.
9. Together: the square root of decel times r, over tangent of half the turn.
10. Ninety degrees: the square root of two point four. One point five five meters per second.
11. Forty-five degrees: two point four one. A one hundred thirty-five degree hairpin: one point zero zero.
12. Under about seventeen degrees, it's capped at top speed. Under one degree, the code calls it straight.
13. In our simulator, routes with this rule were eleven to twenty-six percent faster than stop-and-go.
14. And they stayed within about twenty centimeters of the drawn lines.

## Chapter 13: Margins and the finish line (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–3 s | Board | Title card on the board. | **MARGINS · FINISH LINE** | Two orange bars rise. | Soft note. |
| 2 | 3–9 s | Board | Orange bars: "plan 6" and "limiter 10" (video 1); the gap is labelled "margin". | **PLAN 6 · LIMIT 10** | A robot rolls onto the board's floor. | Pluck. |
| 3 | 9–14 s | World | A robot whose plan bar is taller than its limit bar: it brakes late and slides past its mark. | **PLAN > LIMIT → OVERSHOOT** | A sim card slides in. | Boing. |
| 4 | 14–19 s | Board | Sim card: plan 4, limit 3: 67 cm overshoot. | **67 cm OVER (SIM)** | A tag target appears. | Soft note. |
| 5 | 20–25 s | World | Two target clusters of finish dots on a tag spot: plan 9, spread out; plan 6, tight. | **PLAN 9: LESS ACCURATE · PLAN 6: ACCURATE (SIM)** | Three checkboxes. | Two plucks. |
| 6 | 26–32 s | World | Checkboxes tick: 2 cm (green), 2° (purple), under 0.05 m/s (blue). | **2 cm · 2° · < 0.05 m/s** | Push into a wheel. | Three ticks. |
| 7 | 33–37 s | World | EXTREME CLOSE-UP: a wheel's encoder ticks slowing to nothing, "measured"; a speech bubble "please stop" crossed out beside it. | **MEASURED, NOT ASKED** | Pull back to a corner circle. | Tick, tick, silence. |
| 8 | 38–42 s | World | A waypoint's finish circle: the bigger of the switch radius and 5 cm. | **WAYPOINT DONE: INSIDE max(r, 5 cm)** | WIDE pull-back. | Soft note. |
| 9 | 43–48 s | World | WIDE: the robot crosses the 0.4 m circle of our 90° corner at about 1.55 m/s; then the camera pushes into its brain and the code panel unfolds. | **DONE AT 0.4 m · ~1.55 m/s** | The code panel opens on line 1 (Part 4). | Paper unfold. |

**Narration (ch13-voice.json):**

1. Two last pieces. First, margins.
2. We plan to brake at six; video one's limiter allows ten. That gap is on purpose.
3. Plan more than the limiter allows, and the robot can't follow. It overshoots.
4. In our simulator, planning four with a limit of three overshot by sixty-seven centimeters.
5. Why not nine? In our simulator, that made tag line-ups less accurate. Six stayed accurate and quick.
6. Second, the finish line: under two centimeters, under two degrees, and under zero point zero five meters per second.
7. That's the measured speed. Asking to stop isn't the same as being stopped.
8. A waypoint finishes inside the switch radius, or five centimeters, whichever is bigger.
9. On our ninety degree corner: forty centimeters out, rolling at about one point five five.

# Part 4: The code

## Chapter 14: The top of the file (lines 1–56) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | Code | The editor panel, file tab `DriveToPose.java`; the gutter counts 1 to 245 as the file scrolls past once, then returns to the top. A small World inset (WIDE: the field, robot on its start) sits in the corner. | **DriveToPose.java · 245 LINES** | The highlight lands on lines 1–18. | Quiet keyboard texture. |
| 2 | 6–10 s | Code | Lines 1–18 highlighted, then folded into one grey bar: "licence + imports". | **1–18: LICENCE + IMPORTS** | The highlight moves to the Javadoc. | Fold click. |
| 3 | 10–14 s | Code | Lines 20–33 highlighted; the four `<li>` lines light one at a time and the four question cards from chapter 03 dock beside them. | **20–33: THE FOUR QUESTIONS** | Highlight to line 34. | Four soft notes. |
| 4 | 15–20 s | Code | Line 34; a small diagram of the four methods: initialize (once) → execute ⟲ isFinished? → end (once). | **34: extends ClassicCommand** | Highlight to lines 35–36. | Machine hum. |
| 5 | 21–26 s | Code | Lines 35–36 highlighted with a warning-tape edge; the orange "plan 6 · limit 10" bars from chapter 13 float beside. | **35–36: PLAN BELOW THE LIMIT** | Highlight to line 37. | Low warning tone. |
| 6 | 26–31 s | Code | Line 37; `MAX_SPEED` in blue, ghost `4.0 m/s`; a mini gauge at 88% of 4.53 m/s. | **37: MAX_SPEED = 4.0 m/s** | Highlight to line 38. | Gauge chime. |
| 7 | 31–35 s | Code | Line 38; `DECEL` in orange, ghost `6.0 m/s²`; its comment glows. | **38: DECEL = 6.0 m/s²** | Highlight to lines 39–42. | Pluck. |
| 8 | 36–43 s | Code | Lines 39–42; `KP` ghost `6.0`; `REACTION_TIME` in yellow, ghost `0.05 s`; the comment "0.10 with 100 Hz" glows. | **40: KP = 6 · 42: REACTION_TIME = 0.05 s** | Highlight to line 43. | Two plucks. |
| 9 | 44–46 s | Code | Line 43; `STOP_DEADBAND` in green, ghost `3 mm`. EXTREME CLOSE-UP: the digits "0.003" fill the frame. | **43: STOP_DEADBAND = 0.003 m** | Highlight to lines 45–50. | Tiny tink. |
| 10 | 46–52 s | Code | Lines 45–50, all purple: ghosts `6 rad/s`, `6 rad/s²`, `5`, `0.05 s`, `0.3°`. | **45–50: THE SAME, FOR HEADING** | Highlight to lines 52–56. | Five soft notes. |
| 11 | 53–58 s | Code | Lines 52–56; `MAX_SWITCH_RADIUS` green, ghost `0.4 m`; `SETTLED_SPEED` blue, ghost `0.05 m/s`. | **54: 0.4 m · 56: 0.05 m/s** | The colour key fades in. | Pluck. |
| 12 | 59–64 s | Code | The colour key appears beside the code: green distance, blue velocity, orange acceleration, purple heading, yellow time; matching names flash. | **SAME COLOURS AS THE BOARD** | Scroll to line 58 (chapter 15). | Chime. |

**Narration (ch14-voice.json):**

1. Now the real Java. Every line number shown is the real one.
2. Lines one to eighteen: licence and imports, the tools this file borrows.
3. Lines twenty to thirty-three, the Javadoc: our four questions, written down.
4. Line thirty-four extends Classic Command: initialize once, then execute and is finished every loop, then end.
5. Lines thirty-five and thirty-six warn: plan to brake below the limiter's limits, or it overshoots.
6. Line thirty-seven, max speed: four meters per second, about eighty-eight percent of free speed.
7. Line thirty-eight, decel: six. Braking harder costs tag accuracy, says the comment.
8. Line forty, K P: six. Line forty-two, reaction time: zero point zero five seconds; zero point one zero with slower odometry.
9. Line forty-three, stop deadband: three millimeters.
10. Lines forty-five to fifty, heading: six, six, five, zero point zero five, and zero point three degrees.
11. Line fifty-four, max switch radius: zero point four meters. Line fifty-six, settled speed: five centimeters per second.
12. Distances green, speeds blue, accelerations orange, angles purple, times yellow. Same colours as the board.

**Code on screen:**

```java
 1  // Copyright (c) FIRST and other WPILib contributors.
 2  // Open Source Software; you can modify and/or share it under the terms of
 3  // the WPILib BSD license file in the root directory of this project.
 4
 5  package frc.robot.commands;
 6
 7  import com.ctre.phoenix6.Utils;
 8  import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
 9  import com.ctre.phoenix6.swerve.SwerveRequest;
10  import frc.robot.subsystems.DriveMechanism;
11  import frc.robot.utils.AccelerationLimiter;
12  import frc.robot.utils.AllianceFlip;
13  import frc.robot.utils.ClassicCommand;
14  import java.util.function.Supplier;
15  import org.littletonrobotics.junction.Logger;
16  import org.wpilib.math.geometry.Pose2d;
17  import org.wpilib.math.geometry.Translation2d;
18  import org.wpilib.math.kinematics.ChassisVelocities;
19
20  /**
21   * Drive to a field pose by commanding velocity directly. Every loop:
22   *
23   * <ol>
24   *   <li>Where is the goal? (it may move every loop - {@link DriveToTag} and {@link DriveToPiece}
25   *       build on that)
26   *   <li>How far is it?
27   *   <li>How fast? The smallest of: top speed, the speed we can still stop from, gentle near the
28   *       end.
29   *   <li>How much may the velocity change? Physics decides: {@link AccelerationLimiter}.
30   * </ol>
31   *
32   * <p>Classic-style Commands v3 command on {@link ClassicCommand}.
33   */
34  public class DriveToPose extends ClassicCommand {
35    // Tuned in tools/alignsim (docs/alignment/studies/controllers.md). Keep DECEL and ANGULAR_DECEL
36    // below AccelerationLimiter's limits, or the robot can't brake as planned and overshoots.
37    private static final double MAX_SPEED = 4.0; // m/s
38    private static final double DECEL = 6.0; // m/s^2; braking harder costs tag accuracy at the end
39    // Close to the goal, speed = KP * distance instead of the braking curve (no chatter at the end).
40    private static final double KP = 6.0; // (m/s) per m
41    // Brake this much early to cover sensor + CAN delay. Use 0.10 with 100 Hz odometry (no CANivore).
42    private static final double REACTION_TIME = 0.05; // s
43    private static final double STOP_DEADBAND = 0.003; // m, command zero inside it
44
45    // Same ideas for heading.
46    private static final double MAX_OMEGA = 6.0; // rad/s
47    private static final double ANGULAR_DECEL = 6.0; // rad/s^2
48    private static final double KP_HEADING = 5.0; // (rad/s) per rad
49    private static final double HEADING_REACTION_TIME = 0.05; // s
50    private static final double HEADING_DEADBAND = Math.toRadians(0.3);
51
52    // Corner rule for pass-through waypoints: hand over this far out (at most), at the fastest
53    // speed whose turn fits inside the corner.
54    private static final double MAX_SWITCH_RADIUS = 0.4; // m
55    // "Stopped" for isFinished().
56    private static final double SETTLED_SPEED = 0.05; // m/s
```

## Chapter 15: Fields and constructors (lines 58–108) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | Code | Lines 58–78 highlighted as a block; a "memory shelf" icon beside them. | **58–78: THE COMMAND'S MEMORY** | Highlight to lines 59–61. | Soft note. |
| 2 | 5–12 s | Code | Lines 59–61: `blueGoal` (ghost `null` for a supplier), `goalSupplier`, `blueNext`; a pin, a pin with a motion trail, and a second pin. | **FIXED GOAL · MOVING GOAL · NEXT WAYPOINT** | Highlight to lines 63–66. | Three plucks. |
| 3 | 12–19 s | Code | Lines 63–66; ghosts `0.02 m`, `2°`, `0.0`, `0.0`. | **2 cm · 2° · endSpeed 0 · switchRadius 0** | Highlight to lines 68–73. | Pluck. |
| 4 | 19–25 s | Code | Lines 68–73; a World inset shows the request as a blue field-relative arrow on the blue-origin field. | **FIELD VELOCITY · CLOSED LOOP** | Highlight to line 75. | Whirr. |
| 5 | 26–30 s | Code | Line 75; `commanded` in blue lands in the "last command" memory slot from chapter 03. | **75: commanded = LAST COMMAND** | Highlight to lines 76–78. | Thunk. |
| 6 | 31–36 s | Code | Lines 76–78; ghosts `∞`, `∞` on `distance` and `headingError`. EXTREME CLOSE-UP: the ∞ sign. | **∞ = NOT THERE YET** | Highlight to lines 80–87. | Tiny tink. |
| 7 | 36–41 s | Code | Lines 80–87; the bold "Always write this for the blue alliance" glows; a blue-corner origin icon. | **85: A FIXED GOAL, WRITTEN FOR BLUE** | Highlight to lines 89–95. | Pluck. |
| 8 | 42–48 s | Code | Lines 89–95; a small function box hands out a fresh pin each tick; the "not flipped" words glow; World inset WIDE: odometry's blue-origin axes on the whole field. | **93: A GOAL SUPPLIER · NEVER FLIPPED** | Highlight to lines 97–100. | Soft note. |
| 9 | 49–54 s | Code | Lines 97–100; `protected` glows; `DriveToTag` and `DriveToPiece` badges dock beside it. | **98: FOR SUBCLASSES** | Highlight to lines 102–108. | Pluck. |
| 10 | 55–60 s | Code | Lines 102–108; line 104 glows; the drivetrain gets a small padlock with the command's name on it. | **104: NAME + CLAIM THE DRIVETRAIN** | Scroll to line 110 (chapter 16). | Lock click. |

**Narration (ch15-voice.json):**

1. Lines fifty-eight to seventy-eight are the fields, the command's memory.
2. Blue goal: a fixed goal, written for blue. Goal supplier: a goal that can move. Blue next: the next waypoint.
3. Then the tolerances, two centimeters and two degrees, and end speed and switch radius: zero, unless it's a waypoint.
4. Lines seventy to seventy-three build the request: a closed-loop field velocity, so even slow final moves happen.
5. Line seventy-five, commanded: the last velocity we asked for. The speed rule's only memory.
6. Lines seventy-seven and seventy-eight start distance and heading error at infinity: not there yet.
7. Line eighty-five takes a fixed goal. Its Javadoc says: always write it for the blue alliance.
8. Line ninety-three takes a supplier: a small function returning a fresh goal, already in field coordinates, so never flipped.
9. Line ninety-eight is protected, for subclasses like Drive To Tag that bring their own goal.
10. All of them reach line one hundred four, which names the command and claims the drivetrain.

**Code on screen:**

```java
 58    private final DriveMechanism drivetrain;
 59    private final Pose2d blueGoal; // null when a goal supplier is used instead
 60    private Supplier<Pose2d> goalSupplier;
 61    private Pose2d blueNext; // next waypoint, for passingThrough()
 62
 63    private double positionTolerance = 0.02; // m
 64    private double headingTolerance = Math.toRadians(2.0); // rad
 65    private double endSpeed = 0.0; // m/s; > 0 makes this a pass-through waypoint
 66    private double switchRadius = 0.0; // m; a waypoint finishes inside it
 67
 68    // Field-relative velocity request, blue-origin (the same frame as the pose). Closed-loop
 69    // velocity uses the drive gains in TunerConstants, so slow final moves still happen.
 70    private final SwerveRequest.ApplyFieldVelocity driveRequest =
 71        new SwerveRequest.ApplyFieldVelocity()
 72            .withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.BlueAlliance)
 73            .withDriveRequestType(DriveRequestType.Velocity);
 74
 75    private ChassisVelocities commanded = new ChassisVelocities();
 76    private double lastTime;
 77    private double distance = Double.POSITIVE_INFINITY;
 78    private double headingError = Double.POSITIVE_INFINITY;
 79
 80    /**
 81     * @param drivetrain the swerve drive to command
 82     * @param blueGoal the field pose to drive to, including heading. <b>Always write this for the
 83     *     blue alliance</b> - on red it is flipped for you, see {@link AllianceFlip}.
 84     */
 85    public DriveToPose(DriveMechanism drivetrain, Pose2d blueGoal) {
 86      this("DriveToPose", drivetrain, blueGoal, null);
 87    }
 88
 89    /**
 90     * Drives to a goal that can change every loop. The supplier returns a field pose (blue-origin, as
 91     * odometry reports it) and is <b>not</b> flipped for red.
 92     */
 93    public DriveToPose(DriveMechanism drivetrain, Supplier<Pose2d> goalSupplier) {
 94      this("DriveToPose", drivetrain, null, goalSupplier);
 95    }
 96
 97    /** For subclasses that override {@link #goal()}. */
 98    protected DriveToPose(String name, DriveMechanism drivetrain) {
 99      this(name, drivetrain, null, drivetrain::getPose);
100    }
101
102    private DriveToPose(
103        String name, DriveMechanism drivetrain, Pose2d blueGoal, Supplier<Pose2d> goalSupplier) {
104      super(name, drivetrain); // name + requirement, like v2 addRequirements(drivetrain)
105      this.drivetrain = drivetrain;
106      this.blueGoal = blueGoal;
107      this.goalSupplier = goalSupplier;
108    }
```

## Chapter 16: Options, and the goal method (lines 110–139) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | Code | Lines 110–115; ghosts `meters → 0.02`, `degrees → 2`. | **111: withTolerance** | Highlight to lines 117–125. | Pluck. |
| 2 | 5–11 s | Code | Lines 117–125; a World inset shows a green switch circle and a blue speed arrow set by hand. | **121: withEndSpeed (BY HAND)** | Highlight to lines 127–134. | Pluck. |
| 3 | 12–19 s | Code | Lines 127–134; the corner drawing from chapter 12 flies in beside them. | **131: passingThrough (CORNER RULE)** | Highlight `return this`. | Swoosh. |
| 4 | 20–25 s | Code | `return this;` glows in all three; a second panel shows `DriveThenAlignOpMode.java` lines 53–54 with the chain highlighted. World inset WIDE: the robot passing staging toward the scoring spot. | **RETURNS this → CHAIN** | Highlight to lines 136–139. | Two clicks. |
| 5 | 26–32 s | Code | Lines 136–139; `protected` glows; a white pin pops out of `goal()`. | **137: goal()** | `DriveToTag.java` slides in. | Pluck. |
| 6 | 32–38 s | Code | Side panel: `DriveToTag.java` lines 68–71; the pin flips between a dashed map pin and a tag pin. | **TAG GOAL, OR ELSE THE MAP** | `DriveToPiece.java` slides in. | Lock click. |
| 7 | 38–44 s | Code | Side panel: `DriveToPiece.java` lines 57–69; World inset EXTREME CLOSE-UP: the goal pin 0.40 m short of a game piece, facing it. | **0.40 m SHORT OF THE PIECE, FACING IT** | The side panels slide away. | Rolling sound. |
| 8 | 44–48 s | Code | Back on line 137; the four-question card 1 docks next to it. | **ONE METHOD = A MOVING GOAL** | Scroll to line 141 (chapter 17). | Chime. |

**Narration (ch16-voice.json):**

1. Line one hundred eleven, with tolerance, changes the finish tolerances.
2. Line one hundred twenty-one, with end speed, makes a waypoint by hand: you pick the speed and switch radius.
3. Line one hundred thirty-one, passing through, is the easy way: give it the next point, and the corner rule picks both.
4. Each returns this, the same command, so they chain. The Drive Then Align auto does exactly that.
5. Line one hundred thirty-seven, goal, just asks the supplier. It's protected, so a subclass can replace it.
6. Drive To Tag replaces it: the goal measured from the tag, or else the map goal.
7. Drive To Piece: a spot forty centimeters short of the piece, facing it. No piece? Stay put.
8. That one method lets a moving goal use the same speed rule.

**Code on screen:**

```java
110    /** Finish tolerances (defaults: 2 cm, 2 degrees). */
111    public DriveToPose withTolerance(double meters, double degrees) {
112      positionTolerance = meters;
113      headingTolerance = Math.toRadians(degrees);
114      return this;
115    }
116
117    /**
118     * Makes this a pass-through waypoint: be at {@code metersPerSecond} when {@code switchRadius}
119     * away, then finish (heading isn't checked). Chain the next leg right after.
120     */
121    public DriveToPose withEndSpeed(double metersPerSecond, double switchRadius) {
122      this.endSpeed = metersPerSecond;
123      this.switchRadius = switchRadius;
124      return this;
125    }
126
127    /**
128     * Makes this a pass-through waypoint on the way to {@code blueNext} (blue-origin, flipped for
129     * red). The switch radius and corner speed come from the angle between the two legs.
130     */
131    public DriveToPose passingThrough(Pose2d blueNext) {
132      this.blueNext = blueNext;
133      return this;
134    }
135
136    /** The goal for this loop. Override to supply a moving goal. */
137    protected Pose2d goal() {
138      return goalSupplier.get();
139    }
```

From `DriveThenAlignOpMode.java`:

```java
53                    coroutine.await(
54                        new DriveToPose(robot.drivetrain, staging).passingThrough(scoringGuess));
```

From `DriveToTag.java`:

```java
68    @Override
69    protected Pose2d goal() {
70      return tracker.getFieldGoal(tagToGoal).orElse(mapGoal);
71    }
```

From `DriveToPiece.java`:

```java
57    @Override
58    protected Pose2d goal() {
59      Optional<Translation2d> piece = tracker.getFieldPosition();
60      if (piece.isEmpty()) {
61        return drivetrain.getPose(); // brake here; never back up to where the button was pressed
62      }
63      Translation2d toPiece = piece.get().minus(drivetrain.getPose().getTranslation());
64      if (toPiece.getNorm() < 1e-3) {
65        return drivetrain.getPose();
66      }
67      Rotation2d facing = new Rotation2d(toPiece.getX(), toPiece.getY());
68      return new Pose2d(piece.get().minus(new Translation2d(INTAKE_REACH_METERS, facing)), facing);
69    }
```

## Chapter 17: initialize(): getting ready (lines 141–156) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | Code | Lines 141–143 highlighted; a "runs once" badge. World inset WIDE: the robot at its start, the goal (3.0, 0, 90°) marked. | **143: initialize() · ONCE** | Highlight to lines 144–146. | Pluck. |
| 2 | 5–10 s | Code | Lines 144 and 146; ghost on 146: `activeGoal = (3.00, 0.00, 90°)` on blue. | **146: FLIP FOR OUR ALLIANCE** | Highlight line 145. | Soft note. |
| 3 | 11–16 s | Code | Line 145 glows; a small timeline: "OpMode built (alliance unknown)" → "command starts (alliance known)". | **145: NOT IN THE CONSTRUCTOR** | The field spins in the inset. | Clock tick. |
| 4 | 17–24 s | World | The field spins half a turn about its centre: a blue pin lands on the red side; x → L − x, y → W − y, θ + 180°. | **RED: (L − x, W − y, θ + 180°)** | Back to the code. | Spin whoosh. |
| 5 | 24–30 s | Code | Line 147; a supplier box now hands out the same pin forever. | **147: goal() ALWAYS RETURNS IT** | Highlight to lines 148–150. | Pluck. |
| 6 | 31–38 s | Code | Lines 148–150; the corner drawing appears faintly, starting from the robot's current pose. | **149: CORNER RULE, FROM WHERE WE ARE** | Highlight to line 152. | Soft note. |
| 7 | 38–43 s | Code | Line 152 glows strongly; ghost `commanded = (2.00, 0.00, 0.00)`; World inset: the robot already rolling at 2 m/s. | **152: START FROM THE MEASURED VELOCITY** | The speed graph appears. | Chime. |
| 8 | 43–50 s | Board | Two speed graphs: commanded starting at 0 (a cliff, wheels spin: smoke puff) versus starting at 2 (smooth). EXTREME CLOSE-UP: the smoke puff on a tire. | **NO JUMP · NO SLIP** | Back to the code. | Squeal, then smooth tone. |
| 9 | 50–55 s | Code | Lines 153–155; ghosts: `lastTime = now`, `∞`, `∞`. | **153–155: CLOCK ON · NOT THERE YET** | Scroll to line 158 (chapter 18). | Click. |

**Narration (ch17-voice.json):**

1. Line one hundred forty-three, initialize, runs once when the command starts.
2. With a fixed blue goal, line one hundred forty-six flips it for our alliance.
3. Why here? Line one hundred forty-five: the alliance isn't known yet when an OpMode is built.
4. On red, x becomes field length minus x, y becomes field width minus y, and the heading turns half a circle.
5. Line one hundred forty-seven swaps in a supplier that always returns that goal, so execute only calls goal.
6. Lines one hundred forty-eight to one hundred fifty: with a next waypoint, run the corner rule from the robot's current pose.
7. Line one hundred fifty-two matters most: commanded starts as the robot's measured velocity.
8. Already rolling at two meters per second? The limiter starts from two, not zero: no jump, no slipping wheels.
9. Then start the clock, and reset distance and heading error: not there yet.

**Code on screen:**

```java
141    /** Picks the goal for our alliance and starts from the robot's current velocity. */
142    @Override
143    protected void initialize() {
144      if (blueGoal != null) {
145        // Resolved here, not in the constructor: the alliance isn't known when an OpMode is built.
146        Pose2d activeGoal = AllianceFlip.apply(blueGoal);
147        goalSupplier = () -> activeGoal;
148        if (blueNext != null) {
149          useCornerRule(drivetrain.getPose(), activeGoal, AllianceFlip.apply(blueNext));
150        }
151      }
152      commanded = drivetrain.getFieldVelocity();
153      lastTime = Utils.getCurrentTimeSeconds();
154      distance = Double.POSITIVE_INFINITY;
155      headingError = Double.POSITIVE_INFINITY;
156    }
```

## Chapter 18: execute(): where, and how far? (lines 158–168) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | Code | Lines 158–159; the tick motif returns faintly; a "50 × a second" badge. Ghost-text column fades in at the right edge. | **159: execute() · EVERY LOOP** | The traced moment appears in the inset. | Tick motif, quiet. |
| 2 | 7–12 s | World | WIDE inset grows to half the screen: robot at (0, 0) facing 0°, goal pin at (3.0, 0) facing 90° (purple). | **ROBOT (0, 0, 0°) · GOAL (3.0, 0, 90°)** | A blue arrow and a yellow timer appear. | Soft note. |
| 3 | 13–18 s | World | A blue arrow of 2 m/s straight ahead on the robot; a yellow "20 ms since last loop". | **LAST COMMAND (2.0, 0, 0) · 20 ms** | The inset shrinks back. | Tick. |
| 4 | 19–26 s | Code | Lines 160–162; ghosts `now = …`, `dt = 0.02` (yellow), `lastTime = now`. | **161: dt = 0.02 s** | Zoom to the min and max. | Click. |
| 5 | 27–33 s | Code | EXTREME CLOSE-UP: `Math.min(Math.max(…, 0.001), 0.1)`; a yellow slider clamps between 0.001 and 0.1. | **KEPT BETWEEN 0.001 AND 0.1 s** | Highlight to lines 164–165. | Soft note. |
| 6 | 33–38 s | Code | Lines 164–165; ghosts `pose = (0.00, 0.00, 0°)`, `goal = (3.00, 0.00, 90°)`; question card 1 lights. | **165: QUESTION 1 · goal()** | Highlight to line 166. | Note 1. |
| 7 | 39–43 s | Code | Line 166; ghost `toGoal = (3.00, 0.00)` in green; the inset draws the green arrow. | **166: toGoal = goal − pose** | Highlight to line 167. | Pluck. |
| 8 | 44–47 s | Code | Line 167; ghost `distance = 3.00`; question card 2 lights. | **167: QUESTION 2 · distance = 3.00 m** | Highlight to line 168. | Note 2. |
| 9 | 48–54 s | Code | Line 168; ghost `headingError = 1.571` (purple, "90°"); the inset shows the −π..π circle. | **168: headingError = 1.571 rad** | Scroll to line 170 (chapter 19). | Pluck. |

**Narration (ch18-voice.json):**

1. Now execute, the loop. We'll trace one moment, each line's value in grey at its end.
2. The robot is at zero, zero, facing zero degrees. The goal: three, zero, facing ninety.
3. Last loop we asked for two meters per second straight ahead, no spin. Twenty milliseconds have passed.
4. Lines one hundred sixty to sixty-two: d t, the time since last loop, then save the time. Zero point zero two seconds.
5. It's kept between a millisecond and a tenth of a second, in case a loop is skipped.
6. Line one hundred sixty-four reads the pose. Line one hundred sixty-five asks question one: goal.
7. Line one hundred sixty-six, the arrow to the goal: goal minus pose. Three, zero.
8. Line one hundred sixty-seven, question two: its length. Three meters.
9. Line one hundred sixty-eight, heading error: ninety degrees, one point five seven radians, wrapped between minus pi and pi.

**Code on screen:**

```java
158    @Override
159    protected void execute() {
160      double now = Utils.getCurrentTimeSeconds();
161      double dt = Math.min(Math.max(now - lastTime, 0.001), 0.1); // guards a skipped loop
162      lastTime = now;
163
164      Pose2d pose = drivetrain.getPose();
165      Pose2d goal = goal();
166      Translation2d toGoal = goal.getTranslation().minus(pose.getTranslation());
167      distance = toGoal.getNorm();
168      headingError = goal.getRotation().minus(pose.getRotation()).getRadians(); // -pi..pi
```

## Chapter 19: execute(): how fast? (lines 170–183) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | Code | Lines 170–172; ghost `speedNow = 2.00` (blue); question card 3 lights. | **172: speedNow = 2.00 m/s** | Highlight to line 173. | Note 3. |
| 2 | 7–12 s | Code | Line 173; `distance` green, `speedNow` blue, `REACTION_TIME` yellow, `switchRadius` green. | **173: brakingDistance** | Numbers fill in. | Pluck. |
| 3 | 13–19 s | Code | Ghost `brakingDistance = 3.00 − 2.00 × 0.05 − 0 = 2.90`; EXTREME CLOSE-UP: `Math.max(0.0, …` with a floor under it. | **2.90 m** | The board formula flies in. | Click. |
| 4 | 19–25 s | Code | Lines 174–175; the trapezoid formula from chapter 07 flies off the board and lands on line 175, matching term by term. | **v² = v_end² + 2·a·d** | Numbers fill in. | Whoosh. |
| 5 | 25–30 s | Code | Ghost `√(0 + 12 × 2.90) = 5.90 → min(4.0, 5.90) = 4.00`. | **speed = 4.00** | Highlight to line 176. | Pluck. |
| 6 | 31–35 s | Code | Line 176; ghost `endSpeed == 0.0 → true`. | **176: STOPPING HERE → GENTLE LINE** | Highlight to line 177. | Soft note. |
| 7 | 36–40 s | Code | Line 177; ghost `min(4.00, 6 × 3.00 = 18.0) = 4.00`. | **177: speed = 4.00** | Highlight to lines 178–180. | Pluck. |
| 8 | 41–43 s | Code | Lines 178–180; ghost `3.00 < 0.003 → false`. | **178: NOT INSIDE 3 mm** | Highlight to lines 182–183. | Soft click. |
| 9 | 44–48 s | Code | Lines 182–183 highlighted together. | **182–183: SPEED → ARROW** | The inset draws arrows. | Pluck. |
| 10 | 48–55 s | World | Inset: the green arrow (3, 0) shrinks to a unit arrow of length 1, then stretches to a blue arrow of length 4. | **toGoal ÷ distance × speed** | Back to the code. | Stretch sound. |
| 11 | 55–60 s | Code | Ghosts `vx = 3.00 ÷ 3.00 × 4.00 = 4.00`, `vy = 0.00`. | **vx = 4.00 · vy = 0.00** | The inset zooms out. | Two plucks. |
| 12 | 61–65 s | World | WIDE inset: robot moving at 2 m/s (blue), a faint 4 m/s "wish" arrow ahead of it with a "?" | **HAVE 2 · WANT 4** | Scroll to line 185 (chapter 20). | Soft note. |

**Narration (ch19-voice.json):**

1. Line one hundred seventy-two, speed now: the length of the last command. Two meters per second.
2. Line one hundred seventy-three, braking distance: distance, minus speed times reaction time, minus switch radius.
3. Three, minus two times zero point zero five: two point nine meters. The max stops it going negative.
4. Lines one hundred seventy-four and seventy-five: the trapezoid rule from the board, capped at max speed.
5. The square root of twelve times two point nine is five point nine. Capped: four.
6. Line one hundred seventy-six: end speed is zero, so the gentle line applies.
7. K P times distance is eighteen. Four is smaller, so speed stays four.
8. Line one hundred seventy-eight: under three millimeters? No.
9. Lines one hundred eighty-two and eighty-three turn speed into an arrow.
10. The arrow to the goal, divided by its length, has length one. Times speed: v x and v y.
11. v x is four, v y is zero: straight at the goal, at top speed.
12. We're going two and asking for four. Can we get it? Not decided yet.

**Code on screen:**

```java
170      // Speed toward the goal: the fastest speed that can still brake to endSpeed in the distance
171      // left, capped by MAX_SPEED and (when stopping) by the gentle KP line near the goal.
172      double speedNow = Math.hypot(commanded.vx, commanded.vy);
173      double brakingDistance = Math.max(0.0, distance - speedNow * REACTION_TIME - switchRadius);
174      double speed =
175          Math.min(MAX_SPEED, Math.sqrt(endSpeed * endSpeed + 2.0 * DECEL * brakingDistance));
176      if (endSpeed == 0.0) {
177        speed = Math.min(speed, KP * distance);
178        if (distance < STOP_DEADBAND) {
179          speed = 0.0;
180        }
181      }
182      double vx = distance > 1e-6 ? toGoal.getX() / distance * speed : 0.0;
183      double vy = distance > 1e-6 ? toGoal.getY() / distance * speed : 0.0;
```

## Chapter 20: execute(): how fast to turn? (lines 185–194) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | Code | Lines 185–194 highlighted as a block, all purple names. | **185–194: THE SAME, FOR TURNING** | Highlight to lines 185–186. | Soft note. |
| 2 | 5–12 s | Code | Lines 185–186; ghost `1.571 − 0.00 × 0.05 = 1.571`. | **brakingAngle = 1.571 rad** | Highlight to lines 187–189. | Pluck. |
| 3 | 13–16 s | Code | Lines 187–189; ghost on 189: `6.0`. Three purple dials from chapter 05 appear. | **189: MAX_OMEGA = 6** | Highlight to line 191. | Gauge chime. |
| 4 | 17–23 s | Code | Line 191; ghost `√(12 × 1.571) = 4.342`. | **191: 4.34** | Highlight to line 192. | Pluck. |
| 5 | 24–28 s | Code | Line 192; ghost `5 × 1.571 = 7.854`. | **192: 7.85** | The three dials compare. | Pluck. |
| 6 | 29–32 s | Code | Ghost on 187: `turnSpeed = 4.342`; the board's circled 4.34 from chapter 11 flies in and matches. | **turnSpeed = 4.34 rad/s** | Highlight to lines 193–194. | Sting. |
| 7 | 33–39 s | Code | Lines 193–194; ghost `1.571 < 0.0052 → false`. EXTREME CLOSE-UP: `copySign`. | **DEADBAND? NO · copySign** | The inset turns. | Soft click. |
| 8 | 39–44 s | World | Inset: a purple curved arrow, counterclockwise, toward 90°. | **omega = +4.34 rad/s (COUNTERCLOCKWISE)** | WIDE inset. | Whirr. |
| 9 | 44–50 s | World | WIDE inset: the robot with its wish: a blue 4 m/s arrow forward and a purple 4.34 rad/s spin arrow. | **WISH: 4 m/s · 4.34 rad/s** | A padlock (card 4) appears (chapter 21). | Soft note. |

**Narration (ch20-voice.json):**

1. Lines one hundred eighty-five to ninety-four do the same for turning.
2. Braking angle: the angle left, minus spin rate times heading reaction time. No spin yet, so one point five seven radians.
3. Then three spin rates; the smallest wins. Max omega: six.
4. Can I still stop turning: the square root of twelve times one point five seven. Four point three four.
5. The gentle line: five times one point five seven, seven point eight five.
6. Smallest: four point three four, the number from the board.
7. Lines one hundred ninety-three and ninety-four: inside the deadband, zero. Otherwise copy sign gives it the error's sign.
8. So plus four point three four radians per second: counterclockwise, toward ninety degrees.
9. This loop's wish: four meters per second forward, four point three four radians per second of spin.

**Code on screen:**

```java
185      double brakingAngle =
186          Math.max(0.0, Math.abs(headingError) - Math.abs(commanded.omega) * HEADING_REACTION_TIME);
187      double turnSpeed =
188          Math.min(
189              MAX_OMEGA,
190              Math.min(
191                  Math.sqrt(2.0 * ANGULAR_DECEL * brakingAngle),
192                  KP_HEADING * Math.abs(headingError)));
193      double omega =
194          Math.abs(headingError) < HEADING_DEADBAND ? 0.0 : Math.copySign(turnSpeed, headingError);
```

## Chapter 21: execute(): how much change? (lines 196–205) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–9 s | Code | Lines 196–197; question card 4 (the orange padlock) lights; `AccelerationLimiter.limit` glows. | **197: AccelerationLimiter.limit (VIDEO 1)** | A side card opens. | Note 4. |
| 2 | 9–13 s | Board | Side card: HAVE (2.00, 0, 0) in blue · WANT (4.00, 0, 4.34) in blue and purple · dt 0.02 in yellow. | **HAVE · WANT · dt** | The first check lights. | Soft note. |
| 3 | 14–20 s | Board | Check 1, turning: (4.34 − 0) ÷ 0.02 = 217 rad/s²; capped to 8 (purple bar clipped). | **TURNING: 217 → 8 rad/s²** | Check 2 lights. | Clip click. |
| 4 | 21–28 s | Board | Check 2, motors: wheel speed 2 m/s; teal motor at 150 A each → 26.3 m/s²; wanted (4 − 2) ÷ 0.02 = 100, clipped to 26.3. | **MOTORS: 100 → 26.3 m/s²** | Check 3 lights. | Motor whine. |
| 5 | 28–33 s | Board | Check 3, grip: a circle of radius 10 (orange); the combined arrow √(26.3² + (8 × 0.359)²) = 26.5 sticks far outside it. EXTREME CLOSE-UP: the arrow tip poking out of the circle. | **GRIP: 26.5 > 10** | The arrow shrinks. | Squeal. |
| 6 | 34–38 s | Board | The whole arrow scales by 0.378 to touch the circle, keeping its direction. | **× 0.38 · DIRECTION KEPT** | Numbers drop in. | Spring. |
| 7 | 38–44 s | Board | ax = 9.94 m/s² (orange), alpha = 3.02 rad/s² (purple), × 0.02 s (yellow). | **9.94 m/s² · 3.02 rad/s² · × 0.02 s** | Back to the code. | Two plucks. |
| 8 | 44–50 s | Code | Line 197 ghost `commanded = (2.199, 0.000, 0.060)`. | **NEW COMMAND: 2.20 m/s · 0.06 rad/s** | WIDE inset. | Chime. |
| 9 | 51–54 s | World | WIDE inset: the robot's blue arrow grows a sliver, from 2.00 to 2.20; the purple spin arrow just starts. The tick motif clicks once. | **ALL PHYSICS ALLOWS THIS LOOP** | Highlight to line 199. | Tick. |
| 10 | 54–58 s | Code | Line 199; a teal packet leaves the panel toward the four modules in the inset. | **199: setControl** | Highlight to lines 201–204. | Packet blip. |
| 11 | 58–63 s | Code | Lines 201–204; four small AdvantageScope-style graphs start drawing beside them (goal, distance, heading error, command). | **201–204: LOG IT** | Scroll to line 207 (chapter 22). | Pen draw. |

**Narration (ch21-voice.json):**

1. Physics decides, says line one hundred ninety-six. Line one hundred ninety-seven gives the limiter what we have, what we want, and d t.
2. Have: two, zero, zero. Want: four, zero, four point three four.
3. Three checks. Turning: that spin change is over two hundred radians per second squared, so it's capped at eight.
4. Motors: at two meters per second and one hundred fifty amps each, they can add about twenty-six. We wanted one hundred.
5. Grip: driving and turning together ask for twenty-six point five. The tires give ten.
6. So both scale to about thirty-eight percent, together, keeping the path straight.
7. Nine point nine four forward, three point zero two of spin, for zero point zero two seconds.
8. New command: two point two zero meters per second, and zero point zero six radians per second.
9. That's all physics allows. Next loop, it asks again.
10. Line one hundred ninety-nine sends it to the swerve drive.
11. Lines two hundred one to two hundred four log four values, to graph in AdvantageScope.

**Code on screen:**

```java
196      // Physics decides how much of that change we can have this loop.
197      commanded = AccelerationLimiter.limit(commanded, new ChassisVelocities(vx, vy, omega), dt);
198
199      drivetrain.setControl(driveRequest.withVelocity(commanded));
200
201      Logger.recordOutput(name() + "/Goal", goal);
202      Logger.recordOutput(name() + "/DistanceMeters", distance);
203      Logger.recordOutput(name() + "/HeadingErrorDegrees", Math.toDegrees(headingError));
204      Logger.recordOutput(name() + "/CommandedVelocity", commanded);
205    }
```

## Chapter 22: isFinished(), and the corner rule (lines 207–235) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–5 s | Code | Lines 207–209; a small loop diagram: execute → isFinished? | **209: isFinished() · AFTER EVERY execute** | Highlight to lines 210–212. | Soft note. |
| 2 | 5–11 s | Code | Lines 210–212; ghost `max(switchRadius, 0.05)`; a World inset shows the green finish circle. | **211: WAYPOINT: INSIDE max(r, 5 cm)** | Highlight to lines 213–216. | Pluck. |
| 3 | 12–17 s | Code | Lines 213–216; three checkboxes beside 214, 215, 216; ghosts `3.00 < 0.02 → false`; result ghost `false`. | **214–216: 2 cm · 2° · < 0.05 m/s → false** | Highlight to lines 219–223. | Three soft clicks. |
| 4 | 18–23 s | Code | Lines 219–223; World inset WIDE: the traced route (0, 0) → (2, 0) → (2, 1.5). | **223: useCornerRule** | Highlight to lines 224–227. | Soft note. |
| 5 | 24–27 s | Code | Lines 224–227; ghosts `inX = 2.0, inY = 0.0, outX = 0.0, outY = 1.5`; two green arrows in the inset. | **IN (2, 0) · OUT (0, 1.5)** | Highlight to line 228. | Two plucks. |
| 6 | 28–33 s | Code | Line 228; `inX * outY - inY * outX` labelled "cross", `inX * outX + inY * outY` labelled "dot". | **228: atan2(CROSS, DOT)** | A Board card opens. | Soft note. |
| 7 | 34–40 s | Board | Two arrows: when they cross at a right angle, the "cross" bar is full and the "dot" bar empty; when they line up, the reverse. | **CROSS = ACROSS · DOT = ALONG** | Numbers drop in. | Two notes. |
| 8 | 40–45 s | Code | EXTREME CLOSE-UP: ghost `cross = 3.0, dot = 0.0 → turn = 1.571 (90°)`. | **TURN = 90°** | Highlight to lines 229–230. | Pluck. |
| 9 | 45–52 s | Code | Lines 229–230; ghost `min(0.4, 0.4 × min(2.0, 1.5) = 0.6) = 0.4`. | **switchRadius = 0.4 m** | Highlight to lines 231–234. | Pluck. |
| 10 | 53–59 s | Code | Lines 231–234; ghost `1.571 < 0.0175 → false`; the board's corner formula flies in and lands on line 234. | **234: √(DECEL × r ÷ tan(turn ÷ 2))** | Numbers fill in. | Whoosh. |
| 11 | 59–65 s | Code | Ghost `√(6 × 0.4 ÷ 1.0) = √2.4 = 1.549 → endSpeed = 1.55`. | **endSpeed = 1.55 m/s** | The inset robot rounds the corner. | Sting. |
| 12 | 65–70 s | World | Inset: the robot brakes to 1.55 m/s at the green 0.4 m circle (line 173's switchRadius term glows in the code) and rounds the corner. | **BOARD → JAVA** | Scroll to line 237 (chapter 23). | Swoosh. |

**Narration (ch22-voice.json):**

1. Line two hundred nine, is finished, is checked after each execute.
2. Lines two hundred ten and eleven: a waypoint is done inside its switch radius, at least five centimeters.
3. Otherwise, lines two hundred thirteen to sixteen check all three. We're three meters away, so: false.
4. Lines two hundred nineteen to thirty-five: use corner rule. Let's trace our ninety degree route.
5. Two arrows. In: two, zero. Out: zero, one point five.
6. Line two hundred twenty-eight finds the turn between them, with the cross product and the dot product.
7. The cross product measures how much the arrows point across each other; the dot product, how much along.
8. Cross is three, dot is zero. A-tan-two of three and zero: ninety degrees.
9. Lines two hundred twenty-nine and thirty: forty percent of the shorter leg is zero point six, capped at zero point four.
10. Lines two hundred thirty-one to thirty-four: the turn is over one degree, so use the corner formula.
11. Six times zero point four, over one. Square root: one point five five meters per second.
12. The board's formula, in Java. Execute then brakes to it at the switch circle.

**Code on screen:**

```java
207    /** Waypoints finish when close; otherwise at the goal, facing the right way, and stopped. */
208    @Override
209    protected boolean isFinished() {
210      if (endSpeed > 0.0) {
211        return distance < Math.max(switchRadius, 0.05);
212      }
213      ChassisVelocities measured = drivetrain.getFieldVelocity();
214      return distance < positionTolerance
215          && Math.abs(headingError) < headingTolerance
216          && Math.hypot(measured.vx, measured.vy) < SETTLED_SPEED;
217    }
218
219    /**
220     * Corner rule: switch radius = min(0.4 m, 40% of the shorter leg), speed =
221     * sqrt(DECEL*r/tan(turn/2)).
222     */
223    private void useCornerRule(Pose2d start, Pose2d waypoint, Pose2d next) {
224      double inX = waypoint.getX() - start.getX();
225      double inY = waypoint.getY() - start.getY();
226      double outX = next.getX() - waypoint.getX();
227      double outY = next.getY() - waypoint.getY();
228      double turn = Math.abs(Math.atan2(inX * outY - inY * outX, inX * outX + inY * outY));
229      switchRadius =
230          Math.min(MAX_SWITCH_RADIUS, 0.4 * Math.min(Math.hypot(inX, inY), Math.hypot(outX, outY)));
231      endSpeed =
232          turn < Math.toRadians(1.0)
233              ? MAX_SPEED
234              : Math.min(MAX_SPEED, Math.sqrt(DECEL * switchRadius / Math.tan(turn / 2.0)));
235    }
```

## Chapter 23: end(), and chaining legs (lines 237–245) (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–6 s | Code | Lines 237–239; a "runs once" badge with two arrows into it: "finished" and "interrupted". | **239: end()** | Highlight to line 240. | Soft note. |
| 2 | 6–11 s | Code | Line 240; ghost `interrupted = false, endSpeed = 0.0 → true` for our stopping leg. | **240: INTERRUPTED, OR A STOPPING LEG → STOP** | Highlight to lines 241–242. | Pluck. |
| 3 | 11–17 s | Code | Line 241's warning glows; line 242 ghost `velocity (0, 0, 0)`; EXTREME CLOSE-UP: a module still pointing and spinning under "Idle", then stilled by zero. | **IDLE IS NOT A STOP → SEND ZERO** | WIDE inset of the corner route. | Brake hiss. |
| 4 | 18–23 s | World | WIDE inset: the waypoint robot crosses its circle still rolling at 1.55 m/s; `end()` sends nothing; ghost `endSpeed = 1.55 → skip`. | **WAYPOINT: KEEP ROLLING** | The next leg's badge lights. | Swoosh. |
| 5 | 23–28 s | Code | The next command's line 152 flashes: `commanded = measured (≈ 1.55 m/s)`; the speed graph joins with no step. | **NEXT LEG STARTS FROM THE MEASURED SPEED** | A side panel slides in. | Smooth tone. |
| 6 | 28–33 s | Code | Side panel: `DriveThenAlignOpMode.java` lines 53–63; World inset: through staging at speed, then `DriveToTag` takes over. | **DRIVE THEN ALIGN: PASS, THEN TAG** | A second side panel. | Lock click. |
| 7 | 34–41 s | Code | Side panel: `AutonomousOpMode.java` lines 33–35 with a ghost edit: `.passingThrough(secondLeg)` added to line 34; the inset shows our 90° corner. | **AUTO "2 - Drive To Pose" + passingThrough** | The panel scrolls the whole file. | Pluck. |
| 8 | 41–46 s | Code | The whole file scrolls past in the gutter (1 to 245) and the four question cards stack on top. | **4 QUESTIONS · 1 SPEED RULE · 245 LINES** | Pull out of the robot to the field (Part 5). | Resolve chord. |

**Narration (ch23-voice.json):**

1. Line two hundred thirty-nine, end, runs once when the command finishes or is interrupted.
2. If interrupted, or this leg stops at its goal, it stops the robot.
3. Line two hundred forty-one warns: Idle is not a stop. So line two hundred forty-two sends zero velocity.
4. A waypoint that finishes normally sends nothing, and keeps rolling at its corner speed.
5. The next command's initialize seeds commanded from that measured speed. A smooth hand-over.
6. Drive Then Align does this: through staging at speed, then Drive To Tag takes over.
7. The template's two-leg auto stops after each leg. Make its first leg pass through, and you get our ninety degree corner.
8. That's the whole file: four questions, one speed rule, two hundred forty-five lines.

**Code on screen:**

```java
237    /** Stops the drivetrain - except after a waypoint, where the next leg takes over at speed. */
238    @Override
239    protected void end(boolean interrupted) {
240      if (interrupted || endSpeed == 0.0) {
241        // Idle is not a stop: it leaves every module on its last request, so send zero velocity.
242        drivetrain.setControl(driveRequest.withVelocity(new ChassisVelocities()));
243      }
244    }
245  }
```

From `DriveThenAlignOpMode.java`:

```java
53                    coroutine.await(
54                        new DriveToPose(robot.drivetrain, staging).passingThrough(scoringGuess));
55                    // Local: lock onto the tag and finish on it.
56                    coroutine.await(
57                        new DriveToTag(
58                                robot.drivetrain,
59                                TAG_ID,
60                                TAG_TO_GOAL,
61                                robot.limelightBR,
62                                robot.limelightBL)
63                            .withTimeout(Seconds.of(3.0)));
```

From `AutonomousOpMode.java`:

```java
33          Command.sequence(
34                  new DriveToPose(robot.drivetrain, firstLeg),
35                  new DriveToPose(robot.drivetrain, secondLeg))
```

# Part 5: Recap

## Chapter 24: The whole idea, and check yourself (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–3 s | World | WIDE: the field; the whole traced drive replays at speed, the four cards orbiting the robot. | **THE WHOLE IDEA** | Card 1 glows. | Resolve theme. |
| 2 | 4–10 s | World | The four cards light in turn around the robot as it drives. | **1 · EVERY 20 ms: 4 QUESTIONS** | Three gauges appear. | Four notes. |
| 3 | 11–18 s | World | Three blue gauges over the path; the smallest glows as the robot cruises, brakes, glides. | **2 · SMALLEST OF 3 SPEEDS** | A yellow clock, a purple dial and a corner appear. | Gauge chimes. |
| 4 | 18–25 s | World | A yellow 0.05 s clock, the purple heading dials, and a corner arc, side by side. | **3 · BRAKE EARLY · TURN · CORNERS** | The pin slides; three ticks. | Three plucks. |
| 5 | 25–31 s | World | The pin slides and the robot follows, then three checkboxes tick: 2 cm, 2°, stopped. | **4 · NO CLOCK · DONE = CLOSE, ALIGNED, STOPPED** | A pause card drops. | Success sting. |
| 6 | 32–34 s | Board | A "Check yourself" card with five empty question slots and a pause icon. | **CHECK YOURSELF** | The first slot opens. | Soft chime. |
| 7 | 34–44 s | Board | Pause card "Question 1"; a speed axis with 2 m/s marked; a pause icon; hold 4 s. | **Q1 · 2 m/s, DECEL 6: HOW FAR TO STOP? (PAUSE)** | The answer writes on. | Soft tick during the pause. |
| 8 | 44–47 s | Board | 2² ÷ (2 × 6) = 4 ÷ 12 = 0.33 m, written on; a green bracket on the faint field. | **0.33 m** | Card "Question 2". | Chime. |
| 9 | 47–55 s | Board | Pause card "Question 2": the two lanes from the race, frozen. Hold 4 s. | **Q2 · WHY DOES P × d ALONE OVERSHOOT? (PAUSE)** | The answer draws. | Soft tick. |
| 10 | 55–61 s | Board | Green 0.8 m bracket vs orange 2.67 m bracket. | **BRAKES AT v ÷ P · NEEDS v² ÷ 2a** | Card "Question 3". | Chime. |
| 11 | 62–69 s | Board | Pause card "Question 3": the steep curve at the origin. Hold 4 s. | **Q3 · WHY THE GENTLE LINE? (PAUSE)** | The answer draws. | Soft tick. |
| 12 | 69–73 s | Board | EXTREME CLOSE-UP of the origin: the curve's spike beside the calm line. | **STEEP AT ZERO: WIGGLES → LURCHES** | Card "Question 4". | Chime. |
| 13 | 73–81 s | Code | Pause card "Question 4" over line 152. Hold 4 s. | **Q4 · WHY START FROM THE MEASURED VELOCITY? (PAUSE)** | The answer appears. | Soft tick. |
| 14 | 81–86 s | Board | Two speed graphs from chapter 17: cliff with smoke vs smooth. | **NO JUMP · NO SLIP** | Card "Question 5". | Chime. |
| 15 | 86–95 s | Board | Pause card "Question 5": a 135° corner beside a 90° corner. Hold 4 s. | **Q5 · 135° vs 90°: FASTER OR SLOWER? (PAUSE)** | The answer draws. | Soft tick. |
| 16 | 95–99 s | Board | Arcs: 135° tiny (1.00 m/s), 90° bigger (1.55 m/s). | **SLOWER · 1.00 vs 1.55 m/s** | The simulator window opens (chapter 25). | Chime. |

**Narration (ch24-voice.json):**

1. The whole idea, in four sentences.
2. One: every twenty milliseconds, it asks where the goal is, how far, how fast, and how much it may change.
3. Two: how fast is the smallest of top speed, the speed it can still stop from, and a gentle line.
4. Three: it brakes early for its delay, turns the same way, and takes corners at a speed that fits.
5. Four: with no clock the goal may move, and it's done only when close, aligned, and stopped.
6. Now check yourself. Pause after each question.
7. Question one: at two meters per second, with decel six, how far does it take to stop?
8. Four over twelve: zero point three three meters.
9. Question two: why does P times distance alone overshoot at high speed?
10. It starts braking at v over P, closer than the v squared over two a it needs.
11. Question three: why the gentle line near the goal?
12. The curve is so steep there that tiny wiggles become big lurches.
13. Question four: why does initialize start from the measured velocity?
14. So a moving start has no jump in the command, and the wheels don't slip.
15. Question five: is a one hundred thirty-five degree corner faster or slower than ninety?
16. Slower: one point zero zero meters per second, not one point five five.

## Chapter 25: Try it (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0–4 s | World | WIDE: the simulator's field view with the robot at its start; a laptop frame around it. | **TRY IT: THE JAVA SIMULATOR** | A terminal line types. | Laptop chime. |
| 2 | 4–9 s | Code | A terminal types `./gradlew simulateJava`; the driver station shows auto "2 - Drive To Pose" selected, then Enable. Then AdvantageScope opens the newest `logs/akit_*.wpilog`. | **AUTO "2 - Drive To Pose" · OPEN THE LOG** | AdvantageScope's field and graph appear. | Typing, then click. |
| 3 | 10–15 s | World | AdvantageScope: `Drivetrain/Pose` on the 2D field; `DriveToPose/CommandedVelocity` vx on a line graph. | **Drivetrain/Pose · DriveToPose/CommandedVelocity** | The graph draws. | Pen draw. |
| 4 | 16–20 s | Board | The blue speed trace annotated: speed up · cruise (if long enough) · brake curve · glide. | **UP · CRUISE · BRAKE · GLIDE** | Zoom into line 40. | Four plucks. |
| 5 | 21–25 s | Code | Line 40: ghost edit `KP = 6.0` → `2.0`. EXTREME CLOSE-UP on the "2". | **KP: 6 → 2** | Run again. | Key click. |
| 6 | 26–31 s | Board | Two speed traces overlaid: KP 6 and KP 2; the KP 2 tail creeps long; a snail icon from the race. | **YOU BUILT THE GENTLE P ROBOT** | Line 40 returns to 6. | Slow tick, then a "ding". |
| 7 | 31–37 s | Code | Line 40 back to 6.0; a note card "one change at a time"; a folder icon `tools/alignsim`. | **ONE CHANGE AT A TIME · tools/alignsim** | Pull out to the field. | Soft note. |
| 8 | 38–42 s | World | WIDE: the robot on the mark; a white AprilTag lights up on the element ahead; end card "Next: driving to a tag (video 3)". | **NEXT: TRUST THE TAG** | End card. | Success sting. |

**Narration (ch25-voice.json):**

1. Now try it in the Java simulator.
2. Run the auto called two, Drive To Pose. Then open the newest log in AdvantageScope.
3. Put drivetrain pose on the field view, and Drive To Pose commanded velocity on a graph.
4. Find the shape: speed up, maybe cruise, a smooth brake, then a gentle glide.
5. Now change K P from six to two, and run it again.
6. That's the gentle P robot from the race. The end should turn into a long creep.
7. Put it back. Change one number at a time, or explore the Python simulator in tools, align sim.
8. Next video: measuring the tag, so the goal is the real one.

## Fact check

Line numbers are `DriveToPose.java` unless another file is named. Arithmetic was recomputed in Python.

| Claim | Basis |
| --- | --- |
| Every 20 ms, four questions | `execute()` runs once per 50 Hz robot loop via `ClassicCommand.run` (`ClassicCommand.java`:58–68); the four questions are the Javadoc, lines 20–33. |
| Done = within 2 cm and 2°, measured speed under 0.05 m/s | Lines 63–64 (defaults), 56 (`SETTLED_SPEED`), 213–216 (`getFieldVelocity()` is the measured velocity, `DriveMechanism.java`:80–83). |
| A waypoint finishes inside max(switch radius, 5 cm), heading not checked | Lines 117–119, 210–211. |
| Top speed 4 m/s, about 88% of free speed | Line 37. Free speed 5785 rpm × 2π ÷ 60 = 605.8 rad/s ÷ 7.3636 × 0.05504 m = 4.53 m/s (`kSpeedAt12Volts` 4.54, `TunerConstants.java`:93); 4 ÷ 4.53 = 0.883. |
| Planned braking 6 m/s²; harder braking costs tag accuracy | Line 38 and its comment; controllers study §6 (DECEL 9: 97% → 85–89% on the hard final-approach world; DECEL 6: 98–99%). |
| Stopping from 4 m/s takes 0.67 s and 1.33 m | t = v ÷ a = 4 ÷ 6 = 0.667 s; d = area of the triangle = ½ × 4 × 0.667 = 1.333 m = 4² ÷ (2 × 6) = 16 ÷ 12. |
| v = √(2·a·d): 3.46 m/s at 1 m, 2.45 m/s at 0.5 m, 6 m/s at 3 m (capped to 4) | √12 = 3.464; √6 = 2.449; √36 = 6.0; cap at line 175. |
| Waypoint trapezoid: v² = v_end² + 2·a·d | Area = (v + v_end) ÷ 2 × (v − v_end) ÷ a = (v² − v_end²) ÷ (2a); the code is lines 174–175 (`endSpeed * endSpeed + 2.0 * DECEL * brakingDistance`). |
| Brake 0.05 s early: 0.2 m at 4 m/s; braking starts at 1.53 m | Lines 42, 173. 4 × 0.05 = 0.2; 1.333 + 0.2 = 1.533. |
| The delay is old readings, the loop wait and CAN; use 0.10 at 100 Hz | Comment line 41; controllers study "The answer" table (`REACTION_TIME` row) and §2 ("the loop's total delay (about 45 ms)"). |
| Gentle line wins inside ⅓ m, at 2 m/s | Line 177. 6d = √(12d) → 36d² = 12d → d = 12 ÷ 36 = 0.333 m; 6 × 0.333 = 2.0 = √(12 × 0.333). (Drawn without the reaction term.) |
| At 1 mm: curve 11 cm/s, line 0.6 cm/s | √(12 × 0.001) = 0.1095 m/s; 6 × 0.001 = 0.006 m/s. |
| Curve alone jitters about 9 cm/s with 1 cm noise (sim) | Controllers study §4, pose-noise table: "√ only" 9.3 cm/s jitter. |
| KP 8 or more starts a tiny back-and-forth; 6 doesn't (sim) | Controllers study "The answer" table (`KP` row: "8 or more starts a small limit cycle") and §2 (kp 8 sub-mm limit cycle; kp 6 none). |
| 3 mm stop deadband, no time cost (sim) | Line 43, 178–180; controllers study §4 final-approach table (+3 mm deadband: 1.58 s vs 1.59 s, jitter 0.00). |
| P-only race: P 5 · limit 3 overshoots about 2 m in the worst runs | Controllers study §7: 205 cm overshoot p95 (these are B-Line's README example values; 1–6 m moves from rest, n = 60). |
| P-only race: P 5 · limit 10 overshoots 20 cm | Controllers study §7. |
| P-only race: P 2 is clean but 2.20 s vs 1.39 s to 5 cm | Controllers study §7: P 2 t 5 cm 2.20 s, overshoot 0.1 cm; `DriveToPose` 1.39 s, 0.0 cm. |
| Why P alone brakes late | Speed = P × d drops below the current speed v when d < v ÷ P: 4 ÷ 5 = 0.8 m. Needed at 3 m/s²: 4² ÷ 6 = 2.67 m (2.67 − 0.8 = 1.87 m short, close to the 205 cm); at 10 m/s²: 16 ÷ 20 = 0.8 m (zero margin); P 2 starts at 4 ÷ 2 = 2 m. Same explanation in controllers study §7. |
| Plan 6 under a limit of 10; heading plan 6 under 8 | Lines 35–36, 38, 47; `AccelerationLimiter.java`:26–29 (`TRACTION_ACCEL` 10, `ANGULAR_ACCEL` 8). |
| Plan 4 with limit 3 overshot 67 cm (sim) | Controllers study §5 and "The answer" point 3 ("accel 3 < decel 4 overshoots up to 67 cm"). |
| B-Line: P on remaining path distance, cross-track push, rate limiter, fixed path, editor, heading along the path | `research/4-libraries.md` §3 (BLine: `FollowPath.execute`, `ChassisRateLimiter`, "editing the source affects the next run, never an active run", student-maintained); controllers study §7. |
| 2056: P on distance with a speed cap; line-up end point from the measured scoring spot | `research/2-orbit-581-2056.md` §3 (`DriveToPoint`: `mag = min(mag, maxVel)`; 2025 binder: scoring spot measured relative to the chassis, offset added to odometry); controllers study §7. |
| "Our copies, not their robots" | Controllers study, Caveats: "The other teams' laws are ports from public code ... Details may differ." |
| Heading: 6 rad/s, 6 rad/s², × 5, 0.05 s early, 0.3° deadband | Lines 46–50, 185–194. 6 rad/s ÷ 2π = 0.95 turns per second. |
| Stopping a turn from 6 rad/s takes 3 rad (172°) | 6² ÷ (2 × 6) = 3 rad = 171.9°. |
| Heading reaction at top spin = 0.3 rad | 6 × 0.05 = 0.3 rad (17.2°). |
| Heading gentle line wins inside 0.48 rad (27°) | 5θ = √(12θ) → 25θ² = 12θ → θ = 0.48 rad = 27.5°. |
| Heading error wraps to −π..π (short way) | Line 168 (`Rotation2d.minus(...).getRadians()`, comment "-pi..pi"). |
| 90° traced turn: 4.34, 7.85, 6 → 4.34 rad/s | √(12 × 1.5708) = 4.342; 5 × 1.5708 = 7.854; min(6, 4.342, 7.854) = 4.342 (lines 187–192). |
| Switch radius = min(0.4 m, 40% of the shorter leg) | Lines 54, 229–230. Traced corner: 0.4 × min(2.0, 1.5) = 0.6 → min(0.4, 0.6) = 0.4. |
| Turning circle R = r ÷ tan(turn ÷ 2) | Right triangle (corner, touch point, centre): the radii to the two touch points are perpendicular to the legs, so they meet at the turn angle, and the line to the corner halves it. So tan(turn ÷ 2) = r ÷ R, and R = r ÷ tan(turn ÷ 2). 90°: 0.4 ÷ 1 = 0.4 m. |
| Corner speed v = √(DECEL × r ÷ tan(turn ÷ 2)) | Sideways acceleration v² ÷ R ≤ DECEL; code lines 231–234; Javadoc lines 219–222; controllers study §"The answer" point 4. |
| 45°: 2.41 · 90°: 1.55 · 135°: 1.00 m/s | √(2.4 ÷ 0.4142) = √5.794 = 2.407; √(2.4 ÷ 1) = 1.549; √(2.4 ÷ 2.4142) = √0.9941 = 0.997. |
| Under about 17°, the corner speed is capped at 4 m/s | √(2.4 ÷ tan(θ ÷ 2)) ≥ 4 ⇔ tan(θ ÷ 2) ≤ 0.15 ⇔ θ ≤ 17.06°; cap is `Math.min(MAX_SPEED, …)` at line 234. |
| Under 1°: straight, top speed | Lines 232–233. |
| Corner rule 11–26% faster than stop-and-go; within 13–21 cm of the lines (sim) | Controllers study §3: zigzag 4.68 → 3.45 s (26.3%), dogleg 3.50 → 3.00 s (14.3%), hairpin 3.51 → 3.12 s (11.1%); path deviation 21 / 15 / 13 cm. |
| Execute brakes to the corner speed at the switch circle | Line 173 subtracts `switchRadius`; line 175 uses `endSpeed`. |
| Traced corner: cross 3, dot 0, turn 90° | Lines 224–228: in = (2, 0), out = (0, 1.5); cross = 2 × 1.5 − 0 × 0 = 3; dot = 2 × 0 + 0 × 1.5 = 0; atan2(3, 0) = π ÷ 2. |
| Traced example, lines 160–194 | dt = 0.02; toGoal (3, 0); distance 3.0; headingError π ÷ 2 = 1.5708; speedNow 2.0; brakingDistance 3 − 2 × 0.05 − 0 = 2.9; √(12 × 2.9) = √34.8 = 5.899 → min(4, 5.899) = 4; KP × d = 18 → 4; vx = 3 ÷ 3 × 4 = 4, vy = 0; brakingAngle 1.5708 − 0 = 1.5708; turnSpeed 4.342; omega +4.342 (`copySign`). |
| Traced example, line 197 (`AccelerationLimiter.limit`) | `AccelerationLimiter.java`:47–86. ax = (4 − 2) ÷ 0.02 = 100; alpha = clamp(4.342 ÷ 0.02 = 217.1, ±8) = 8. Speeding up (100 × 2 > 0): wheel speed 2 + 0 × 0.3592 = 2 m/s; motor speed 2 ÷ 0.05504 × 7.3636 = 267.6 rad/s; R = 12 ÷ 476.1 = 0.02520 Ω, Kv = 605.8 ÷ 12 = 50.48; amps = min(150, (12 − 267.6 ÷ 50.48) ÷ 0.02520 = 265.8) = 150; Kt = 9.362 ÷ 476.1 = 0.019664; accel = 4 × 0.019664 × 150 × 7.3636 ÷ 0.05504 ÷ 60 = 26.31 → ax = 26.31. Grip: BASE_RADIUS = hypot(0.254, 0.254) = 0.3592; total = √(26.31² + (8 × 0.3592)²) = √(26.31² + 2.874²) = 26.46 > 10; scale 10 ÷ 26.46 = 0.3779; ax = 9.941, alpha = 3.023; new command (2 + 9.941 × 0.02, 0, 3.023 × 0.02) = (2.199, 0, 0.060). |
| Driving and turning share grip, scaled together so the path stays straight | `AccelerationLimiter.java`:67–75 and comment. |
| Seeding from the measured velocity avoids a command jump (and slip) at a moving start | Line 152; controllers study "The answer" point 1 ("no command steps from a moving start") and §1 (laws without it step the command and slip). |
| The goal is flipped in `initialize`, not the constructor | Lines 144–147 and comment 145; game-info skill. |
| Red flip: x → L − x, y → W − y, θ + 180° | `AllianceFlip.java`:64–72 (field size from the AprilTag layout). |
| Supplier goals are blue-origin and never flipped | Lines 89–95. |
| Protected constructor and `goal()` for subclasses | Lines 97–100, 136–139; `DriveToTag.java`:27, 44, 68–71; `DriveToPiece.java`:24, 39, 57–69. |
| `DriveToTag`: map goal first, then the tag | `DriveToTag.java`:14–26, 54–58, 68–71 (`tracker.getFieldGoal(tagToGoal).orElse(mapGoal)`). |
| `DriveToPiece`: 0.40 m short of the piece, facing it; holds still with no piece | `DriveToPiece.java`:27 (`INTAKE_REACH_METERS` 0.40), 59–68. |
| Options return `this` so they chain; Drive Then Align passes through staging, then `DriveToTag` | Lines 111–134; `DriveThenAlignOpMode.java`:53–63. |
| The template auto has two stopping legs, (2, 0, 0°) then (2, 1.5, 90°) | `AutonomousOpMode.java`:21, 28–35. |
| Closed-loop field velocity so slow final moves happen | Lines 68–73 and comment. |
| Claims the drivetrain | Line 104 (`super(name, drivetrain)`; `ClassicCommand.java`:29–32). |
| `end()`: Idle is not a stop, so send zero; waypoints send nothing | Lines 237–244 (comment line 241). |
| Logged keys | Lines 201–204; log-reading skill (`/RealOutputs/DriveToPose/CommandedVelocity`, `/RealOutputs/Drivetrain/Pose`). |
| Try it: run auto "2 - Drive To Pose" in the Java sim, open the newest log | run-sim skill (`./gradlew simulateJava`, or headless `./gradlew simulateJavaAgent '-Pmode=auto:2 - Drive To Pose'`); log-reading skill (`logs/akit_*.wpilog`). |
| KP 2 turns the legs into the race's "gentle P" robot | 2d < √(12d) ⇔ d < 3 m, so on the 2 m legs the KP line is always the smallest. Not run in the Java sim for this brief: narration says "should". |
| The Python simulator where the numbers were tuned | Line 35 comment; `tools/alignsim/README.md`. |
| The file is 245 lines | `wc -l DriveToPose.java`. |
| A thumb is about 2 cm wide; two pennies about 3 mm | Everyday sizes for scale (a US penny is 1.52 mm thick). |
