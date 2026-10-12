# Video 1: "How hard can a robot push?" (the physics in `AccelerationLimiter`)

A long-form lesson in 19 chapters (3,578 words, about 20 minutes of narration at 3 words a second;
about 23 minutes with the gaps and pause cards) that takes high-school
students from the problem to the real Java. **Goal:** the robot can't change its velocity as fast as the
code might ask, and slipping wheels make odometry lie. **The idea:** grip is a budget that moves with the
robot's weight, is shared with turning, and motors fade at speed. **The maths:** every rule derived from a
picture with this robot's real numbers, ending in one worked loop. **The code:** every line of
[`AccelerationLimiter.java`](../../../../src/main/java/frc/robot/utils/AccelerationLimiter.java) in file
order, with that worked loop shown as ghost values, then its one call site in `DriveToPose`. **Recap:**
four sentences, five check-yourself questions and a simulator experiment.

**Making a chapter.** In the animate project, with this folder copied in, run one chapter at a time:

```
/animate chapter 1 of the explainer in brief.md, use ch01-voice.json as the voice-over
```

Change both numbers for the other chapters (`chapter 7` … `ch07-voice.json`). The voice files are
generated from each chapter's numbered narration list, as in [../README.md](../README.md): each line
becomes `{ "id": "c01-1", "text": ..., "after": 0.6 }` (the last line 0.3). In chapter 19 the five
question lines get `"after": 5.0` for the pause cards. The older `ch1`–`ch5-voice.json` files belong to
the earlier 3-minute version and are replaced by `ch01`–`ch19`.

Beat times are a guide, computed at 3 words a second plus a 0.6 s gap per line and a 1.2 s lead. Each
beat matches one narration line, so animate can time it to its voice line. Within a beat, the Picture
cell lists the 2–3 changes that keep something new on screen every 2–4 s.

## Intake answers (same for every chapter)

| Question | Answer |
| --- | --- |
| **Subject** | The physics that decides how fast a swerve robot may speed up, slow down and turn (tire grip, weight shift, turning sharing grip, motors weakening at speed), and the 87-line Java class that applies it 50 times a second. |
| **The one true thing** | A robot can only push as hard as its tires grip, and that grip moves around, so every loop the code limits the *change* in velocity from physics (turning cap, then motors, then traction) instead of trusting the motors' current limit. |
| **Must include** | The numbers below, each derived on screen from a picture. The five-part path: goal, idea, maths, code, recap. The three checks in order. Every line of `AccelerationLimiter.java` with its real line number and the traced example's ghost values. The one call site in `DriveToPose`. |
| **Must avoid** | Claiming robot-tested results: everything is physics or "in our simulator". More than one new equation on screen at a time (write each on, fade the last). Re-teaching what frc5712.com covers (PID, swerve basics). Calling other teams' code "wrong". |
| **Formats and length** | 16:9 (the code chapters need the width; an optional 9:16 cut-down of Parts 1–2). 19 chapters of about 1–2 min: about 20 min of narration, 23 min with gaps and pause cards. |
| **Voice** | Narrated voice-over, one JSON per chapter (`ch01-voice.json` … `ch19-voice.json`), about 3 words a second. |
| **Look** | **"field-lab"** custom style, per [../STYLE.md](../STYLE.md): World mode (isometric robot and field on navy), Board mode (written-on maths, live readouts), Code mode (editor with real line numbers, a highlight bar and debugger-style ghost values). Colour code from STYLE.md: velocity blue, acceleration orange, turning purple, force/grip/weight red, time yellow, current/motor teal, distance green. |
| **Hero** | The robot from above and in isometric, as in STYLE.md: square frame, four swerve modules (wheel + steering ring), bumpers in the team colour with the team number (TODO: colour and number), two cameras on the front corners with lens LEDs. Same model in videos 0, 2 and 3. |
| **Music** | Composed in code, calm and curious. One motif per part (Part 4 adds a soft 50 Hz-ish tick under the code). A small sting at each chapter's key reveal; a held pad under the chapter 19 pause cards. |
| **Story format** | F5 fixed-hero journey for Parts 1–3 (the robot stays put while forces, arrows and graphs change around it); an F1-style three-station "machine" for the checks (chapter 7, reused in chapter 19); Part 4 is a code walkthrough with a small world or board inset beside the editor. |

## Numbers on screen (from this robot)

Use these exactly; the arithmetic is in the fact check at the end.

| Thing | On screen | Where it comes from |
| --- | --- | --- |
| Loop | 50 a second, dt = 0.02 s | robot loop; `DriveToPose.java:161` measures dt |
| Robot mass m | 60 kg | `ROBOT_MASS` (TODO: weigh it) |
| Push for 10 m/s² | 600 N, 150 N per wheel | 60 × 10; ÷ 4 |
| Load per wheel (standing still) | 147 N | 60 × 9.81 ÷ 4 |
| Grip number μ | about 1 | good tread on carpet (an assumption) |
| Grip limit | 9.81 ≈ 10 m/s² | μ·g; `TRACTION_ACCEL` |
| Wheel radius | 2.167 in = 0.05504 m | `TunerConstants` `kWheelRadius` |
| Gear ratio | 7.3636 : 1 | `TunerConstants` `kDriveGearRatio` |
| Kraken X60 FOC (CTRE dyno) | 9.362 N·m and 476.1 A stalled, 5785 rpm free | `DRIVE_MOTOR` |
| K_t, R, K_v | 0.01966 N·m/A · 0.0252 Ω · 50.48 rad/s per V | 9.362 ÷ 476.1 · 12 ÷ 476.1 · 605.8 ÷ 12 |
| Current to reach grip | about 56 A per wheel | 147 N × 0.05504 m ÷ 7.3636 ÷ 0.01966 N·m/A |
| Hardware current limit | 360 A (947 N per wheel, 6.4 × grip) | `kSlipCurrent`: a ceiling, not a plan |
| Planning current | 150 A | `MODEL_CURRENT`, motor check only |
| Module position x | 10 in = 0.254 m | `TunerConstants` module positions (±10 in) |
| Wheel to centre r | 0.359 m | `BASE_RADIUS` = hypot(0.254, 0.254) |
| Example CG height h | 0.20 m | an example; the real one is a TODO |
| Weight shift at 6 m/s² | ΔN 71 N: front 76 N, back 218 N | m·a·h ÷ (4x) |
| Weight shift at 10 m/s² | front 29 N, back 265 N (≈ 101 A) | same |
| Tip-over | 12.5 m/s² | g·x ÷ h |
| Equal-push limit | 5.5 m/s² | μg ÷ (1 + μh/x) |
| 60 A cap | 158 N per wheel | 60 × 2.63 N/A |
| Turning cap | 8 rad/s² | `ANGULAR_ACCEL` |
| Turning share of grip | 2.87 m/s², driving keeps 9.58 | 8 × 0.359; √(100 − 8.26) |
| Motor push vs speed | 26.3 m/s² up to 3.1 m/s, then 19.0 at 3.5, 9.7 at 4.0, 4.2 at 4.3, 0 at 4.53 m/s | `motorAccel` |
| Motors weaker than grip | above about 4 m/s | where the curve crosses 10 |
| Traced example | current (3.6, 0, 1.0), wanted (4.0, 0, 3.0), dt 0.02 → (3.79, 0, 1.15) | chapters 14–18 |

# Part 1: The goal

## Chapter 01: The jump the robot can't make (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:08 | World | **WIDE:** the whole field from high above, carpet sheen drifting. The hero robot sits near the blue wall, module LEDs idling. A yellow "20 ms" stopwatch in the corner ticks over and over; a small blue arrow flickers on the robot each tick. | **50 TIMES A SECOND** | The camera swoops down to the robot. | Soft tick motif; music in. |
| 2 | 0:08–0:13 | World | A blue arrow grows from the robot's nose (spring), labelled v; a purple curl for spin wraps around it; a blue swatch appears beside the label. | **v = velocity: speed + direction** | The arrows shrink back to nothing. | Small chime. |
| 3 | 0:13–0:20 | World | The robot is still (readout v = 0). A dashed blue "wanted" arrow, 4 m/s long, snaps out ahead. A "?" pops between the two. | **HAVE: 0 · WANT: 4 m/s** | The stopwatch zooms up to one 20 ms tick. | Questioning two-note. |
| 4 | 0:20–0:27 | Board | The floor tilts flat into axes (v up, t across). A near-vertical line climbs from 0 to 4 in 0.02 s; a slope readout counts up to 200. | **a = 4 ÷ 0.02 = 200 m/s²** | The slope line flashes orange. | Rising whistle. |
| 5 | 0:27–0:31 | Board | An orange "a" writes on with "how fast v changes"; orange and blue swatches sit side by side. | **a = acceleration (m/s²)** | The board tilts back into the world. | Chime. |
| 6 | 0:31–0:38 | World | **EXTREME CLOSE-UP:** one tire patch on carpet fibres. The wheel whips round, the fibres flatten, a puff of carpet dust; the tread slides. | **SLIP** | Fast pull-back. | Squeal and scuff. |
| 7 | 0:38–0:43 | World | All four wheels spin in place; the robot barely creeps. A red "slip" timer starts counting. | **SLIP: WORSE THAN SLOW** | The timer's digits blur. | Low warning tone. |
| 8 | 0:43–0:50 | Board | Over the faint field, a goal card writes on; a small station icon labelled AccelerationLimiter drops beside it. | **GOAL: THE BIGGEST CHANGE THE ROBOT CAN REALLY MAKE, EVERY LOOP** | Four tabs slide in under the card. | Key-reveal sting. |
| 9 | 0:50–0:56 | Board | The four tabs light in turn: IDEA · MATHS · CODE · RECAP. | **IDEA → MATHS → CODE → RECAP** | Tabs dim; a ghost robot peels off the hero (chapter 2). | One soft note per tab. |

**Narration (ch01-voice.json):**

1. Fifty times a second, our robot picks a velocity: how fast to drive, which way, and how fast to spin.
2. Velocity, written v, is speed with a direction. It's always blue.
3. Say the robot sits still, and the code suddenly wants four meters per second. Why not just ask for it?
4. To get there in one loop, twenty milliseconds, the robot would need two hundred meters per second squared.
5. That's acceleration, written a: how fast velocity changes. It's always orange.
6. No tire can push that hard. The motors spin up, the tread lets go, and the wheel spins on the carpet.
7. That's called slip. And slip isn't just slow. It's worse than slow.
8. So our goal for this video: find the biggest change the robot can really make, every loop, without slipping.
9. We'll get there in four steps: the idea, the maths, the code line by line, and a recap.

## Chapter 02: Spinning wheels lie (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:07 | World | The robot rolls forward slowly. **EXTREME CLOSE-UP:** a wheel hub with a tick counter; a green distance readout counts up. | **ODOMETRY = COUNTING WHEEL TURNS** | Pull back alongside the robot. | Odometer clicks. |
| 2 | 0:07–0:15 | World | Each wheel turn unrolls a green strip of carpet behind it; the strips stack into a green distance bar. | **1 TURN ≈ 35 cm** | The wheel speeds up. | Light clicks. |
| 3 | 0:15–0:22 | World | The wheel spins in place: the counter keeps ticking, the green bar keeps growing, the robot doesn't move. A translucent white ghost slides ahead to where the counter says; a red gap line stretches back to the real robot. | **GHOST = WHERE IT THINKS IT IS** | The camera rises. | Clicks over a squeal; "wrong answer" buzz. |
| 4 | 0:22–0:31 | World | **WIDE:** the field from above. A robot jumps its command in one loop; its ghost shoots ahead; a tape measure zips out to 1.5 m. Then a careful robot accelerates hard; a small gap opens; the tape reads 35 cm. | **ONE-LOOP JUMP: UP TO 1.5 m · CAREFUL HARD DRIVING: UP TO 35 cm (our simulator)** | Pan to a scoring spot below an AprilTag. | Tape-measure zip; softer buzz. |
| 5 | 0:31–0:36 | World | A 2 cm circle on the carpet; the tag on the wall (white, thin team-colour outline). | **TARGET: WITHIN 2 cm** | Push in from behind the robot's camera. | One precise tick. |
| 6 | 0:36–0:43 | World | The camera's view cone slides off the tag and greys out; the robot keeps creeping in, steered by its wheel counter. | **TAG OUT OF VIEW → ODOMETRY ONLY** | The ghost lands dead on the circle. | Lens LED blinks off. |
| 7 | 0:43–0:48 | World | The ghost sits perfectly on target with a green check; the real robot stops beside it, off the circle; a red miss marker pulses. | **PARKED WRONG, THINKS IT'S RIGHT** | Rewind (reverse motion streaks). | Miss thud. |
| 8 | 0:48–0:55 | World | Rewound: the robot speeds up within its grip, wheels planted, ghost and robot overlapping exactly. | **PUSH HARD, NEVER PAST THE GRIP** | Push into a drive motor (chapter 3). | Settling chord. |

**Narration (ch02-voice.json):**

1. Here's why. The robot tracks where it is mostly by counting wheel turns. That's called odometry.
2. Each wheel turn rolls out about thirty-five centimeters of carpet. Add up the turns, and you know how far you went.
3. But a slipping wheel turns without moving the robot. The count grows anyway, so the robot thinks it's somewhere it isn't.
4. In our simulator, a one-loop command jump over-counted by up to one and a half meters. Even careful hard driving cost up to thirty-five centimeters.
5. Now remember the job: stop on a scoring spot within two centimeters.
6. Close to the target, the camera can lose sight of the tag. Then the robot finishes on odometry alone.
7. If odometry is wrong, the robot parks in the wrong place, and thinks it's right.
8. So we want the robot to push as hard as it can, but never harder than its tires can grip.

## Chapter 03: A ceiling, not a plan (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:10 | World | Push into a module: the drive motor glows teal. A big dial labelled "current limit" floats over it; a hand reaches for the knob to turn it down. | **EASY FIX? TURN THE CURRENT DOWN** | The dial fills the frame. | Knob click. |
| 2 | 0:10–0:14 | World | A teal "I" writes on beside the dial with "amps (A)"; a teal swatch. | **I = current, in amps (A)** | The needle swings up. | Chime. |
| 3 | 0:14–0:22 | World | **EXTREME CLOSE-UP:** the dial face. A small mark at 56 A says "grip reached"; the needle sits at 360 A, labelled kSlipCurrent; the gap between them shades in. | **kSlipCurrent = 360 A** | Pull back to a side view of the robot. | Needle thunk. |
| 4 | 0:22–0:30 | World | Side view: the robot speeds up; red load bars over the wheels: back bars grow, front bars shrink. A "2026" stamp lands in the corner. | **2026: WEIGHT SHIFTS BACK** | Hold on the tall back bars. | Seesaw creak. |
| 5 | 0:30–0:37 | World | The back wheels' big grip rings sit half empty while a low "60 A" bar slams down on all four push arrows, chopping them to the same small size. | **A LOW LIMIT CHOKES THE HEAVY WHEELS** | Tilt to the board. | Choked motor groan. |
| 6 | 0:37–0:47 | Board | Two pairs of bars (CG 20 cm, accel 10): time 1.26 s vs 1.70 s (yellow); odometry error 4.4 cm vs 7.4 cm (green). Labels: 120 A (same as 360 A) vs 60 A. | **60 A: +0.44 s, 1.7 × ODOMETRY ERROR (our simulator)** | Bars fade. | Two low notes. |
| 7 | 0:47–0:53 | World | The 360 A dial melts into a safety net far below a high wire; the robot drives along the wire. | **360 A IS A CEILING, NOT A PLAN** | The camera pushes into the robot's "brain". | Settling chord. |
| 8 | 0:53–1:00 | Code | Preview: an editor panel unfolds from the brain showing line 25, `public final class AccelerationLimiter {`; it folds away and the camera pulls out. **WIDE:** the field; four cards appear around the robot: GRIP · WEIGHT · TURNING · MOTORS. | **THE PLAN: AccelerationLimiter · FOUR IDEAS** | Dive toward a tire (chapter 4). | Key-reveal sting; one note per card. |

**Narration (ch03-voice.json):**

1. There's an easy-looking fix. Every drive motor has a current limit. Less current, less push. So why not set it low and never slip?
2. Current, written I, is measured in amps, and drawn teal.
3. Our limit is k slip current, in Tuner Constants: three hundred sixty amps. That's far more than a wheel needs to break loose.
4. In twenty twenty-six, our team learned why it stays high. When the robot speeds up, its weight shifts onto the back wheels.
5. Heavier wheels could push harder. But a low current limit chops every wheel down to the same small push.
6. Our simulator agrees. With the center of mass twenty centimeters up, sixty amps was zero point four four seconds slower, with one point seven times the odometry error.
7. So the current limit is only a ceiling, a safety net. It is not the plan.
8. The plan lives in a class called Acceleration Limiter. To build it, we need four ideas: grip, weight, turning, and motors.

# Part 2: The idea

## Chapter 04: Grip is a budget, and the budget moves (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:10 | World | A heavy crate on carpet; a hand pushes; the orange push arrow grows past a dashed line, then the crate slides. The crate morphs into a tire. **EXTREME CLOSE-UP:** its contact patch on carpet fibres, red floor arrows pointing forward, back and sideways, each stopping at a faint red ring. | **THE FLOOR GRIPS BACK, UP TO A POINT** | Pull back a little. | Scrape; rubber squeak. |
| 2 | 0:10–0:16 | World | A weight block presses the tire down (red down arrow); the red ring grows as the block grows, shrinks as it shrinks. | **PRESS HARDER → MORE GRIP** | The ring holds steady. | Tone up, tone down. |
| 3 | 0:16–0:24 | World | The ring becomes a budget gauge over the wheel: a red meter labelled GRIP BUDGET; a red swatch "F, N" ticks in beside it. | **GRIP = A BUDGET · RED = FORCE, GRIP, WEIGHT** | The meter ticks. | Coin clink. |
| 4 | 0:24–0:30 | World | Three arrows take bites from the meter: orange "speed up", orange "brake", purple "turn". It empties; the tire slips with a red puff. | **SPEED UP · BRAKE · TURN · OVERSPEND → SLIP** | Freeze, then pull out. | Three clinks; squeal. |
| 5 | 0:30–0:34 | World | **WIDE:** the robot on the field with a budget meter over all four wheels. Two meters shrink and two grow, sliding like liquid between them. | **THE BUDGET MOVES** | Match-cut to a bus. | Curious motif; whoosh. |
| 6 | 0:34–0:41 | World | An isometric bus pulls away; a standing rider's feet go with it while the body lags back. | **FEET GO, BODY LAGS** | Match-cut to the robot's side view. | Bus engine. |
| 7 | 0:41–0:48 | World | Side view: a checkered centre-of-mass dot above the floor; the robot speeds up and a red weight arrow slides toward the back wheels. | **SPEED UP → WEIGHT GOES BACK** | Load bars appear. | Seesaw creak. |
| 8 | 0:48–0:55 | World | Load bars and grip rings: back rings grow, front rings shrink. | **BACK: BIG BUDGET · FRONT: SMALL** | The robot brakes. | Creak. |
| 9 | 0:55–0:57 | World | Braking: front rings grow, back rings shrink. | **BRAKE → WEIGHT GOES FORWARD** | Back to speeding up. | Creak reverses. |
| 10 | 0:57–1:04 | World | Four equal orange push arrows; the small front rings overflow (slip puffs) while the big back rings are half used. | **EQUAL PUSH: FRONT SLIPS FIRST** | The robot starts to spin (chapter 5). | Squeal from the front. |

**Narration (ch04-voice.json):**

1. Push a heavy box across carpet, and the floor grips back, up to a point. A tire works the same way, forwards, backwards or sideways.
2. How hard it grips depends on how hard the tire is pressed down. Press harder, more grip.
3. So think of grip as a budget, set by the weight on each tire. Forces, grip and weight are always red.
4. Speeding up, braking and turning all spend it. Spend more than you have, and the tire slips.
5. Here's the twist: the budget doesn't stay put.
6. When a bus pulls away, standing riders lean back. Their feet go with the bus, but their bodies lag behind.
7. A robot does the same. Its weight sits above the floor, so speeding up shifts weight onto the back wheels.
8. Back tires get pressed harder and get a bigger budget. Front tires get lighter and a smaller one.
9. Braking does the opposite.
10. The catch: our four modules push about the same amount. So the light front wheels run out of grip first.

## Chapter 05: Turning shares the budget (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:06 | World | **WIDE:** top view of the field; the robot starts to spin; each wheel's path traces a purple circle around the robot's centre. | **SPIN: EVERY WHEEL GOES AROUND** | Hold on the spin. | Whirr. |
| 2 | 0:06–0:12 | Board | A purple ω writes on: "omega: how fast it spins (rad/s)"; purple swatch. | **ω = spin rate (rad/s)** | Back to the world. | Chime. |
| 3 | 0:12–0:20 | World | Steady spin: the budget meters barely move. Then the spin speeds up: each wheel grows a purple sideways arrow and its meter drains. | **STEADY SPIN: CHEAP · CHANGING SPIN: COSTS GRIP** | Push in on one wheel. | Whirr rises. |
| 4 | 0:20–0:26 | World | **EXTREME CLOSE-UP:** one tire patch: an orange forward arrow and a purple sideways arrow both drawing from the same red ring. | **SAME RUBBER, SAME CARPET, ONE BUDGET** | The ring flattens onto the floor as a circle. | Ring chime. |
| 5 | 0:26–0:33 | World | Top-down: the red circle around the wheel; an orange driving arrow; a purple spin-up arrow at a right angle to it. | **DRIVING ⟂ SPIN-UP** | The two arrows slide into a right triangle. | Click. |
| 6 | 0:33–0:39 | Board | Right triangle: orange and purple legs; the red long side must end inside the circle; then it pokes out. | **LONG SIDE MUST FIT** | Try a fix. | Warning blip. |
| 7 | 0:39–0:44 | World | Bad fix: only the orange arrow shrinks; the robot's dashed trail curves off its line. | **SHRINK ONE → PATH BENDS** | Rewind. | "Wrong" buzz. |
| 8 | 0:44–0:51 | World | Good fix: both arrows shrink by the same fraction; the triangle keeps its shape; the trail stays straight. | **SHRINK BOTH TOGETHER → SAME DIRECTION** | The spin jerks. | Clean click. |
| 9 | 0:51–0:56 | World | A sudden spin step jerks every wheel sideways with sparks; then a gentle ramp, held by a purple clamp labelled "cap". | **SPIN CHANGES GET A CAP** | Push into a motor (chapter 6). | Jerk, then smooth whirr. |

**Narration (ch05-voice.json):**

1. Now add turning. When the robot spins, every wheel travels around a circle.
2. Omega, a Greek letter like a curly w, is how fast the robot spins. Turning is purple.
3. Steady spinning is cheap. Changing the spin costs grip, because the wheels must push sideways to speed the spin up.
4. And that sideways push uses the same rubber on the same carpet as driving. One budget.
5. Draw the budget as a circle. Driving is one arrow. Speeding up the spin is another, at a right angle.
6. Together they make a right triangle, and its long side has to fit inside the circle.
7. If it doesn't fit, shrinking just the driving arrow would bend the robot's path.
8. So we shrink both arrows by the same fraction. The robot keeps its direction, just a little gentler.
9. Sudden spin changes caused most slip in our simulator, so they get their own cap.

## Chapter 06: Motors get tired at speed (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:10 | World | The camera pushes into a Kraken motor (teal glow). **EXTREME CLOSE-UP:** a magnet spinning past copper coils. Then a hand cranks the shaft and a bulb wired to it lights up. | **EVERY MOTOR IS ALSO A GENERATOR** | The bulb glows brighter. | Low hum; click, glow buzz. |
| 2 | 0:10–0:16 | World | The motor wired to a 12 V battery; as it spins faster, a teal "back-EMF" arrow grows, pointing against the battery's arrow. | **BACK-EMF PUSHES BACK** | The two arrows race. | Hum rises. |
| 3 | 0:16–0:23 | World | Current as teal dots flowing in the wire thin out as back-EMF grows; a twist arrow on the shaft, labelled torque, shrinks with them. | **FASTER → LESS CURRENT → LESS TORQUE (TWIST)** | The twist fades. | Fewer crackles; hum steady. |
| 4 | 0:23–0:31 | World | At top speed the back-EMF arrow matches the 12 V arrow; the dots stop; the twist arrow is gone. | **TOP SPEED: 12 V vs 12 V → NO PUSH LEFT** | Cut to a playground. | Hum plateau. |
| 5 | 0:31–0:38 | World | A merry-go-round: a kid shoves hard while it's still; when it spins fast, their hands slap at it and barely keep up. | **FAST → HARD TO ADD PUSH** | Back to the field. | Whoosh, playground laugh. |
| 6 | 0:38–0:43 | World | **WIDE:** the robot crosses the field. Slow: its tire rings glow (grip limits). Near top speed: its motors glow instead. | **SLOW: GRIP LIMITS · FAST: MOTORS LIMIT** | The robot brakes. | Switch click. |
| 7 | 0:43–0:49 | World | Braking: the back-EMF arrow flips to point the same way as the battery's; only the tire rings glow. | **BRAKING: GRIP ONLY** | Push into the robot's brain (chapter 7). | Chord resolves. |

**Narration (ch06-voice.json):**

1. Inside each motor, a magnet spins past coils of wire. Spin one by hand, and it makes electricity. Every motor is also a generator.
2. So while it spins, it pushes voltage back against the battery. That's called back E M F.
3. The faster it spins, the harder it pushes back. Less current flows, so the shaft gets less twist, called torque.
4. At top speed, the push-back matches the battery's twelve volts. No current flows, so there's no push left at all.
5. It's like a playground merry-go-round. Standing still, you can shove it hard. Spinning fast, your hands can barely keep up.
6. So at low speed, the tires are the limit. Near top speed, the motors are.
7. Braking is different. There, back E M F helps instead of fighting, so only grip limits braking.

## Chapter 07: Three checks, fifty times a second (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:08 | World | Inside the robot's brain: two tokens arrive, a solid blue "current" and a dashed blue "wanted"; a yellow 20 ms clock ticks above them. | **EVERY 20 ms: CURRENT + WANTED** | The tokens line up. | Tick motif. |
| 2 | 0:08–0:15 | Board | A v-vs-t graph: a clamp tries the height of the line (crossed out), then grips its slope instead, one small orange step per tick. | **LIMIT THE CHANGE, NOT THE SPEED** | The slope highlights. | Click. |
| 3 | 0:15–0:22 | World | A robot cruising steadily: its grip meters barely used. It speeds up: the meters fill. | **CHANGE NEEDS PUSH · CRUISING DOESN'T** | Back to the graph. | Whoosh. |
| 4 | 0:22–0:30 | Board | A staircase: each step 0.2 m/s tall (blue) and 0.02 s wide (yellow). **EXTREME CLOSE-UP** on one step and its labels. | **10 m/s² × 0.02 s = 0.2 m/s PER LOOP** | The steps blur into a ramp. | One tick per step. |
| 5 | 0:30–0:34 | World | A machine with three stations appears in the brain; the wanted token waits at the entry. | **THREE CHECKS, ALWAYS IN THIS ORDER** | The token rolls to station 1. | Conveyor rumble. |
| 6 | 0:34–0:38 | World | Station 1, a purple clamp, squeezes the spin part of the token. | **1. TURNING CAP** | Roll on. | Clamp clunk. |
| 7 | 0:38–0:44 | World | Station 2, a teal motor gauge, trims the driving part; a sign reads "only when speeding up". | **2. MOTORS (WHEN SPEEDING UP)** | Roll on. | Gauge tick. |
| 8 | 0:44–0:50 | World | Station 3, a red ring: the token's driving + turning triangle must fit; it shrinks evenly until it does. | **3. TRACTION: SHRINK BOTH** | The ring glows. | Ring chime. |
| 9 | 0:50–0:56 | World | A referee's whistle hangs over station 3; it is the only station wired to both the driving and turning parts. | **TRACTION IS THE REFEREE** | The token exits. | Whistle. |
| 10 | 0:56–0:59 | World | The token leaves as "command", runs down to the four modules. **WIDE:** the robot moves off across the field. | **WHAT SURVIVES = THIS LOOP'S COMMAND** | Tilt to the board. | Success tone. |
| 11 | 0:59–1:05 | Board | Green bars: odometry error with one fixed limit vs with the three checks, 15–30% shorter. | **15–30% LESS ODOMETRY ERROR (our simulator)** | The bars slide away. | Two notes. |
| 12 | 1:05–1:08 | Board | The world tilts flat into the board; fresh axes draw in. | **NOW: THE NUMBERS** | Into Part 3. | Transition whoosh. |

**Narration (ch07-voice.json):**

1. Now the plan. Every twenty milliseconds, the code gets two velocities: the one we have, and the one we want.
2. It doesn't limit the velocity. It limits the change: how far the velocity may move in this one loop.
3. Because changing velocity is what takes a push. Cruising at a steady speed costs almost no grip.
4. At ten meters per second squared, that's two tenths of a meter per second per loop. Small steps, fifty times a second.
5. Each change goes through three checks, always in this order.
6. One, the turning cap: the spin may only change so fast.
7. Two, the motors: when speeding up, ask no more than they can still add at this speed.
8. Three, traction: driving and turning together must fit inside the grip. If not, shrink both.
9. Traction goes last, like a referee, because it's the only check that sees driving and turning together.
10. Whatever survives becomes this loop's command.
11. In our simulator, these checks left fifteen to thirty percent less odometry error than one fixed limit.
12. Now let's put real numbers on every check.

# Part 3: The maths

## Chapter 08: Push and grip, in numbers (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:06 | Board | "F = m · a" writes on: F red, m red, a orange. | **F = m · a** | Hold. | Soft chime. |
| 2 | 0:06–0:11 | Board | Units write on under each symbol. | **F (N) · m (kg) · a (m/s²)** | The robot drops onto a scale in a world inset. | Three ticks. |
| 3 | 0:11–0:18 | World | The robot sits on a bathroom-style scale reading 60 kg; a "TODO: weigh it" tag flutters from the bumper. | **ROBOT_MASS = 60 kg (TODO: weigh it)** | The scale fades. | Scale beep. |
| 4 | 0:18–0:24 | Board | 60 × 10 = 600 writes on; in a world inset a big red arrow grows from the robot's back, then splits into four red arrows at the tire patches, 150 N each. | **60 × 10 = 600 N · ÷ 4 = 150 N PER WHEEL** | Push into one tire. | Thud; four light thuds. |
| 5 | 0:24–0:29 | Board | "grip ≤ μ · N" writes on, the previous line fading. | **GRIP ≤ μ · N** | Zoom to N. | Chime. |
| 6 | 0:29–0:35 | World | **EXTREME CLOSE-UP:** the floor pushing up on a tire: a red up arrow N under the patch. | **N = HOW HARD THE FLOOR PUSHES UP** | Pull out. | Low thump. |
| 7 | 0:35–0:43 | Board | N = 60 × 9.81 ÷ 4 = 147 writes on. | **N = 60 × 9.81 ÷ 4 = 147 N** | μ appears. | Ticks. |
| 8 | 0:43–0:51 | World | Tread on carpet; a μ dial swings to "about 1". | **μ ≈ 1 (good tread on carpet)** | Back to the board. | Dial click. |
| 9 | 0:51–1:00 | Board | 147 N ÷ 15 kg = 9.81 ≈ 10 writes on (15 kg labelled "one wheel's share of 60 kg"); the 10 lifts off and lands on line 28 of a code card, `TRACTION_ACCEL = 10.0`. **WIDE:** a world inset of the robot launching across the field with a "1 g" tag. | **147 ÷ 15 = 9.81 ≈ 10 m/s² = TRACTION_ACCEL ≈ 1 g** | The card flies back to the board. | Rising tone; key-reveal sting. |
| 10 | 1:00–1:06 | Board | μ · (m g ÷ 4) ÷ (m ÷ 4): the two m's strike out, leaving μ · g. | **a_max = μ · g (THE MASS CANCELS)** | Follow a red arrow up into the wheel (chapter 9). | Satisfying click. |

**Narration (ch08-voice.json):**

1. Let's start with the most famous rule in physics: F equals m times a.
2. F is force, in newtons. m is mass, in kilograms. a is acceleration.
3. Our robot is sixty kilograms. In the code that's robot mass, still waiting for a real weigh-in.
4. To speed up at ten: sixty times ten, six hundred newtons, or one hundred fifty per wheel.
5. Now, grip. The most a tire can push is mu times N.
6. N is the normal force: how hard the floor pushes up, the tire's share of the weight.
7. Weight is mass times g, gravity's pull, nine point eight one. Sixty times nine point eight one, over four: one hundred forty-seven newtons.
8. Mu, a Greek letter like a u with a tail, is the grip number. For good tread on carpet, it's about one.
9. So one tire can push one hundred forty-seven newtons to move its fifteen kilograms: nine point eight one, about ten. That's traction accel, about one g.
10. Notice the mass canceled out. A heavier robot needs more push, but also grips harder.

## Chapter 09: From newtons to amps (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:07 | World | **WIDE:** the robot on the field, then the camera follows a red force arrow backwards from a tire patch, up through the wheel, the gearbox and into the motor. | **FLOOR → WHEEL → GEARBOX → MOTOR** | Stop at the wheel. | Whoosh along the path. |
| 2 | 0:07–0:13 | World | The wheel as a lever: a green radius from axle to tread; a twist arrow at the axle. | **TORQUE = FORCE × RADIUS** | A board panel slides in. | Creak. |
| 3 | 0:13–0:21 | Board | 2.167 in = 0.05504 m, then 147 × 0.05504 = 8.10 writes on. | **147 N × 0.05504 m = 8.10 N·m** | The gearbox opens. | Ticks. |
| 4 | 0:21–0:26 | World | **EXTREME CLOSE-UP:** inside the gearbox, a small gear turning a big one; a tag reads 7.3636 : 1. | **GEAR RATIO 7.3636 : 1** | The gears spin. | Gear whirr. |
| 5 | 0:26–0:30 | Board | 8.10 ÷ 7.3636 = 1.10 writes on. | **8.10 ÷ 7.3636 = 1.10 N·m AT THE MOTOR** | A teal motor slides in. | Tick. |
| 6 | 0:30–0:37 | Board | K_t = 9.362 ÷ 476.1 = 0.01966 writes on in teal. | **K_t = 0.01966 N·m PER AMP** | An amps meter appears. | Chime. |
| 7 | 0:37–0:42 | Board | 1.10 ÷ 0.01966 = 56; the meter needle settles on 56 A. | **≈ 56 A PER WHEEL TO REACH GRIP** | The meter zooms out. | Key-reveal sting. |
| 8 | 0:42–0:52 | World | The dial again: a 56 A mark, the needle at 360 A; a red bar shows 360 A could push 947 N, next to the 147 N grip. Cut to the robot launching with smoking wheels under a "360 A" banner that never moves. | **360 A → 947 N = 6.4 × GRIP: THE CEILING NEVER STOPS A SLIP** | Back to the board. | Low tone; squeal. |
| 9 | 0:52–1:00 | Board | A teal line at 150 A drops between 56 and 360 on the dial scale. | **MODEL_CURRENT = 150 A (PLANNING ONLY)** | Swing to a side view (chapter 10). | Settling chord. |

**Narration (ch09-voice.json):**

1. How much current does one hundred forty-seven newtons take? Follow the push back to the motor.
2. The wheel is a lever. The twist at the axle, called torque, is force times wheel radius.
3. Our wheel radius is zero point zero five five oh four meters. That gives eight point one oh newton meters.
4. A gearbox, seven point three six three six to one, multiplies the motor's torque.
5. So the motor needs only one point one oh newton meters.
6. A Kraken makes zero point zero one nine six six newton meters per amp. That's K t, its torque constant.
7. One point one oh divided by that: about fifty-six amps per wheel.
8. At the three hundred sixty amp ceiling, a wheel could push nine hundred forty-seven newtons, six point four times its grip. That ceiling never stops a slip.
9. For the motor check, the code plans with one hundred fifty amps, called model current. Under the ceiling, but plenty to reach grip.

## Chapter 10: Why weight moves, on the board (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:05 | World | **WIDE:** the field; the camera swings round to a side view of the robot, and the floor line becomes the board's x-axis. | **SIDE VIEW** | Hold. | Whoosh. |
| 2 | 0:05–0:12 | Board | A checkered centre-of-mass dot; a green height h down to the floor; a "TODO: measure" tag. | **h = 20 cm = 0.20 m (EXAMPLE · TODO: MEASURE)** | The axles light up. | Tick. |
| 3 | 0:12–0:20 | Board | Green x from the dot's vertical line to the front axle, and again to the back axle. | **x = 10 in = 0.254 m** | An acceleration arrow appears. | Tick. |
| 4 | 0:20–0:26 | Board | An orange a arrow at the dot; a red floor-push arrow, m · a, at floor level, h below the dot. | **FLOOR PUSH = m · a, h BELOW THE BALANCE POINT** | A curved arrow forms. | Thud. |
| 5 | 0:26–0:34 | Board | A curved red "moment" arrow around the dot; "force × lever arm" writes on, then m · a · h. | **MOMENT = m · a · h** | A ghost of the robot starts to tip. | Creak. |
| 6 | 0:34–0:40 | World | A translucent ghost tips nose-up; the real robot stays flat on all four wheels. | **IT WANTS TO TIP BACK** | Load arrows appear. | Seesaw creak. |
| 7 | 0:40–0:47 | Board | Red up-arrows at the wheels: each back one grows by ΔN, each front one shrinks by ΔN. | **BACK +ΔN · FRONT −ΔN** | A second moment writes on. | Two tones. |
| 8 | 0:47–0:54 | Board | 4 · ΔN · x writes on beside the four arrows. | **BALANCING MOMENT = 4 · ΔN · x** | An equals sign slides between the two moments. | Click. |
| 9 | 0:54–0:59 | Board | m · a · h = 4 · ΔN · x rearranges (sigmoid) into ΔN = m · a · h ÷ (4x); a box draws around it. | **ΔN = m · a · h ÷ (4 x)** | Box glows. | Key-reveal sting. |
| 10 | 0:59–1:07 | Board | The full load formula writes on; "back +, front −" tags. | **N = m g ÷ 4 ± m a h ÷ (4 x)** | An a slider appears. | Chime. |
| 11 | 1:07–1:14 | Board | Slider at a = 6: ΔN = 71; red bars count to front 76, back 218. **EXTREME CLOSE-UP** in an inset: a front tire's grip ring shrinking. | **a = 6: ΔN 71 N · FRONT 76 N · BACK 218 N** | Slider to 10. | Bars creak. |
| 12 | 1:14–1:19 | Board | Slider at a = 10: front 29, back 265. | **a = 10: FRONT 29 N · BACK 265 N** | The slider keeps going. | Low tone. |
| 13 | 1:19–1:25 | Board | The front bar hits zero at 12.5; a dashed tip-over line; the ghost lifts its nose. | **TIP-OVER: g · x ÷ h = 12.5 m/s²** | Top view of four equal arrows (chapter 11). | Warning tone. |

**Narration (ch10-voice.json):**

1. Why does weight move? Look at the robot from the side.
2. Its balance point, the center of mass, sits at height h. We'll use twenty centimeters as an example.
3. The wheels sit a distance x in front and behind. Our modules are ten inches out: zero point two five four meters.
4. Speeding up, the floor pushes the tires forward with m times a, h below the balance point.
5. A push below the balance point makes a turning effect, a moment: force times lever arm, here m times a times h.
6. It tries to tip the robot back, nose up. The robot doesn't tip, so something balances it.
7. The wheel loads do. Each back wheel gains an amount, delta N. Each front wheel loses the same.
8. Four wheels, each delta N, each x from the middle: a balancing moment of four times delta N times x.
9. Set the moments equal: delta N equals m a h, over four x.
10. So each wheel carries m g over four, plus delta N at the back, minus delta N at the front.
11. At six meters per second squared, delta N is seventy-one newtons. Front wheels carry seventy-six, back wheels two hundred eighteen.
12. At ten, the front wheels carry only twenty-nine newtons. Their budget is almost gone.
13. At zero front load, the robot tips: g x over h, twelve point five meters per second squared.

## Chapter 11: Equal push, uneven grip (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:06 | World | Top view: each module shows a blue speed dial at the same reading and an equal orange push arrow. | **SPEED-CONTROLLED → ABOUT EQUAL PUSH** | Tilt to the board. | Tick. |
| 2 | 0:06–0:11 | Board | "each wheel: m · a ÷ 4" writes on. | **EACH WHEEL PUSHES m · a ÷ 4** | An inequality forms. | Chime. |
| 3 | 0:11–0:16 | Board | m · a ÷ 4 > μ · N_front writes on; N_front glows red. | **FRONT SLIPS WHEN m a ÷ 4 > μ N_front** | Algebra steps. | Tick. |
| 4 | 0:16–0:23 | Board | N_front from chapter 10 slides in; each algebra step writes on and the last one fades; the result is boxed. | **a = μ g ÷ (1 + μ h ÷ x)** | The numbers drop in. | One click per step. |
| 5 | 0:23–0:27 | Board | 9.81 ÷ (1 + 0.787) = 5.5 writes on. | **9.81 ÷ 1.787 = 5.5 m/s²** | The 5.5 glows. | Key-reveal sting. |
| 6 | 0:27–0:33 | World | **WIDE:** the robot accelerating at 10 across the field; short skid marks behind the front wheels; the back rings half full. | **ABOVE 5.5 (THIS CG): FRONT WHEELS SLIP** | Push into a back wheel. | Squeal. |
| 7 | 0:33–0:41 | World | **EXTREME CLOSE-UP:** a back tire patch with a big red ring labelled 265 N; a teal meter reads 101 A. | **BACK WHEEL AT 10: 265 N ≈ 101 A** | A dashed "command" ghost pulls slightly ahead. | Hum. |
| 8 | 0:41–0:50 | World | The robot lags its command ghost; the back modules' speed dials see the gap and their push arrows grow. | **FRONT SLIPS → BACK MUST PUSH HARDER** | A "60 A" bar slams down. | Rising hum. |
| 9 | 0:50–0:57 | World | The 60 A bar caps the back arrows at 158 N, well short of their rings. | **60 A → 158 N CAP: BACK WHEELS STARVED** | The bar lifts. | Choked groan, release. |
| 10 | 0:57–1:05 | Board | Card: plan with 10 anyway; a small bar chart: tag alignment 96–99% (CG 0.2 m, accel 10). | **TAG ALIGNMENT STILL 96–99% (our simulator)** | The robot starts to spin (chapter 12). | Two notes. |

**Narration (ch11-voice.json):**

1. Here's the problem. Our modules are speed-controlled, so they all push about the same.
2. Equal push means each wheel pushes a quarter of m times a.
3. A front wheel slips when that's more than mu times its load.
4. Put in the front load, solve for a, and you get mu g over one plus mu h over x.
5. With our numbers, that's five point five meters per second squared.
6. So above five point five, the light front wheels slip, though the whole robot could grip ten.
7. The heavy back wheels have grip to spare: at ten, two hundred sixty-five newtons each. That takes about one hundred one amps.
8. When a front wheel slips, the robot falls behind its command, and the back modules push harder, if the current limit lets them.
9. Sixty amps caps every wheel at one hundred fifty-eight newtons, so the back wheels can't help. That's the twenty twenty-six lesson.
10. Our code still plans with ten. In our simulator, some wheels slipped, but tag alignment still succeeded ninety-six to ninety-nine percent.

## Chapter 12: The friction circle (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:09 | Board | Top view: the robot spins up; each wheel grows a purple arrow along its circle. A purple α writes on: "alpha: how fast the spin changes". | **α = spin change (rad/s²)** | A green r draws in. | Whirr; chime. |
| 2 | 0:09–0:17 | Board | Green r from the centre to one wheel; α · r writes on beside that wheel's purple arrow. | **WHEEL'S SIDEWAYS ACCEL = α · r** | A right triangle forms on r. | Tick. |
| 3 | 0:17–0:24 | Board | Legs 0.254 and 0.254, long side r = 0.359. **EXTREME CLOSE-UP:** the module's corner marker, "±10 in". | **r = √(0.254² + 0.254²) = 0.359 m** | r slides into α · r. | Click. |
| 4 | 0:24–0:30 | Board | 8 × 0.359 = 2.87 writes on in purple. | **α · r = 8 × 0.359 = 2.87 m/s²** | A red circle draws in. | Tick. |
| 5 | 0:30–0:38 | Board | Red circle of radius 10; the purple leg 2.87; an orange leg; the long side touching the circle. | **√(DRIVING² + (α r)²) ≤ 10** | Solve for driving. | Ring chime. |
| 6 | 0:38–0:44 | Board | √(10² − 2.87²) = √(100 − 8.26) = 9.58 writes on. | **DRIVING ≤ √(100 − 8.26) = 9.58 m/s²** | 9.58 glows next to 10. | Key-reveal sting. |
| 7 | 0:44–0:48 | Board | A bar shrinks from 10 to 9.58; "−4%" tag. | **A HARD SPIN-UP COSTS ONLY ~4% OF DRIVING** | A too-big triangle appears. | Soft click. |
| 8 | 0:48–0:57 | Board | A triangle with legs 10 and 2.87 pokes outside the circle: long side 10.40; scale = 10 ÷ 10.40 = 0.961 writes on. | **SCALE = 10 ÷ LONG SIDE = 10 ÷ 10.40 = 0.961** | Both legs shrink. | Warning blip; tick. |
| 9 | 0:57–1:04 | Board | Both legs × 0.961 (9.61 and 2.76): the triangle keeps its shape. **WIDE:** a world inset of the field shows the robot's trail staying straight. | **SAME SCALE → SAME DIRECTION** | A slip pie chart appears. | Clean click. |
| 10 | 1:04–1:13 | Board | A pie: about 80% of slip time while turning, from spin steps. Then a purple dial turns from 20 to 8; a slip bar shrinks to a sixth; a stopwatch doesn't move. | **~80% OF SLIP WHILE TURNING · ANGULAR_ACCEL 20 → 8 (ANGULAR_DECEL 8 → 6): ~6 × LESS SLIP, NO TIME LOST (our simulator)** | Push into a motor (chapter 13). | Low tone; settling chord. |

**Narration (ch12-voice.json):**

1. Now the turning share. Alpha, a Greek letter like a little fish, is how fast the spin changes, in radians per second squared.
2. When the spin speeds up, a wheel a distance r from the center speeds up sideways by alpha times r.
3. Our wheels are ten inches forward and ten to the side. By Pythagoras, r is zero point three five nine meters.
4. With our turning cap of eight, that's two point eight seven meters per second squared.
5. Driving and spin-up push at right angles, so they add like a right triangle. The long side must stay under ten.
6. So driving still gets the square root of one hundred minus eight point two six: nine point five eight.
7. A hard spin-up costs only four percent of the drive.
8. Too big? The code finds one scale: ten over the long side. With ten and two point eight seven, that's zero point nine six one.
9. Both parts get the same scale. The triangle keeps its shape, so the robot keeps its direction.
10. In our simulator, about eighty percent of slip came from sudden spin steps. Lowering the turning limits cut it about six times, at no cost in time.

## Chapter 13: The motor curve (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:06 | World | **WIDE:** the robot near top speed on the field, motors glowing teal; the camera dives into one motor. | **HOW MUCH CAN THE MOTORS STILL ADD?** | The floor tilts into torque-vs-rpm axes. | Hum. |
| 2 | 0:06–0:15 | Board | A teal point at 0 rpm, 9.362 N·m, tagged 476.1 A and "CTRE dyno". | **STALLED: 9.362 N·m · 476.1 A** | A second point. | Tick. |
| 3 | 0:15–0:22 | Board | A teal point at 5785 rpm, 0 N·m; a straight line draws between the two; a marker at half speed reads half torque (4.68 N·m). | **FREE: 5785 rpm · TORQUE FALLS IN A STRAIGHT LINE** | A circuit inset slides in. | Tick; line-draw sound. |
| 4 | 0:22–0:29 | Board | Circuit: 12 V battery, a back-EMF source facing it, resistor R; I = (12 − back-EMF) ÷ R writes on. | **I = (12 V − back-EMF) ÷ R** | Zoom to R. | Chime. |
| 5 | 0:29–0:38 | Board | Stalled: back-EMF is 0, so R = 12 ÷ 476.1 = 0.0252 writes on. | **R = 12 ÷ 476.1 = 0.0252 Ω** | Zoom to back-EMF. | Tick. |
| 6 | 0:38–0:45 | Board | back-EMF = motor speed ÷ K_v; K_v = 605.8 ÷ 12 = 50.48; at free speed the back-EMF bar reaches 12 V. | **K_v = 605.8 ÷ 12 = 50.48 rad/s PER V** | Torque formula. | Tick. |
| 7 | 0:45–0:51 | Board | τ = K_t · I; the current line and torque line overlay exactly. **EXTREME CLOSE-UP:** the line touching zero at 5785 rpm. | **τ = K_t · I** | A cap drops. | Click. |
| 8 | 0:51–0:58 | Board | A teal ceiling at 150 A clips the line flat at 2.95 N·m on the left. | **150 A CAP → 2.95 N·m** | The x-axis morphs from rpm to robot speed. | Clamp. |
| 9 | 0:58–1:04 | Board | The chain writes on: wheel force = τ × 7.3636 ÷ 0.05504, then a = 4 × force ÷ 60; the y-axis morphs to m/s². | **a = 4 · τ · 7.3636 ÷ 0.05504 ÷ 60** | A dot appears at 0. | Tick. |
| 10 | 1:04–1:11 | Board | The flat part at 26.3 runs out to 3.1 m/s; a dot rides it. | **26.3 m/s² UP TO 3.1 m/s** | The dot reaches the bend. | Hum rises with the dot. |
| 11 | 1:11–1:21 | Board | The dot passes labelled points: 19.0 at 3.5, 9.7 at 4.0, 4.2 at 4.3, 0 at 4.53. | **19.0 · 9.7 · 4.2 · 0 m/s²** | A red dashed line draws at 10. | One ping per point. |
| 12 | 1:21–1:28 | Board | Red dashed grip line at 10 crosses the curve near 4 m/s; left region "grip limits", right region "motors limit". | **BELOW ~4 m/s: GRIP · ABOVE: MOTORS** | An arrow flips backwards. | Switch click at the crossing. |
| 13 | 1:28–1:33 | Board | A braking arrow points backwards; the motor curve greys out; only the grip line stays. | **BRAKING: GRIP ONLY** | A clipboard slides in (chapter 14). | Chord resolves. |

**Narration (ch13-voice.json):**

1. Last: how much can the motors still add at a given speed?
2. C T R E's dyno says a stalled Kraken X sixty makes nine point three six two newton meters, at four hundred seventy-six point one amps.
3. Spinning free, it reaches five thousand seven hundred eighty-five r p m. In between, torque falls in a straight line.
4. Why a line? Current is twelve volts minus the back E M F, divided by the motor's resistance, R.
5. Stalled, there's no back E M F, so R is twelve over four seventy-six point one: zero point zero two five two ohms.
6. Back E M F is motor speed over K v, fifty point four eight. At free speed it reaches twelve volts.
7. Torque, written tau, is K t times current, so it follows the same line, down to zero.
8. Our planning current caps it at one hundred fifty amps: a flat two point nine five newton meters.
9. Times gear ratio, over wheel radius, is wheel force. Times four, over sixty kilograms, is acceleration.
10. The flat part is twenty-six point three meters per second squared, up to about three point one meters per second.
11. Then it falls. Nineteen point oh at three point five. Nine point seven at four. Four point two at four point three. Zero at four point five three.
12. Draw the grip line at ten. Below about four meters per second, grip limits. Above, the motors do.
13. Braking skips this curve: back E M F helps there, so only grip limits braking.

## Chapter 14: One loop, by hand (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:09 | Board | A clipboard titled TRACED EXAMPLE over a **WIDE** world inset: the robot mid-run, a goal ahead and to its left. The robot gets a blue 3.6 m/s arrow and a purple 1.0 rad/s curl; dashed "wanted" versions appear: 4.0 m/s and 3.0 rad/s. | **CURRENT: vx 3.6 · vy 0 · ω 1.0 · WANTED: vx 4.0 · vy 0 · ω 3.0** | A yellow clock appears. | Pencil scratch; two ticks. |
| 2 | 0:09–0:19 | Board | A yellow dt clock reads 0.02 s; (4.0 − 3.6) ÷ 0.02 = 20 writes on in orange, then (3.0 − 1.0) ÷ 0.02 = 100 in purple. | **dt = 0.02 s · ax = 0.4 ÷ 0.02 = 20 · α = 2.0 ÷ 0.02 = 100** | The purple clamp drops. | Clock tick; two ticks. |
| 3 | 0:19–0:22 | Board | The clamp squeezes 100 down to 8. | **CHECK 1: α = 8** | Push in on the outer wheel. | Clamp clunk. |
| 4 | 0:22–0:31 | World | Velocity and acceleration arrows both point forward ("speeding up"). **EXTREME CLOSE-UP:** the outer wheel's speed readout: 3.6 + 1.0 × 0.359 = 3.96. A dot on chapter 13's curve at 3.96 reads 10.49; the orange 20 shrinks to 10.49. | **CHECK 2: WHEEL SPEED 3.96 m/s → MOTORS ALLOW 10.49, NOT 20** | A triangle draws. | Gauge ticks. |
| 5 | 0:31–0:41 | Board | Triangle legs 10.49 (orange) and 8 × 0.359 = 2.87 (purple); long side 10.88 pokes past the circle; scale = 10 ÷ 10.88 = 0.919; the legs shrink to 9.64 and 7.35 × 0.359. | **CHECK 3: √(10.49² + 2.87²) = 10.88 → SCALE 0.919 → ax 9.64 · α 7.35** | Add up. | Warning blip; ring chime. |
| 6 | 0:41–0:46 | Board | 3.6 + 9.64 × 0.02 = 3.79 and 1.0 + 7.35 × 0.02 = 1.15 write on. | **NEW COMMAND: vx 3.79 · ω 1.15** | The board folds into the robot's brain and an editor unfolds (Part 4). | Success sting. |

**Narration (ch14-voice.json):**

1. One loop by hand. The robot drives at three point six meters per second, spinning at one. It wants four, spinning at three.
2. With d t, the loop time, at zero point zero two seconds, that asks for twenty meters per second squared of driving, and one hundred of spin.
3. Check one caps the spin change at eight.
4. Check two: we're speeding up, and the fastest wheel moves at three point nine six. There, the motors allow ten point four nine, not twenty.
5. Check three: ten point four nine and two point eight seven make ten point eight eight. That's over ten, so scale by zero point nine one nine.
6. The new command: three point seven nine meters per second, spinning at one point one five.

# Part 4: The code, line by line

## Chapter 15: The file and its constants, lines 1 to 41 (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:06 | Code | The camera pushes into the robot's brain; the editor panel unfolds; tab "utils/AccelerationLimiter.java". **WIDE:** a minimap of all 87 lines with the three checks tinted purple, teal and red. | **AccelerationLimiter.java · 87 LINES** | Highlight bar to lines 1–10. | Panel-unfold whoosh. |
| 2 | 0:06–0:13 | Code | Lines 1–10 highlight, then fold into a grey summary chip; the `TunerConstants` and `DCMotor` imports stay lit (`DCMotor` teal). | **L1–10: LICENSE · PACKAGE · 4 IMPORTS** | Scroll to 12. | Soft ticks. |
| 3 | 0:13–0:21 | Code | Lines 12–24 highlight; the three list items light purple (Turning), teal (Motors), red (Traction); then lines 22–23 glow. Inset: chapter 7's three-station machine, then the 360 A dial with a "CEILING" net. | **L12–24: THE THREE CHECKS · kSlipCurrent = CEILING ONLY** | To line 25. | Three notes; chord. |
| 4 | 0:21–0:25 | Code | Line 25 highlights; `final` glows; inset: a padlock snaps onto a class box. | **L25: public final class** | To 26–27. | Lock click. |
| 5 | 0:25–0:32 | Code | Lines 26–27 highlight with an amber flag; inset: a robot overshoots its mark when the limit sits below DECEL. | **L26–27: KEEP LIMITS ABOVE DriveToPose's DECEL (6) AND ANGULAR_DECEL (6)** | To line 28. | Warning blip. |
| 6 | 0:32–0:37 | Code | Line 28: `TRACTION_ACCEL` orange; ghost `10.0 m/s² ≈ 1 g`; inset: the red grip circle of radius 10 (chapter 8). | **L28: TRACTION_ACCEL = 10.0 m/s²** | To line 29. | Ring chime. |
| 7 | 0:37–0:40 | Code | Line 29: `ANGULAR_ACCEL` purple; ghost `8 rad/s²`; inset: the purple clamp. | **L29: ANGULAR_ACCEL = 8.0 rad/s²** | To line 30. | Clamp clunk. |
| 8 | 0:40–0:45 | Code | Line 30: `MODEL_CURRENT` teal; inset: the dial with 150 A between the 56 A mark and the 360 A needle. | **L30: MODEL_CURRENT = 150 A** | To line 31. | Tick. |
| 9 | 0:45–0:50 | Code | Line 31: `ROBOT_MASS` red; inset: the scale reading 60 kg with its TODO tag. | **L31: ROBOT_MASS = 60 kg (TODO)** | To 32–35. | Scale beep. |
| 10 | 0:50–0:56 | Code | Lines 32–35: `DRIVE_MOTOR` teal; each argument gets a chip: 12.0 V · 9.362 N·m · 476.1 A · 0.0 A · 5785 rpm = 605.8 rad/s · 1 motor. | **L34–35: new DCMotor(12 V, 9.362 N·m, 476.1 A, 0 A, 5785 rpm, 1)** | Zoom to the 0.0. | One tick per chip. |
| 11 | 0:56–1:03 | Code | **EXTREME CLOSE-UP:** the `0.0` argument; inset: a free-spinning motor whose ammeter reads about 10 A, labelled "friction". | **FREE CURRENT = 0, ON PURPOSE** | A small graph slides in. | Hum. |
| 12 | 1:03–1:12 | Board | Inset graph: two motor curves; the one with 10.4 A free current reads 11.4 at 4 m/s and runs past 4.53; the real one reads 9.7. | **WITH 10.4 A: 11.4 · WITH 0: 9.7 m/s² (at 4 m/s)** | Back to the editor, lines 36–37. | Buzz, then click. |
| 13 | 1:12–1:18 | Code | Lines 36–37: `GEAR_RATIO` teal and `WHEEL_RADIUS` green; ghosts `7.3636` and `0.05504 m`; thin links run to `TunerConstants.java` lines 99 and 101. | **L36–37: FROM TunerConstants · 7.3636 · 0.05504 m** | To 38–39. | Link click. |
| 14 | 1:18–1:24 | Code | Lines 38–39: `BASE_RADIUS` green; ghost `hypot(0.254, 0.254) = 0.359 m`; inset: the top view with r drawn (chapter 12). | **L38–39: BASE_RADIUS = 0.359 m** | To line 41. | Tick. |
| 15 | 1:24–1:31 | Code | Line 41: the private constructor; inset: a toolbox labelled "static methods" beside Java's `Math`. | **L41: NO OBJECTS · A TOOLBOX** | Scroll to line 43 (chapter 16). | Toolbox clunk. |

**Narration (ch15-voice.json):**

1. Here's the real file: Acceleration Limiter dot java, in the utils folder. Eighty-seven lines.
2. Lines one to ten: license, package and imports, including Tuner Constants and the D C Motor model.
3. Lines twelve to twenty-four are the Javadoc. It lists the three checks, and says k slip current is only the hardware ceiling.
4. Line twenty-five: public final class. Final means no class can extend it.
5. Lines twenty-six and twenty-seven warn: keep these limits above Drive To Pose's braking numbers, or the robot overshoots.
6. Line twenty-eight: traction accel, ten. The grip budget, shared by driving and turning.
7. Line twenty-nine: angular accel, eight. Check one's turning cap.
8. Line thirty: model current, one hundred fifty amps, only for the motor check.
9. Line thirty-one: robot mass, sixty kilograms, with a to-do to weigh it.
10. Lines thirty-four and thirty-five build the motor model from C T R E's dyno numbers.
11. The free current is zero on purpose. The real motor draws about ten amps spinning free, just fighting its own friction.
12. W P I lib would count those amps as push, and promise eleven point four, not nine point seven, at four meters per second.
13. Lines thirty-six and thirty-seven read gear ratio and wheel radius from Tuner Constants, so they always match.
14. Lines thirty-eight and thirty-nine: base radius, center to wheel, by Pythagoras. Zero point three five nine meters.
15. Line forty-one: a private constructor, so nobody can make an Acceleration Limiter object. It's a toolbox, like Java's Math.

**Code on screen:**

```java
 1  // Copyright (c) FIRST and other WPILib contributors.
 2  // Open Source Software; you can modify and/or share it under the terms of
 3  // the WPILib BSD license file in the root directory of this project.
 4
 5  package frc.robot.utils;
 6
 7  import frc.robot.generated.TunerConstants;
 8  import org.wpilib.math.kinematics.ChassisVelocities;
 9  import org.wpilib.math.system.DCMotor;
10  import org.wpilib.math.util.Units;
11
12  /**
13   * How fast the robot's velocity may change, from physics. Give it the velocity you have and the one
14   * you want; it returns the closest one the robot can reach this loop without slipping:
15   *
16   * <ol>
17   *   <li><b>Turning:</b> the change in spin is capped (sudden heading changes slip the wheels).
18   *   <li><b>Motors:</b> when speeding up, no more than the drive motors can still add at this speed.
19   *   <li><b>Traction:</b> driving and turning together, no more than the tires can grip.
20   * </ol>
21   *
22   * <p>kSlipCurrent in TunerConstants is only the hardware ceiling; these numbers decide how hard we
23   * actually push. Tuned in tools/alignsim (docs/alignment/studies/controllers.md).
24   */
25  public final class AccelerationLimiter {
26    // Keep TRACTION_ACCEL above DriveToPose's DECEL, and ANGULAR_ACCEL above its ANGULAR_DECEL, or
27    // the robot can't brake as planned and overshoots.
28    private static final double TRACTION_ACCEL = 10.0; // m/s^2 (~1 g), shared by driving and turning
29    private static final double ANGULAR_ACCEL = 8.0; // rad/s^2
30    private static final double MODEL_CURRENT = 150.0; // A per drive motor, for the torque limit
31    private static final double ROBOT_MASS = 60.0; // kg with battery and bumpers. TODO: weigh it
32    // Kraken X60 FOC, CTRE dyno data: 9.362 Nm and 476.1 A at stall, 5785 rpm free. Free current
33    // stays 0 so torque hits zero at the real free speed (DCMotor counts friction as push).
34    private static final DCMotor DRIVE_MOTOR =
35        new DCMotor(12.0, 9.362, 476.1, 0.0, Units.rotationsPerMinuteToRadiansPerSecond(5785), 1);
36    private static final double GEAR_RATIO = TunerConstants.FrontLeft.DriveMotorGearRatio;
37    private static final double WHEEL_RADIUS = TunerConstants.FrontLeft.WheelRadius; // m
38    private static final double BASE_RADIUS = // m, robot center to a wheel
39        Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);
40
41    private AccelerationLimiter() {}
```

## Chapter 16: limit(), the turning cap and the motor check, lines 43 to 65 (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:09 | Code | Lines 43–48 highlight; in the signature, `current` solid blue, `wanted` dashed blue, `dt` yellow. **WIDE:** the minimap marks lines 43–78 as `limit()`. Inset: a ChassisVelocities box with three slots, vx, vy, ω, over the field's axes. | **L47–48: limit(current, wanted, dt) · (vx, vy, ω), FIELD-RELATIVE** | The slots fill. | Tick; slot clicks. |
| 2 | 0:09–0:17 | Code | Ghost text after line 48: `current = (3.6, 0, 1.0)  wanted = (4.0, 0, 3.0)  dt = 0.02`; inset: the traced robot from chapter 14. | **TRACED: (3.6, 0, 1.0) → (4.0, 0, 3.0), dt 0.02** | To line 49. | Pencil scratch. |
| 3 | 0:17–0:22 | Code | Line 49; ghost `(4.0 − 3.6) / 0.02 = 20.0`; inset: an orange arrow 20 long. | **L49: ax = 20.0** | To line 50. | Tick. |
| 4 | 0:22–0:26 | Code | Line 50; ghost `0.0`; inset: no sideways arrow. | **L50: ay = 0.0** | To 51–52. | Tick. |
| 5 | 0:26–0:35 | Code | Lines 51–52; ghost `(3.0 − 1.0) / 0.02 = 100 → min(8, 100) = 8 → max(−8, 8) = 8.0`. **EXTREME CLOSE-UP:** the `ANGULAR_ACCEL` inside `Math.min`. Inset: the purple clamp. | **L51–52: alpha = 8.0 · CHECK 1 (Part 3: turning cap)** | To 54–55. | Clamp clunk. |
| 6 | 0:35–0:40 | Code | Lines 54–55 highlight in grey; two tags pop: "speeding up only" and "spin-up not counted (Q7)". | **L54–55: ONLY SPEEDING UP · SPIN-UP NOT COUNTED** | To line 56. | Soft tick. |
| 7 | 0:40–0:47 | Code | Line 56; inset: the velocity arrow and the acceleration arrow; their x parts multiply, their y parts multiply, the two products add. | **L56: ax·vx + ay·vy = DOT PRODUCT** | Ghost value. | Tick. |
| 8 | 0:47–0:53 | Code | Ghost `20.0 × 3.6 + 0 × 0 = 72 > 0 → true`; inset: both arrows forward (speeding up), and a mini braking case with opposite arrows giving a negative number. | **72 > 0 → SPEEDING UP** | To 57–58. | Gauge tick. |
| 9 | 0:53–1:01 | Code | Lines 57–58; ghost `3.6 + 1.0 × 0.359 = 3.96`; inset: top view, the outer wheel's arrow longest. | **L57–58: wheelSpeed = 3.96 m/s** | To line 59. | Tick. |
| 10 | 1:01–1:07 | Code | Line 59; ghost `motorAccel(3.96) = 10.49`; inset: a dot on chapter 13's curve. | **L59: motorLimit = 10.49 (Part 3: motor curve)** | To line 60. | Ping. |
| 11 | 1:07–1:13 | Code | Line 60; ghost `hypot(20.0, 0.0) = 20.0`; inset: a tiny right triangle labelled "hypot". | **L60: a = 20.0 · hypot = PYTHAGORAS** | To 61–64. | Tick. |
| 12 | 1:13–1:23 | Code | Lines 61–64; ghosts `20.0 > 10.49 → true`, `ax = 20.0 × 10.49 / 20.0 = 10.49`, `ay = 0.0`; inset: the orange arrow shrinks, direction unchanged. | **L61–64: ax = 10.49 · CHECK 2** | Highlight `alpha`'s ghost. | Gauge tick. |
| 13 | 1:23–1:29 | Code | `alpha`'s ghost stays `8.0`, untouched; inset: two bars, the 2026 version 0.08–0.12 s slower. | **alpha NOT CUT HERE · 2026 VERSION: 0.08–0.12 s SLOWER (our simulator)** | Scroll to line 67 (chapter 17). | Two notes. |

**Narration (ch16-voice.json):**

1. Lines forty-seven and forty-eight: limit takes current, wanted and d t. Each velocity holds v x, v y and omega, along the field.
2. In goes our example. Current: three point six, zero, one. Wanted: four, zero, three. d t: zero point zero two.
3. Line forty-nine: a x, the change in v x over d t. Twenty.
4. Line fifty: a y. Zero, since we drive straight along x.
5. Lines fifty-one and fifty-two: alpha, the change in omega over d t, clamped between minus eight and eight. One hundred becomes eight. Check one.
6. Lines fifty-four and fifty-five are comments: only speeding up is limited, and spin-up isn't counted.
7. Line fifty-six asks: are we speeding up? It's a dot product: multiply x with x, y with y, and add.
8. Positive means speeding up. Twenty times three point six is seventy-two, so we check the motors.
9. Lines fifty-seven and fifty-eight: wheel speed, the fastest any wheel could move. Speed, plus spin times base radius: three point nine six.
10. Line fifty-nine: motor accel at that speed. Motor limit is ten point four nine.
11. Line sixty: a, the size of the driving acceleration. Hypot, short for hypotenuse, is Pythagoras in one call. Twenty.
12. Lines sixty-one to sixty-four: if a is too big, scale a x and a y by motor limit over a. Twenty becomes ten point four nine.
13. Alpha isn't cut here: in our simulator, cutting spin-up too, like the twenty twenty-six code, slowed turns while driving.

**Code on screen:**

```java
43    /**
44     * The velocity to command this loop: as close to {@code wanted} as physics allows, starting from
45     * {@code current}. Both field-relative; {@code dt} in seconds.
46     */
47    public static ChassisVelocities limit(
48        ChassisVelocities current, ChassisVelocities wanted, double dt) {
49      double ax = (wanted.vx - current.vx) / dt;
50      double ay = (wanted.vy - current.vy) / dt;
51      double alpha =
52          Math.max(-ANGULAR_ACCEL, Math.min(ANGULAR_ACCEL, (wanted.omega - current.omega) / dt));
53
54      // Motors: only speeding up is limited; less torque is left as the wheels near free speed.
55      // Spin-up isn't counted: using the worst-case wheel speed, it slowed turns while driving (Q7).
56      if (ax * current.vx + ay * current.vy > 0.0) {
57        double wheelSpeed =
58            Math.hypot(current.vx, current.vy) + Math.abs(current.omega) * BASE_RADIUS;
59        double motorLimit = motorAccel(wheelSpeed);
60        double a = Math.hypot(ax, ay);
61        if (a > motorLimit) {
62          ax *= motorLimit / a;
63          ay *= motorLimit / a;
64        }
65      }
```

## Chapter 17: limit(), traction and the answer, lines 67 to 78 (about 1 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:07 | Code | Lines 67–68 highlight; inset: the red circle with the orange and purple arrows. | **L67–68: ONE GRIP, SCALED AS ONE** | To line 69. | Ring chime. |
| 2 | 0:07–0:15 | Code | Line 69: the inner `hypot` glows orange, `alpha * BASE_RADIUS` purple, the outer `hypot` red. **EXTREME CLOSE-UP:** `alpha * BASE_RADIUS`. Inset: the triangle assembling. | **L69: total = LONG SIDE (Part 3: friction circle)** | Ghost value. | Click. |
| 3 | 0:15–0:21 | Code | Ghost `hypot(10.49, 8.0 × 0.359 = 2.87) = 10.88`; inset numbers on the triangle. | **total = 10.88** | To line 70. | Tick. |
| 4 | 0:21–0:24 | Code | Line 70; ghost `10.88 > 10.0 → true`; inset: the long side pokes past the circle. | **L70: 10.88 > TRACTION_ACCEL** | To line 71. | Warning blip. |
| 5 | 0:24–0:29 | Code | Line 71; ghost `10.0 / 10.88 = 0.919`. | **L71: scale = 0.919** | To 72–74. | Tick. |
| 6 | 0:29–0:38 | Code | Lines 72–74; ghosts `ax = 9.64`, `ay = 0.0`, `alpha = 7.35`; inset: the triangle shrinks, same shape. **WIDE:** the inset pulls back to the field; the robot's trail stays straight. | **L72–74: SAME SCALE FOR ALL THREE · CHECK 3** | To 76–77. | Ring chime. |
| 7 | 0:38–0:43 | Code | Lines 76–77; inset: the blue current arrow plus a short orange step equals the new blue arrow. | **L76–77: NEW = CURRENT + ACCEL × dt** | Ghost for vx. | Click. |
| 8 | 0:43–0:50 | Code | Ghost `vx: 3.6 + 9.64 × 0.02 = 3.79`. | **vx = 3.79 m/s** | Ghost for ω. | Tick. |
| 9 | 0:50–0:58 | Code | Ghosts `omega: 1.0 + 7.35 × 0.02 = 1.15` and `vy: 0.0 + 0.0 = 0.0`. | **ω = 1.15 rad/s · vy = 0** | Line 78 closes the method. | Tick. |
| 10 | 0:58–1:04 | Code | The result token (3.79, 0, 1.15) drops out of the panel to the robot; the next tick it loops back in as `current`. | **THE FASTEST CHANGE THE ROBOT CAN REALLY MAKE** | Scroll to line 80 (chapter 18). | Success sting. |

**Narration (ch17-voice.json):**

1. Lines sixty-seven and sixty-eight: driving and turning share the grip, scaled as one, so paths stay straight.
2. Line sixty-nine: total. The inner hypot is driving. Alpha times base radius is turning. The outer hypot is the long side.
3. Here, ten point four nine and two point eight seven make ten point eight eight.
4. Line seventy: is that more than traction accel? Yes.
5. Line seventy-one: scale is traction accel over total. Zero point nine one nine.
6. Lines seventy-two to seventy-four multiply a x, a y and alpha by that one scale: nine point six four, zero, seven point three five. Check three.
7. Lines seventy-six and seventy-seven build the answer: current, plus acceleration times d t.
8. V x: three point six plus nine point six four times zero point zero two. Three point seven nine.
9. Omega: one plus seven point three five times zero point zero two. One point one five. V y stays zero.
10. That's the fastest change the robot can really make this loop. The next loop starts from here.

**Code on screen:**

```java
67      // Traction: driving and turning share the tread's grip. Scaled as one vector, so paths stay
68      // straight.
69      double total = Math.hypot(Math.hypot(ax, ay), alpha * BASE_RADIUS);
70      if (total > TRACTION_ACCEL) {
71        double scale = TRACTION_ACCEL / total;
72        ax *= scale;
73        ay *= scale;
74        alpha *= scale;
75      }
76      return new ChassisVelocities(
77          current.vx + ax * dt, current.vy + ay * dt, current.omega + alpha * dt);
78    }
```

## Chapter 18: motorAccel(), and who calls limit() (about 1.5 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:08 | Code | Lines 80–81 highlight; ghost `wheelSpeed = 3.96`. **WIDE:** the minimap shows we're at the last method. | **L81: motorAccel(wheelSpeed) → m/s²** | To line 82. | Tick. |
| 2 | 0:08–0:15 | Code | Line 82; ghost `3.96 / 0.05504 × 7.3636 = 529.7 rad/s (5058 rpm)`; inset: the wheel turning, the gears, the motor turning faster. | **L82: motorSpeed ≈ 530 rad/s** | To line 83. | Gear whirr. |
| 3 | 0:15–0:25 | Code | Line 83, `getCurrent` glows; ghost `(12 − 529.7 / 50.48) / 0.0252 = 59.8`; inset: the circuit with back-EMF 10.5 V. Then `Math.min(MODEL_CURRENT, …)` glows; ghost `min(150, 59.8) = 59.8`; inset: the curve's flat part labelled "150 A wins here, at low speed". | **L83: getCurrent → 59.8 A (Part 3: I = (12 − back-EMF) ÷ R) · min(150, 59.8)** | To line 84. | Hum; tick. |
| 4 | 0:25–0:33 | Code | Line 84; ghost `59.8 × 0.01966 = 1.176 N·m → × 7.3636 / 0.05504 = 157.4 N`; inset: a red arrow at one tire patch. | **L84: wheelForce = 157.4 N** | Zoom to `Math.max`. | Tick. |
| 5 | 0:33–0:39 | Code | **EXTREME CLOSE-UP:** `Math.max(0.0, amps)`; inset: past 4.53 m/s the current line dips below zero and is clipped to 0. | **max(0, amps): NEVER A NEGATIVE LIMIT** | To line 85. | Click. |
| 6 | 0:39–0:46 | Code | Line 85; ghost `4 × 157.4 / 60 = 10.49`; "F = m · a" lifts off a board card and lands on the line as "a = F ÷ m". | **L85: 4 × 157.4 ÷ 60 = 10.49 m/s²** | A second tab opens. | Key-reveal sting. |
| 7 | 0:46–0:51 | Code | Tab `commands/DriveToPose.java` opens and scrolls to line 197. | **WHO CALLS limit()? DriveToPose.java:197** | `vx, vy, omega` glow. | Tab click. |
| 8 | 0:51–1:00 | Code | Line 197: `new ChassisVelocities(vx, vy, omega)` glows dashed blue ("wanted, from the speed rule, video 2"); then `commanded` glows solid blue; inset: a loop arrow, commanded → limit → commanded. | **wanted = (vx, vy, omega) · current = LAST LOOP'S COMMAND** | Jump to line 152. | Tick; loop whoosh. |
| 9 | 1:00–1:07 | Code | Line 152 highlights: `commanded = drivetrain.getFieldVelocity();`; inset: the robot already rolling when the command starts. | **L152: STARTS FROM THE MEASURED VELOCITY** | Jump to line 161. | Tick. |
| 10 | 1:07–1:13 | Code | Line 161 highlights; `dt` yellow; ghost `≈ 0.02`; tags on the 0.001 and 0.1 guards. | **L161: dt MEASURED EVERY LOOP** | Jump to line 199. | Clock tick. |
| 11 | 1:13–1:20 | Code | Line 199 highlights; the command flows out of the panel to the four modules. **WIDE:** the robot drives off across the field. | **L199: TO THE DRIVETRAIN, 50 TIMES A SECOND** | Into Part 5. | Success sting. |

**Narration (ch18-voice.json):**

1. Line eighty-one: motor accel takes a wheel speed and returns the acceleration the four motors can still add.
2. Line eighty-two: over the wheel radius, times the gear ratio, gives the motor's speed. About five hundred thirty radians per second.
3. Line eighty-three: get current uses twelve volts and the back E M F: fifty-nine point eight amps. Math dot min caps it at one hundred fifty.
4. Line eighty-four: get torque turns amps into newton meters. Times gear ratio, over wheel radius: one hundred fifty-seven point four newtons at the tire.
5. Math dot max with zero matters past top speed, where the current would go negative.
6. Line eighty-five: four wheels times that force, over the mass. F equals m a, turned around. Ten point four nine.
7. Who calls limit? Exactly one line: Drive To Pose, line one ninety-seven.
8. Drive To Pose passes in the velocity it wants, and, as current, last loop's command. So each command is a small step from the last.
9. Line one fifty-two seeds it with the measured velocity when Drive To Pose starts, so the first step starts from reality.
10. Line one sixty-one measures d t each loop, about zero point zero two seconds.
11. Line one ninety-nine sends the result to the drivetrain. Fifty times a second, one small, safe step at a time.

**Code on screen:**

`AccelerationLimiter.java`, lines 80–87:

```java
80    /** Most acceleration the four drive motors can still add at this wheel speed (m/s^2). */
81    public static double motorAccel(double wheelSpeed) {
82      double motorSpeed = wheelSpeed / WHEEL_RADIUS * GEAR_RATIO; // rad/s
83      double amps = Math.min(MODEL_CURRENT, DRIVE_MOTOR.getCurrent(motorSpeed, 12.0));
84      double wheelForce = DRIVE_MOTOR.getTorque(Math.max(0.0, amps)) * GEAR_RATIO / WHEEL_RADIUS;
85      return 4.0 * wheelForce / ROBOT_MASS;
86    }
87  }
```

`DriveToPose.java`, lines 152, 161 and 196–199:

```java
152      commanded = drivetrain.getFieldVelocity();
161      double dt = Math.min(Math.max(now - lastTime, 0.001), 0.1); // guards a skipped loop
196      // Physics decides how much of that change we can have this loop.
197      commanded = AccelerationLimiter.limit(commanded, new ChassisVelocities(vx, vy, omega), dt);
198
199      drivetrain.setControl(driveRequest.withVelocity(commanded));
```

# Part 5: Recap

## Chapter 19: Recap and check yourself (about 2 min)

| # | Time | Mode | Picture | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0:00–0:04 | World | **WIDE:** the field; the robot does a clean drive-and-turn onto a scoring spot. A RECAP card slides in. | **THE WHOLE IDEA IN FOUR SENTENCES** | Card 1. | Calm motif returns. |
| 2 | 0:04–0:12 | World | **EXTREME CLOSE-UP:** a tire patch inside its red ring; then a ghost robot drifts away from the hero. | **1. PUSH ≤ GRIP (≈ 1 g) · SLIP MAKES ODOMETRY LIE** | Card 2. | Chime. |
| 3 | 0:12–0:18 | World | Load bars shift; a turning triangle; the motor curve falls. | **2. GRIP MOVES WITH WEIGHT, IS SHARED WITH TURNING · MOTORS FADE AT SPEED** | Card 3. | Chime. |
| 4 | 0:18–0:25 | World | The three-station machine runs a token through: clamp, gauge, ring. | **3. EVERY LOOP: TURNING CAP → MOTORS → TRACTION** | Card 4. | Conveyor. |
| 5 | 0:25–0:32 | World | The 360 A dial becomes a safety net under the robot. | **4. CURRENT LIMIT = CEILING, NOT PLAN** | A pause card. | Chord. |
| 6 | 0:32–0:34 | Board | A CHECK YOURSELF card with a pause icon. | **CHECK YOURSELF · PAUSE AFTER EACH QUESTION** | Question 1. | Soft gong. |
| 7 | 0:34–0:43 | Board | Q1 card: the robot on a scale, 60 → 70 kg. **PAUSE card** (5 s, clock sweeping). | **Q1: +10 kg. CHANGE TRACTION_ACCEL?** | Answer. | Held pad, 5 s pause (`after` 5.0). |
| 8 | 0:43–0:50 | Board | μ · (m g ÷ 4) ÷ (m ÷ 4) = μ g with the m's struck out; then line 85's `/ ROBOT_MASS` glows. | **A1: NO, THE MASS CANCELS · THE MOTOR CHECK GETS WEAKER** | Question 2. | Correct ding. |
| 9 | 0:50–1:00 | Board | Q2 card: the current-limit dial turned to 60 A. **PAUSE card.** | **Q2: WHY NOT kSlipCurrent = 60 A?** | Answer. | Held pad, 5 s pause (`after` 5.0). |
| 10 | 1:00–1:08 | World | Load bars: tall back bars capped by a 60 A bar; the robot lags; a ghost drifts. | **A2: THE HEAVY BACK WHEELS NEED ROOM TO PUSH** | Question 3. | Correct ding. |
| 11 | 1:08–1:17 | Board | Q3 card: speedometer at 4 m/s. **PAUSE card.** | **Q3: AT 4 m/s, GRIP OR MOTORS?** | Answer. | Held pad, 5 s pause (`after` 5.0). |
| 12 | 1:17–1:23 | Board | The motor curve's dot at 4.0 reads 9.7, just under the red grip line at 10. | **A3: MOTORS (9.7 < 10)** | Question 4. | Correct ding. |
| 13 | 1:23–1:33 | Board | Q4 card: the triangle with three arrows, ax, ay, alpha. **PAUSE card.** | **Q4: WHY SCALE ax, ay AND alpha TOGETHER?** | Answer. | Held pad, 5 s pause (`after` 5.0). |
| 14 | 1:33–1:38 | World | Two trails: one bent (shrink one), one straight (shrink all). | **A4: SO THE DIRECTION DOESN'T CHANGE** | Question 5. | Correct ding. |
| 15 | 1:38–1:47 | Board | Q5 card: a braking robot. **PAUSE card.** | **Q5: WHY NO MOTOR CHECK WHEN BRAKING?** | Answer. | Held pad, 5 s pause (`after` 5.0). |
| 16 | 1:47–1:52 | Board | The back-EMF arrow flips to help; line 56's `> 0.0` glows. | **A5: BACK-EMF HELPS BRAKING · ONLY GRIP LIMITS** | TRY IT card. | Correct ding. |
| 17 | 1:52–2:00 | Code | TRY IT card; the editor shows `tools/alignsim/studies/controllers/q7_limiter.py`; a new line types into the `CTRL` dict: `"physics, traction 12": phys(traction=12.0),`; a terminal runs `cd tools && python3 alignsim/studies/controllers/q7_limiter.py 60 4 bench`. | **TRY IT: ADD traction=12.0 TO q7_limiter.py, RUN THE BENCH** | The table prints. | Typing; terminal clicks. |
| 18 | 2:00–2:06 | Code | The results table's `slip ms` and `odo p95 cm` columns glow; a balance scale weighs "time saved" against "odometry error"; then the 12 becomes 8. | **COMPARE slip ms · odo p95 cm · THEN TRY 8** | Switch to the Java sim. | Curious motif. |
| 19 | 2:06–2:14 | Code | Line 28 edited from `10.0` to `7.0`; `./gradlew simulateJavaAgent '-Pmode=auto:2 - Drive To Pose'`; AdvantageScope plots `DriveToPose/CommandedVelocity` and the ramp gets gentler. **WIDE:** AdvantageScope's field view with the robot. End card: "Next: Fifty decisions a second (video 2) · learn more: frc5712.com/drive-to-point". | **JAVA SIM: TRACTION_ACCEL 7 · WATCH THE RAMP · KEEP IT ABOVE 6 (LINE 26)** | End. | Success sting; music out. |

**Narration (ch19-voice.json):**

1. Here's the whole idea, in four sentences.
2. One. A robot can only push as hard as its tires grip, about one g, and slipping wheels make odometry lie.
3. Two. Grip moves with the weight, turning shares it, and motors lose push as they speed up.
4. Three. So every loop, Acceleration Limiter caps the spin change, checks the motors, and fits it all inside the grip.
5. Four. The current limit stays high, as a ceiling, so the heavy wheels have room to push.
6. Now, check yourself. Pause after each question.
7. Question one. Our robot gains ten kilograms. Should traction accel change?
8. Not by the formula: the mass cancels. But the motor check gets weaker, because robot mass is in it.
9. Question two. Why not set k slip current to sixty amps, and stop slip that way?
10. A low limit stops the heavy back wheels from pushing harder when the front ones slip. Slower, with worse odometry.
11. Question three. At four meters per second, which limit matters more, grip or motors?
12. The motors. At four, they can add only nine point seven, just under the grip's ten.
13. Question four. Why does the traction check shrink a x, a y and alpha together?
14. So the direction doesn't change. Shrinking only one would bend the path.
15. Question five. Why doesn't the motor check run when the robot brakes?
16. When braking, back E M F helps instead of fighting. Only grip limits braking.
17. Now try it. In the Python simulator's Q seven limiter study, add a row with traction twelve, and run the bench.
18. Compare slip and odometry error with traction ten. Is a little time worth it? Then try eight.
19. Or in the Java simulator, set traction accel to seven, and watch the commanded velocity ramp more gently. Never go below six.

## Fact check

Every number recomputed in Python (`python3`) from the constants below. Results are physics or
"in our simulator" (tools/alignsim); none is robot-tested.

| Claim | Basis |
| --- | --- |
| 50 loops a second, dt ≈ 0.02 s | The robot loop; `DriveToPose.java:161` measures dt each loop and clamps it to 0.001–0.1 s. |
| 0 → 4 m/s in one loop needs 200 m/s² | 4 ÷ 0.02 = 200. 4 m/s is `DriveToPose`'s `MAX_SPEED` (`DriveToPose.java:37`). |
| One wheel turn ≈ 35 cm | 2π × 0.05504 m = 0.3458 m. |
| Up to 1.5 m of odometry over-count from one-loop command jumps | controllers.md §1, "Laws without a vector limit wreck odometry from a moving start": a 30–300 m/s² step in one loop; "Encoders then over-read by up to 1.5 m" (simulator). |
| Careful hard driving costs up to 35 cm of odometry error (2–35 cm) at the start of an approach | skid-odometry.md, Bottom line: "Accel limits of 10–12 m/s² and fast direction changes cost 2–35 cm, at the start of the approach" (simulator). |
| Close to the target the robot finishes on odometry | skid-odometry.md, Question: the robot "coasts on odometry once the tag leaves view"; frames keep arriving until about 3 cm out. |
| Target within 2 cm | alignsim README, Metrics: success = true error < 2 cm and < 1°. |
| `kSlipCurrent` = 360 A, a hardware ceiling | `TunerConstants.java:61` (comment lines 57–60: "Hardware ceiling only"). `AccelerationLimiter.java:22–23`. |
| The 2026 team lesson (low slip current starves the loaded wheels) | q6_physics.md intro ("Prompted by two robot-side points from the team"); controllers.md §6, Q5/Q6 bullets. |
| 60 A: +0.44 s, 1.7 × odometry error (CG 0.2 m, accel 10) | q6_physics.md, CG 0.2 m table, 50 Hz rows: 120 A = 833 ms slip / 1.26 s / 4.4 cm; 60 A = 1022 / 1.70 / 7.4. 1.70 − 1.26 = 0.44 s; 7.4 ÷ 4.4 = 1.68. 120 A and 360 A give the same results (q6_physics.md, "120 A vs 360 A … changes nothing"). |
| F = m·a: 600 N, 150 N per wheel | 60 × 10 = 600; 600 ÷ 4 = 150. |
| m = 60 kg (TODO) | `AccelerationLimiter.java:31`. |
| Load per wheel 147 N | 60 × 9.81 ÷ 4 = 147.15 N. |
| μ ≈ 1; `TRACTION_ACCEL` = 10 ≈ 1 g | `AccelerationLimiter.java:28` ("~1 g"). μ·g = 1 × 9.81 = 9.81; 147.15 ÷ 15 = 9.81. 10 is a round "about 1 g": at μ = 1 the 150 N push for 10 m/s² is 2% over the 147 N grip. The Python simulator assumes μ_static 1.15, μ_kinetic 1.0 (`tools/alignsim/params.py:17–18`). |
| Mass cancels: a_max = μ·g | μ·(m·g/4) ÷ (m/4) = μ·g. |
| Wheel radius 0.05504 m | `TunerConstants.java:101`: 2.167 in × 0.0254 = 0.0550418 m. |
| 147 N × 0.05504 m = 8.10 N·m | 147.15 × 0.0550418 = 8.0994. |
| Gear ratio 7.3636 | `TunerConstants.java:99`: 7.363636… |
| 1.10 N·m at the motor | 8.0994 ÷ 7.363636 = 1.0999. |
| K_t = 0.01966 N·m/A | 9.362 ÷ 476.1 = 0.019664 (CTRE dyno numbers, `AccelerationLimiter.java:32–35`). |
| About 56 A per wheel to reach grip | 1.0999 ÷ 0.019664 = 55.94 A (the task's 147 × 0.05504 ÷ 7.3636 ÷ 0.01966 = 55.9). The earlier short brief said 57 A: that is the 150 N push for 10 m/s² (150 ÷ 2.6307 = 57.0 A), or WPILib's built-in Kraken model (9.37 N·m / 483 A gives 56.7 A). |
| 2.63 N per amp at the tire; 360 A → 947 N = 6.4 × grip | 0.019664 × 7.363636 ÷ 0.0550418 = 2.6307 N/A; 360 × 2.6307 = 947.0 N; 947.0 ÷ 147.15 = 6.44. |
| `MODEL_CURRENT` = 150 A | `AccelerationLimiter.java:30`; q7_limiter.md intro. |
| x = 0.254 m | `TunerConstants.java:155–156` (and 166–189): module positions ±10 in = ±0.254 m. |
| h = 0.20 m is an example | Same value as the q6/q7 "CG 0.2 m" rows. The real CG height is unmeasured. |
| ΔN = m·a·h ÷ (4x) | Moments about the centre of mass: the floor push m·a acts h below it (moment m·a·h); two back wheels gain ΔN and two front wheels lose ΔN, each x away (moment 4·ΔN·x). Assumes the CG is centred between the axles, level floor, no suspension; the simulator also lags the load by 30 ms (q6_physics.md). |
| a = 6: ΔN 71 N, front 76 N, back 218 N | 60 × 6 × 0.2 ÷ (4 × 0.254) = 72 ÷ 1.016 = 70.87; 147.15 − 70.87 = 76.28; 147.15 + 70.87 = 218.02. (q6_physics.md quotes "~90 N / ~210 N" from the simulator; this brief uses the formula.) |
| a = 10: front 29 N, back 265 N | ΔN = 120 ÷ 1.016 = 118.11; 147.15 − 118.11 = 29.04; 147.15 + 118.11 = 265.26. |
| Tip-over at 12.5 m/s² | Front load zero when a = g·x ÷ h = 9.81 × 0.254 ÷ 0.2 = 12.46. |
| Velocity-controlled modules push about equally | q6_physics.md, "Reading it", first bullet; controllers.md §6, Q6. |
| Equal-push limit 5.5 m/s² | m·a/4 = μ(m·g/4 − m·a·h/(4x)) ⇒ a = μg ÷ (1 + μh/x) = 9.81 ÷ (1 + 0.2 ÷ 0.254) = 9.81 ÷ 1.7874 = 5.49. controllers.md §6 says "~6": that uses the simulator's μ = 1.15 (11.28 ÷ 1.905 = 5.92). |
| Back wheel at 10 m/s²: 265 N ≈ 101 A; 60 A → 158 N | 265.26 ÷ 2.6307 = 100.8 A; 60 × 2.6307 = 157.8 N. |
| When the front slips, the back modules push harder if the limit allows | q6_physics.md: with a low slip current "the loaded wheels can't deliver the force". The "falls behind its command, the speed loop pushes harder" wording is our plain-language explanation of a velocity loop. |
| Tag alignment still 96–99% at accel 10 with a 0.2 m CG | controllers.md §6, Q6 bullet "Alignment still works (96–99%)"; q6_physics.md tag tables (CG 0.2 m, 120 A, accel 10: 99% hard, 96% very hard). |
| Pushing by load: 3–4 × less slip at 6–8 m/s² | q6_physics.md, CG 0.2 m table: 250 Hz 120 A = 376 / 890 ms vs 250 Hz + load-split force FF = 98 / 295 ms (3.8 × and 3.0 ×). Needs the faster loop too. |
| 0.2 m/s change per loop at 10 m/s² | 10 × 0.02 = 0.2. |
| 250 steps a second removed most of the step slip | q6_physics.md: "250 Hz control removes the setpoint-step slip"; CG 0, accel 8: 183 → 12 ms. Not in the template yet (it runs at 50 Hz). |
| Three checks: 15–30% less odometry error than one fixed limit | q7_limiter.md, "Reading it", first bullet (7.0 → 5.3, 14.2 → 9.6, 17.7 → 13.8 cm); controllers.md §6, Q7. |
| Steady spin is cheap; changing spin costs grip | A constant spin needs no net turning push; the limiter budgets only α (`AccelerationLimiter.java:69`). controllers.md Caveats: "A steady drive-and-spin (3 m/s with 3 rad/s) never slips". |
| r = 0.359 m | `AccelerationLimiter.java:38–39`: hypot(0.254, 0.254) = 0.35921. |
| α·r = 2.87; driving keeps 9.58 (−4%) | 8 × 0.35921 = 2.8737; 2.8737² = 8.258; √(100 − 8.258) = 9.578; (10 − 9.578) ÷ 10 = 4.2%. |
| Scale example 0.961 | hypot(10, 2.8737) = 10.405; 10 ÷ 10.405 = 0.9611; 10 × 0.9611 = 9.61, 2.8737 × 0.9611 = 2.76. |
| About 80% of slip happened while turning | controllers.md §1, "Where its slip came from": "About 80 % of VectorBrake's slip happens while turning … from the heading rate steps" (simulator). |
| Turning cap 20 → 8: about 6 × less slip, no time lost | controllers.md §1: "ANGULAR_ACCEL 8 with ANGULAR_DECEL 6 cuts slip by 6× at no time cost"; old values 20 and 8 in "The answer" table. |
| Kraken X60 FOC dyno: 9.362 N·m, 476.1 A, 5785 rpm | `AccelerationLimiter.java:32–35`; q7_limiter.md last bullet (WPILib's built-in is 9.37 / 483 / 5800). |
| Half speed, half torque | 9.362 × (1 − 2892.5 ÷ 5785) = 4.681 N·m. |
| I = (12 − back-EMF) ÷ R; R = 0.0252 Ω; K_v = 50.48 | WPILib `DCMotor`: R = V ÷ stallCurrent, K_v = freeSpeed ÷ (V − R·freeCurrent), `getCurrent` = (V − ω/K_v) ÷ R, `getTorque` = K_t·I. 12 ÷ 476.1 = 0.025205 Ω; 5785 × 2π ÷ 60 = 605.80 rad/s; 605.80 ÷ 12 = 50.484 (free current 0). |
| 150 A cap → 2.95 N·m → 26.3 m/s² | 150 × 0.019664 = 2.9496 N·m; × 7.363636 ÷ 0.0550418 = 394.60 N; 4 × 394.60 ÷ 60 = 26.31. |
| Flat up to 3.1 m/s | (12 − 150 × 0.025205) × 50.484 = 414.94 rad/s at the motor; × 0.0550418 ÷ 7.363636 = 3.10 m/s, shown as 3.1. (The task's "~3.3 m/s" recomputes to 3.10.) |
| 19.0 / 9.7 / 4.2 / 0 m/s² at 3.5 / 4.0 / 4.3 / 4.53 m/s | `motorAccel`: 3.5 m/s → 468.2 rad/s, 108.1 A, 18.96; 4.0 → 535.1 rad/s, 55.5 A, 9.74; 4.3 → 575.3 rad/s, 24.0 A, 4.21; free wheel speed 605.80 × 0.0550418 ÷ 7.363636 = 4.528 m/s (`kSpeedAt12Volts` 4.54, `TunerConstants.java:93`). Matches q7_limiter.md "26 / 19 / 10 / 4 / 0". |
| Motors weaker than grip above about 4 m/s | `motorAccel` = 10 at 3.99 m/s (11.59 at 3.9, 9.74 at 4.0). |
| Free current 10.4 A would give 11.4 instead of 9.7 at 4 m/s | q7_limiter.md last bullet; recomputed: K_v = 605.80 ÷ (12 − 0.025205 × 10.4) = 51.60 → 11.35 m/s². `AccelerationLimiter.java:32–33`. |
| Braking is grip-only; the motor check runs only when speeding up | `AccelerationLimiter.java:56` (`ax * current.vx + ay * current.vy > 0.0`). While braking, the motor current (V − back-EMF) ÷ R gets larger in size as back-EMF grows, so the motor has plenty of braking torque. |
| Spin-up not counted in the motor check; the 2026 version was 0.08–0.12 s slower with the same error | `AccelerationLimiter.java:55`; q7_limiter.md, "Reading it", fifth bullet (from rest 1.65 vs 1.57 s, moving starts 1.73 vs 1.61 s). |
| Traced example, `limit()` | current (3.6, 0, 1.0), wanted (4.0, 0, 3.0), dt 0.02. ax = 0.4 ÷ 0.02 = 20.0; ay = 0; raw α = 2.0 ÷ 0.02 = 100 → 8.0; dot = 20 × 3.6 = 72 > 0; wheelSpeed = 3.6 + 1.0 × 0.35921 = 3.9592; motorLimit = 10.4931; a = 20.0 → ax = 10.4931; total = hypot(10.4931, 8 × 0.35921 = 2.8737) = 10.8795; scale = 0.91916; ax = 9.6448, α = 7.3533; result vx = 3.6 + 0.19290 = 3.7929, ω = 1.0 + 0.14707 = 1.1471. All three checks change the answer. |
| Traced example, `motorAccel(3.96)` | motorSpeed = 3.9592 ÷ 0.0550418 × 7.363636 = 529.67 rad/s (5058 rpm); back-EMF = 529.67 ÷ 50.484 = 10.49 V; current = (12 − 10.49) ÷ 0.025205 = 59.83 A (under 150); torque = 1.1765 N·m; wheelForce = 157.40 N; 4 × 157.40 ÷ 60 = 10.493. |
| The one call site; `current` is last loop's command | `DriveToPose.java:197` (`commanded = AccelerationLimiter.limit(commanded, new ChassisVelocities(vx, vy, omega), dt)`); seeded from the measured field velocity at `:152`; sent at `:199`; logged as `DriveToPose/CommandedVelocity` at `:204`. |
| Limits above `DECEL` (6) and `ANGULAR_DECEL` (6) | `AccelerationLimiter.java:26–27`; `DriveToPose.java:38` and `:47`. |
| Try it (Python) | `tools/alignsim/studies/controllers/q7_limiter.py`: add a row to the `CTRL` dict (lines 33–38) with `phys(traction=12.0)`; run `cd tools && python3 alignsim/studies/controllers/q7_limiter.py 60 4 bench`. A quick n = 8 check (from rest, CG 0, 360 A) gave traction 8 / 10 / 12 → slip 117 / 358 / 384 ms, odometry p95 0.5 / 5.2 / 15.8 cm, settle 1.57 / 1.53 / 1.52 s. |
| Try it (Java) | run-sim skill: `./gradlew simulateJavaAgent '-Pmode=auto:2 - Drive To Pose'` (`AutonomousOpMode.java:21`); log-reading skill: `DriveToPose/CommandedVelocity`. This shows the commanded ramp only; tire slip is studied in the Python simulator, whose plant models stick/slip (alignsim README). |
