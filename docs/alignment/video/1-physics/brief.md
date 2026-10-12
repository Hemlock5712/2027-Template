# Video 1: "How hard can a robot push?" (the physics in `AccelerationLimiter`)

An in-depth lesson in five chapters. Each chapter is its own short animate piece (20–45 s) with its
own voice-over file, so you can render them one at a time and join them, or post them as a series.
The code it explains: [`AccelerationLimiter.java`](../../../../src/main/java/frc/robot/utils/AccelerationLimiter.java).

For each chapter, run in the animate project:

```
/animate chapter 1 of the explainer in brief.md, use ch1-voice.json as the voice-over
```

Beat times in the tables are a guide: with a voice-over, animate times each beat to its narration line, so the chapter lengths below come from the narration (about 3 words a second).

## Intake answers (same for every chapter)

| Question | Answer |
| --- | --- |
| **Subject** | The physics that decides how fast a swerve robot may speed up, slow down and turn: tire grip, weight shift, turning sharing the grip, and motors weakening at speed. |
| **The one true thing** | A robot can only push as hard as its tires grip, and the grip moves around, so the code checks physics 50 times a second instead of trusting the motors' current limit. |
| **Must include** | The real numbers below (they come from this robot's constants). The three checks in order: turning cap, motors, grip. |
| **Must avoid** | Calling any of this tested on the robot (it is simulation and physics so far). Long equations on screen: show one at a time, built up from pictures. |
| **Formats and length** | 16:9 first, 9:16 second. 20–45 s per chapter, about 3.1 min for the whole video. |
| **Voice** | Narrated voice-over (one JSON per chapter in this folder). |
| **Look** | Math style (diagram-driven, like a whiteboard lesson). Keep the same robot drawing in every chapter. |
| **Hero** | The robot seen from above: a square frame, four swerve wheels, bumpers. TODO: team colours and number. Same hero as videos 2 and 3. |
| **Music** | Composed in code, calm and curious; a small sting at each chapter's key reveal. |
| **Story format** | F5 fixed-hero journey: the robot stays in the middle of the screen while forces, arrows and graphs change around it. Chapter 5 ends with an F1-style "machine" recap. |

## Numbers on screen (from this robot)

Use these exactly; they come from `TunerConstants` and WPILib's Kraken X60 model.

| Thing | Value | How |
| --- | --- | --- |
| Robot mass | 60 kg | `ROBOT_MASS` (TODO: weigh it) |
| Tire grip | about 1 g, 10 m/s² | `TRACTION_ACCEL`; good tread on carpet grips about as hard as gravity pulls |
| Grip per wheel | 147 N | 60 kg × 9.81 m/s² ÷ 4 wheels |
| Motor current that reaches grip | about 57 A per wheel | 147 N through a 2.17" wheel and a 7.36:1 gearbox on a Kraken X60 |
| Hardware current limit | 360 A | `kSlipCurrent` in `TunerConstants`: a ceiling, not a plan |
| Planning current | 150 A | `MODEL_CURRENT`, used only for the motor check |
| Wheel to robot centre | 0.36 m | the module positions in `TunerConstants` |
| Example CG height | 20 cm | an example; the real one is a TODO |
| Turning cap | 8 rad/s² | `ANGULAR_ACCEL` |
| Top speed (motors at full voltage) | 4.53 m/s | CTRE dyno free speed 5785 rpm through the gearing (`kSpeedAt12Volts` says 4.54) |

## Chapter 1: Push and grip (about 45 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–7 s | The robot sits still. An arrow labelled "want: 10 m/s²" grows from its front. A balance shows 60 kg. | **F = m × a** | The formula slides aside; the camera tilts down to the wheels. | Soft tick, music in. |
| 2 | 7–17 s | 600 N appears as one big arrow, then splits into four arrows at the four tire patches, 150 N each. The carpet under each tire pushes back. | **600 N ÷ 4 = 150 N per wheel** | Zoom into one tire. | Four light thuds as the arrows split. |
| 3 | 17–29 s | A tire on carpet. A "grip meter" fills as the push grows; at 147 N it hits the top. Past that, the tire spins in place and smoke puffs. | **GRIP ≈ WEIGHT** | The spinning tire slows; a wheel counter keeps ticking. | Rising tone, then a squeal at the top. |
| 4 | 29–42 s | Split screen: the real robot moved 1 m; the wheel counter says 1.2 m. A ghost robot appears where the robot *thinks* it is. | **SPINNING WHEELS LIE** | The ghost fades; cut to the motor. | A "wrong answer" buzz. |
| 5 | 42–55 s | The motor with two dials: "grip reached" at 57 A, "hardware limit" at 360 A. A hand points at the gap. | **360 A IS A CEILING, NOT A PLAN** | Fade to the robot starting to accelerate (chapter 2). | Settling chord. |

**Narration (ch1-voice.json):**

1. Our robot weighs sixty kilograms. To speed up at ten meters per second squared, something has to push it with six hundred newtons.
2. That push only comes from the floor, through four tires. About one hundred fifty newtons each.
3. A tire can only grip so hard. On carpet, about as hard as the robot's weight presses it down. Push past that, and the wheel just spins.
4. And a spinning wheel lies. It counts distance the robot never moved, so the robot thinks it's somewhere it isn't.
5. Each wheel reaches its grip at about fifty-seven amps. Our motors could pull three hundred sixty. So the current limit is only a ceiling. The code has to plan with physics.

## Chapter 2: Weight moves around (about 40 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–9 s | A person standing on a bus that pulls away leans back. Match-cut to the robot seen from the side, with a dot for its centre of mass 20 cm up. | **CG 20 cm UP** | The robot starts to accelerate. | Bus engine, then a whoosh. |
| 2 | 9–22 s | Load bars over each wheel: front bars shrink, back bars grow. Numbers count to front 76 N, back 218 N at 6 m/s². | **FRONT 76 N · BACK 218 N** | Zoom to a front wheel. | Bars creak like a seesaw. |
| 3 | 22–35 s | All four motors push the same amount (equal arrows). The front wheel's grip meter overflows; it spins. The back wheel's meter is half empty. | **SAME PUSH, LESS GRIP** | Turn the acceleration up to 10. | Squeal from the front. |
| 4 | 35–45 s | At 10 m/s² the front bars are almost gone: 29 N. A dashed tip-over line shows where the front would lift (12.5 m/s²). | **10 m/s² → FRONT 29 N** | A slider labelled "current limit" appears. | Low warning tone. |
| 5 | 45–55 s | The slider drops to a low current limit: every wheel is capped, so the loaded back wheels can't push either, and the robot is slower. The slider goes back up; a "physics plan" box takes over. | **DON'T STARVE THE BACK WHEELS** | Fade to a robot turning (chapter 3). | Two-note "ah-ha". |

**Narration (ch2-voice.json):**

1. When a bus pulls away, you lean back. Robots do too. Our robot's weight sits about twenty centimeters up.
2. Speed up, and weight slides to the back wheels. At six meters per second squared, each front wheel carries seventy-six newtons. Each back wheel, two hundred eighteen.
3. But all four motors push the same. The light front wheels slip first, while the heavy back wheels still have grip to spare.
4. Push to ten, and the front wheels carry almost nothing.
5. A low current limit would choke the back wheels too, right when they could help. So we keep the limit high, and plan the push with physics.

## Chapter 3: Turning shares the grip (about 35 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–10 s | Top view. A circle appears around one wheel: its grip budget, radius 10 m/s². An arrow for "drive forward" fills it to the edge. | **ONE GRIP BUDGET** | The robot starts to spin as well. | A ring-like chime. |
| 2 | 10–24 s | A sideways arrow for "spin up" (8 rad/s² × 0.36 m = 2.9 m/s²) joins. The two arrows form a right triangle; the long side must stay inside the circle, so the forward arrow shrinks to 9.6. | **√(9.6² + 2.9²) = 10** | The triangle pulses. | Pythagoras "click". |
| 3 | 24–38 s | Bad version: the arrows are added end to end and poke outside the circle; the wheel slips. Good version: the whole pair is scaled down together, so the direction of travel stays the same. | **SCALE TOGETHER, KEEP THE DIRECTION** | Zoom out to the robot spinning fast. | Slip squeal, then a clean click. |
| 4 | 38–50 s | A sudden spin change jerks every wheel sideways: sparks. Then the spin ramps gently and the wheels stay planted. A cap labelled 8 rad/s² clamps the ramp. | **SPIN CHANGES ARE CAPPED: 8 rad/s²** | Fade to the robot at high speed (chapter 4). | Jerk sound vs. smooth whirr. |

**Narration (ch3-voice.json):**

1. Each tire has one grip budget. Picture it as a circle.
2. Driving forward uses some. Spinning the robot uses some too. They add like the sides of a right triangle, so spinning up costs a little forward push.
3. When the two together are too much, we shrink both by the same amount. That way the robot keeps heading the same direction, just a little slower.
4. And quick changes in spin are the worst: in our simulator, most wheel slip came from turning, not driving. So spin may only change by eight radians per second squared.

## Chapter 4: Motors get tired at speed (about 35 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–12 s | Inside the motor: a magnet spinning past coils. As it spins faster, a "push-back voltage" arrow grows against the battery's 12 V. | **A SPINNING MOTOR PUSHES BACK** | The arrows turn into a graph. | Rising hum. |
| 2 | 12–28 s | Graph: available acceleration vs speed, from CTRE's dyno data. Flat at 26 m/s² until 3 m/s, then falling: 19 at 3.5, 10 at 4.0, 4 at 4.3, zero at 4.53. A horizontal grip line sits at 10. | **26 → 10 → 0 m/s²** | A dot rides the curve as the robot speeds up. | The hum rises with the dot. |
| 3 | 28–40 s | The dot is below the grip line at low speed (grip limits) and crosses at about 4 m/s; above that, the motor limits. | **SLOW: GRIP LIMITS · FAST: MOTORS LIMIT** | Highlight the crossing point. | A soft "switch" click at the crossing. |
| 4 | 40–50 s | Braking: the arrow points backwards and the motor curve doesn't apply; only grip does. | **BRAKING: GRIP ONLY** | Fade to the machine (chapter 5). | Chord resolves. |

**Narration (ch4-voice.json):**

1. A spinning motor is also a generator. The faster it spins, the harder it pushes back against the battery.
2. So its push fades with speed. At low speed our motors could give twenty-six meters per second squared. At four meters per second, about ten. At top speed, zero.
3. Slow, the tires are the limit. Fast, the motors are. The code checks which one is smaller, using a planning current of one hundred fifty amps.
4. Braking is different: the motors aren't fighting their own speed, so only grip limits it.

## Chapter 5: The machine: `AccelerationLimiter` (about 30 s)

| # | Time | What happens (picture) | On-screen text | Bridge | Sound |
| --- | --- | --- | --- | --- | --- |
| 1 | 0–8 s | A token labelled "wanted velocity" drops into a machine with three stations. | **50 TIMES A SECOND** | The token rolls to station 1. | Conveyor rumble. |
| 2 | 8–16 s | Station 1, a clamp: the spin change is capped at 8 rad/s². | **1. TURNING CAP** | Roll on. | Clamp clunk. |
| 3 | 16–25 s | Station 2, a motor gauge: only if speeding up, the push is cut to what the motors can still add. | **2. MOTORS** | Roll on. | Gauge tick. |
| 4 | 25–35 s | Station 3, the grip circle: driving and turning together are scaled to fit inside 10 m/s². | **3. GRIP** | The token comes out as "allowed velocity". | Ring chime. |
| 5 | 35–45 s | The three lines of code that do each check appear one at a time, each next to its station. | **AccelerationLimiter.limit(current, wanted, dt)** | End card: "Next: where should the robot go?" (video 2). | Success sting. |

Code to show in beat 5 (one line per station, from `AccelerationLimiter.java`):

```java
double alpha = Math.max(-ANGULAR_ACCEL, Math.min(ANGULAR_ACCEL, (wanted.omega - current.omega) / dt));
double motorLimit = motorAccel(wheelSpeed);
double total = Math.hypot(Math.hypot(ax, ay), alpha * BASE_RADIUS);
```

**Narration (ch5-voice.json):**

1. All of this runs fifty times a second, in one small class.
2. The velocity we want goes in. First, cap how fast the spin may change.
3. Second, if we're speeding up, cut the push to what the motors can still give.
4. Third, fit driving and turning inside the tires' grip.
5. What comes out is the fastest change the robot can really make. Next: deciding where to go.

## Fact check

| Claim | Basis |
| --- | --- |
| 600 N to accelerate 60 kg at 10 m/s² | F = m·a. |
| Grip about equal to weight on carpet | μ ≈ 1 for good tread; `TRACTION_ACCEL` = 10 m/s². |
| 57 A per wheel at grip | 147 N × 0.055 m ÷ 7.36 ÷ 0.0197 N·m/A (Kraken X60 FOC, dyno). |
| Spinning wheels over-count distance | Wheel odometry counts wheel turns; the skid study measured it. |
| Weight-shift numbers | Load per wheel = m·g/4 ∓ m·h·a/(4·x), x = 0.254 m from centre to axle line, h = 0.20 m (example). Tip-over at g·x/h = 12.5 m/s². |
| A low current limit is slower with weight shift | Controllers study Q6: at a 0.2 m CG and accel 10, 60 A was 0.4 s slower with about twice the odometry error. |
| Turning uses 2.9 m/s² of grip | 8 rad/s² × 0.359 m; √(10² − 2.9²) = 9.6. |
| Most slip came from turning | Controllers study: about 80% of slip happened while turning, from heading-rate steps. |
| Motor acceleration vs speed | `motorAccel`: Kraken X60 FOC from CTRE's dyno data (9.362 N·m stall, 5785 rpm free), 150 A planning current, 7.36:1, 2.17" wheels, 60 kg: 26 / 19 / 10 / 4 / 0 m/s² at 3 / 3.5 / 4 / 4.3 / 4.53 m/s. |
| Braking uses grip only | The motor check runs only when the acceleration points the same way as the velocity. |
