# Series style and conventions: "Field Lab"

A custom animate style for the driving series, plus the conventions every video brief follows. Build
the style once with animate's new-style process (`new-style.md`): give it this file and 3–6 reference
stills, approve the matched frames, then every piece uses `"style": "field-lab"`.

## Why a custom style

The shipped styles each cover one part of a lesson. **Math** (manim-like) is perfect for formulas and
graphs, but it is flat by design (no depth, no texture) and has no code layout. **Isometric** gives real
depth for machines, but it is colourless and has no math. **Cut paper** is fun for stories (the trailer)
but too loose for equations and code. A lesson that goes goal → logic → math → code needs all three
looks to feel like **one world**, so the style has three modes in one kit.

## The three modes

| Mode | Used for | Look | Built from |
| --- | --- | --- | --- |
| **World** | The robot, the field, wheels, motors, cameras, forces acting on real things | 2:1 isometric on a dark navy ground; white-faced rounded slabs with hairline edges; real depth (painter's order, faces hide what's behind); the team colour as the one accent (bumpers) | the isometric kit, dark variant, plus colour arrows |
| **Board** | Formulas, graphs, worked numbers | The math kit's conventions on the same navy ground: serif math written on, never popped; axes and graphs drawn in; live readouts driven by the same value that moves the picture | the math kit |
| **Code** | Line-by-line walkthroughs of the real Java | An editor panel (mono, ≥ 34 px at 1920 wide, real line numbers from the file); the line being explained has a soft highlight bar; values of the traced example appear in ghost text at the end of each line, like a debugger; a variable is drawn in its quantity's colour (below) | new component |

**Moving between modes is a camera move, not a cut.** The isometric world tilts flat into the board
(the floor becomes the axes), or the camera pushes into the robot's "brain" and the code panel unfolds
from it. A formula lifts off the board and lands on the code line that implements it.

## The colour code (the one rule that ties it together)

Each quantity has one colour everywhere: in the world (arrows, trails), on the board (symbols, graphs)
and in the code (variable names, ghost values).

| Quantity | Colour | In the code |
| --- | --- | --- |
| distance, position (d, x, y) | green | `distance`, `toGoal`, `brakingDistance` |
| velocity (v) | blue | `speed`, `vx`, `vy`, `commanded`, `current`, `wanted` |
| acceleration (a) | orange | `ax`, `ay`, `DECEL`, `TRACTION_ACCEL`, `motorLimit` |
| heading, turn rate (θ, ω, α) | purple | `headingError`, `omega`, `alpha`, `ANGULAR_ACCEL` |
| force, grip, weight (F, N) | red | `wheelForce` |
| time (t, dt) | yellow | `dt`, `REACTION_TIME`, `frameAge` |
| current, motor (I) | teal | `amps`, `MODEL_CURRENT`, `DRIVE_MOTOR` |
| the tag and vision | white with a thin team-colour outline | `tagPose`, `TagTracker` |

Everything else (keywords, punctuation, comments) stays light grey so the coloured quantities pop.

## Motion and life (from animate's craft rules)

- **Springs** for anything with mass (robots, slabs, arrows growing); smooth sigmoid for formulas.
- **Something new every 2–4 s** (a reveal, a value changing, a camera move). In a code walkthrough the
  highlight moving to the next line counts, but give each line a picture or a ghost value too.
- **Idle life:** the robot's swerve modules twitch to keep angle, the camera LEDs pulse, the field's
  carpet has a faint moving sheen. Nothing is ever a still slide.
- **Scale:** every chapter has at least one extreme close-up (a tire patch, a motor coil, a single pixel
  of the tag) and one wide shot of the field.
- **The camera frames world points:** it follows the robot, pushes into the module being discussed,
  pulls back for the field. No zoom-about-the-centre.
- **One constant for the eye:** the robot's outline in World, the axes on the Board, the line-number
  gutter in Code.

## Hero

The robot from above and in isometric: square frame, four swerve modules (wheel + steering ring),
bumpers in the team colour with the team number (TODO: colour and number), two cameras on the front
corners with small lens LEDs. Same model in every video and in the trailer.

## References to gather for new-style (stills only; kept out of the repo per animate's rules)

- A manim math explainer: colour-coded variables, written-on equations, a live readout on a graph.
- A coding explainer that shows code and its visual effect side by side, with the running line
  highlighted.
- An interactive mechanical explainer with clean isometric or 3D machines (gears, engines) on a dark
  ground.
- An FRC swerve module render (for the hero's wheels and steering).

## Video format conventions (every lesson brief)

Each lesson walks the same path, so students always know where they are:

1. **Goal:** the problem, and why it matters on the field.
2. **The idea:** the logic in plain words, with an analogy and a picture. No maths yet.
3. **The maths:** derived step by step from the picture, with this robot's real numbers and one worked
   example that the code walkthrough will reuse.
4. **The code, line by line:** the real Java, in file order, each line on screen with its line number.
   For each line: what it does, which formula from part 3 it is, and the traced example's value. Long
   boilerplate (imports, logging) is shown once and summarised.
5. **Recap and check yourself:** the whole idea in four sentences, then 3–5 questions with a pause
   card and the answer, and one "try it" change to make in the simulator.

**Narration rules:** high-school level; one idea per line; ≤ 3.5 words per second (aim for 3); lines of
≤ 30 words; say a symbol's name the first time ("a, the acceleration"); say numbers the way they're
written on screen; never claim robot-tested accuracy until the test plan has run ("in our simulator").

**Brief layout:** per chapter, a beat table (time, mode, picture, on-screen text, bridge, sound), then
"Narration (chNN-voice.json)" as a numbered list (one voice line per item), then for code chapters the
exact code shown. Voice files are generated from the numbered lists (see README).
