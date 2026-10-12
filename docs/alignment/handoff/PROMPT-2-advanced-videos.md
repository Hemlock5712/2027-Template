# Prompt 2: make the advanced videos

Run this after prompt 1 has pushed the advanced code. Start a new Claude Code session with the repo
that holds the advanced code (or a private repo for the videos) and the
[animate](https://github.com/cth9191/animate) skill, attach `drive-precision-handoff.zip`, and paste
everything below the line.

---

You're making three long lesson videos, plus a one-minute trailer, for FRC team 5712's **advanced**
drive track. The audience is high-school students on our team and, later, other teams. They aren't
part of the frc5712.com workshop series for local schools, so don't add them there and don't
replace any of its videos.

## Read first

1. Unzip the attachment into your scratchpad, not into the repo.
2. `reference/video/README.md`: the series index, chapter lengths, how voice files are made.
3. `reference/video/STYLE.md`: the custom **Field Lab** style (World, Board and Code modes, the colour
   code for each quantity, motion rules, the five-part lesson format, narration rules).
4. Each video's `brief.md` and its `chNN-voice.json` files.
5. The code on screen comes from **prompt 1's branches** (`precision-1-DriveToPose`,
   `precision-2-DriveToTag`, or whatever the user approved). Ask the user which repo and branches.

## Do not use these as models

- **The frc5712.com videos (`Workshop-Site/videos-next`).** They teach the simple track, in a flatter
  look we're moving away from. That's especially true of `drive-to-point`, `advanced-drive-to-point`
  and `drive-to-tag-inline`.
- **The code in the briefs.** It was copied from the alpha-6 prototype. Re-copy every line from the
  ported branch, and fix line numbers, names and APIs where they changed (`ZERO`, `Telemetry`,
  LimelightLib 2). If a beat's explanation no longer matches the ported code, fix the beat and its
  narration together.
- **The old 2027-Template code**, for anything shown on screen.

## Step 1: build the style once, and get approval

Follow animate's `new-style.md` with `STYLE.md` and 3–6 reference stills (STYLE.md lists what to look
for). Show the user the matched frames, one per mode (World, Board, Code), before any chapter.
Keep the same robot hero in every video. The team colours and number are a TODO: ask.

## Step 2: one chapter at a time

For each chapter: render it, look at frames from every beat, fix overlaps, empty stretches and
anything that contradicts the narration, then move on. Start with video 1, chapter 1, and show it to
the user before doing the rest, so style problems are caught once.

## Accuracy rules

- On-screen code is copied **exactly** from the ported branch. Reformatting long lines is fine;
  changing the API isn't.
- Every number marked "simulation" in a brief stays worded as "in our simulator". Never claim
  robot-tested accuracy until the test plan in `reference/research/README.md` has run.
- **Gains:** ask the user whether `KP` and `KP_HEADING` show on screen. Derived limits (`DECEL`,
  `TRACTION_ACCEL`, motor numbers) come from physics and can show.
- Logging on screen is WPILib `Telemetry`, never AdvantageKit. Lambdas are `() -> foo()`. No enums.
- A command that ends sends nothing. Never animate the robot stopping just because a command ended,
  unless the code sends the zero-velocity request.

## If the user prefers the site's engine instead of animate

Copy the engine into the private repo; don't build in public Workshop-Site. Each brief chapter
becomes a `lines` list (`[id, text]`) and a research brief, following `series/BUILD.md`'s contract
(`createScene(VOICE)`, deterministic `draw`). Field Lab's colour code breaks that engine's one-accent
rule, so it needs its own kit. Show the user a kit preview before converting lessons.

## Report back

Per chapter: its length, one line per beat, the weakest beat, and anything in the brief that didn't
match the ported code.
