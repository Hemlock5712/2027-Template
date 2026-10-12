# Video series: driving precisely

Briefs for the [animate](https://github.com/cth9191/animate) Claude Code skill, one folder per
video. This is the team's advanced track; it isn't part of the schools' workshop series. Each brief
answers animate's intake questions, has a beat table per chapter (time, mode, picture, on-screen
text, bridge, sound), the exact code shown, and a fact-check table. The `chNN-voice.json` files are
the narration in animate's `voice/script.json` format, generated from the brief's numbered lists.

| # | Video | Explains | Chapters | Length | Look |
| --- | --- | --- | --- | --- | --- |
| 0 | [How a robot hits a 2 cm target](0-trailer/brief.md) | The whole idea in one story (trailer) | 1 | ~1 min | cut paper |
| 1 | [How hard can a robot push?](1-physics/brief.md) | `AccelerationLimiter`: grip, weight shift, turning shares grip, motors at speed, the current ceiling | 19 | ~20 min (~22 with pauses) | [Field Lab](STYLE.md) |
| 2 | [Fifty decisions a second](2-drive-to-point/brief.md) | `DriveToPose`: the four questions, the speed rule, braking early, the P-only race, turning, corners, moving goals | 25 | ~22 min (~26) | [Field Lab](STYLE.md) |
| 3 | [Trust the tag, not the map](3-drive-to-tag/brief.md) | `DriveToTag` / `TagTracker`: the map lies, two poses, old photos, combining photos, camera calibration | 23 | ~25 min (~27) | [Field Lab](STYLE.md) |

Every lesson follows the same five parts (goal, idea, maths, code line by line, recap), described in
[STYLE.md](STYLE.md) along with the custom style.

**Making one chapter** (in the animate project, with the video's folder copied in):

```
/animate chapter 1 of the explainer in brief.md, use ch01-voice.json as the voice-over
```

Build the Field Lab style first (STYLE.md says how), then render chapters one at a time and join
them. Keep the same robot hero across videos 1–3 so they read as one course.

**Before publishing:** fill in the TODOs (team colours and number on the hero, robot mass and CG
height), and re-copy the code on screen from the ported alpha-7 branches (see
[../handoff/PROMPT-2-advanced-videos.md](../handoff/PROMPT-2-advanced-videos.md)). Numbers marked
"simulation" stay worded as "in our simulator" until the robot test plan in
[../README.md](../README.md) has run.

**Editing narration:** change the numbered list in the brief, then regenerate that video's JSON files
with `python3 make_voice.py <folder> [question pause]`.
Each line becomes `{ "id": "c<NN>-<line>", "text": ..., "after": 0.6 }`, the last line 0.3; in the
recap chapter the question lines hold longer for the pause card (video 1: 5.0 s, videos 2 and 3:
4.0 s). Keep lines at 30 words or fewer and under about 3.5 words per second.
