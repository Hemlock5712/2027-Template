# Video series: driving precisely

Briefs for the [animate](https://github.com/cth9191/animate) Claude Code skill, one folder per
video. Each brief answers animate's intake questions, has a beat table per chapter (time, picture,
on-screen text, bridge, sound) and a fact-check table. The `*-voice.json` files are the narration in
animate's `voice/script.json` format, generated from the brief's narration lists.

| # | Video | Explains | Chapters | Length | Look |
| --- | --- | --- | --- | --- | --- |
| 0 | [How a robot hits a 2 cm target](0-trailer/brief.md) | The whole idea in one story (trailer) | 1 | ~1 min | cut paper |
| 1 | [How hard can a robot push?](1-physics/brief.md) | `AccelerationLimiter`: grip, weight shift, turning shares grip, motors at speed | 5 | ~3 min | math |
| 2 | [Fifty decisions a second](2-drive-to-point/brief.md) | `DriveToPose`: the loop, the speed rule, the overshoot race, turning, corners, moving goals | 6 | ~2.7 min | math |
| 3 | [Trust the tag, not the map](3-drive-to-tag/brief.md) | `DriveToTag` / `TagTracker`: the map lies, two poses, old photos, combining photos, camera calibration | 6 | ~3.6 min | math |

**Making one chapter** (in the animate project, with the video's folder copied in):

```
/animate chapter 1 of the explainer in brief.md, use ch1-voice.json as the voice-over
```

Render chapters one at a time, then join them (or post them as a series). Keep the same robot hero
and style across videos 1–3 so they read as one course.

**Before publishing:** fill in the TODOs (team colours and number on the hero), and re-check the
numbers marked "simulation" against the robot test plan in [../README.md](../README.md) section 7 once
real logs exist. None of these videos should claim robot-tested accuracy until then.

**Editing narration:** change the numbered list in the brief, then regenerate that chapter's JSON
(each line becomes `{ "id": "c<chapter>-<line>", "text": ..., "after": 0.6 }`, the last line 0.3; keep
lines under ~3.5 words per second).
