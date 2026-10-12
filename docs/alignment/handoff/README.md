# Handoff: the advanced drive track

Everything needed to take this research into new sessions. There are two tracks, and they stay
separate:

| Track | Who it's for | What it uses | Where it lives |
| --- | --- | --- | --- |
| **Simple** (today's workshop) | Local schools | `5-DriveToPoint`, `6-ProfiledToPoint`, `7-InlineCommands`; the frc5712.com lessons and videos | Workshop-Code and Workshop-Site (public). **Not replaced.** |
| **Advanced** (this research) | Team 5712 first, maybe shared with other teams later | `AccelerationLimiter`, `DriveToPose`, `DriveToTag` and friends; the three long videos | A private repo (recommended) until we choose to share |

**Heads-up:** 2027-Template is a public template repo, so the branch this was built on
(`claude/drive-point-apriltag-align-w1mpo2`) is public right now. To keep the advanced work private,
push the branch to a private repo, then delete it here. Don't merge it into the template's `main`,
because the template is the schools' starting point.

## The prompts, in order

1. **[PROMPT-1-advanced-code.md](PROMPT-1-advanced-code.md):** build the advanced robot code in
   alpha-7 conventions. Open a new session with Workshop-Code (the reference for conventions) and
   attach the zip.
2. **[PROMPT-2-advanced-videos.md](PROMPT-2-advanced-videos.md):** make the three long videos from
   the briefs, with the code on screen taken from step 1's branches. Do it after step 1 lands.
3. **[PROMPT-3-refresh-simple-chain.md](PROMPT-3-refresh-simple-chain.md)** (optional, later): bring
   the simple swerve chain up to alpha-7 conventions, **keeping it simple**. Run it in a Workshop-Code
   session.

Each prompt warns the new session against copying the wrong things. For the advanced track that means
the simple branches, the old lessons and this template's alpha-6 code shape.

## Decisions each session will ask you for

- **Where the advanced code lives:** a new private repo (recommended), the team's competition repo,
  or public Workshop-Code branches.
- **Gains in the advanced code:** keep the simulator values (fine in a private repo) or ship `0.0`
  placeholders (the site's rule, if it's shared publicly).
- **Game pieces:** include `DriveToPiece` / `PieceTracker` in the advanced track or not. They are
  less tested than the tag work.
- **Video tool:** use animate with the custom "Field Lab" style (the briefs are written for it), or
  the site's `videos-next` engine.

## The zip (`drive-precision-handoff.zip`)

| Folder | What | Use it as |
| --- | --- | --- |
| `README.md`, `PROMPT-*.md` | This file and the prompts | Paste the prompts into new sessions |
| `reference/2027-template/` | The prototype code (`src/main/java`, `vendordeps`, `build.gradle`) | **Logic only.** Its shape is alpha-6 and behind (see the prompts) |
| `reference/research/` | `docs/alignment`: the decisions, test plan and studies | Why each choice was made, and the numbers |
| `reference/sim/alignsim/` | The Python simulator, studies and log replay | Check a port's behaviour; replay robot logs |
| `reference/video/` | Video briefs, `STYLE.md`, voice JSONs | Input for prompt 2 |
| `reference/motor/kraken-x60-foc-dyno.csv` | CTRE dyno data for the Kraken X60 FOC (torque, current, power, efficiency vs rpm) | Source of `AccelerationLimiter`'s motor numbers |

If the zip is out of date, the same files live on the 2027-Template branch above.
