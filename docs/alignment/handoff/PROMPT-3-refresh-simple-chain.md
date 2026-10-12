# Prompt 3 (optional, later): refresh the simple swerve chain

Run this in a Workshop-Code session, only when the user wants the schools' swerve lessons brought up
to alpha-7. No zip needed. Paste everything below the line.

---

Workshop-Code's numbered swerve chain (`1-Swerve` → `7-InlineCommands`) is behind alpha-7
conventions. Hemlock5712/Workshop-Site's `CLAUDE.md` names the gaps: a copied `LimelightHelpers.java`
and a subsystem called `Limelight` (alpha-7 uses the LimelightLib 2 vendordep and a `Vision` class),
`TalonFXUtil` and `SimStartup`, the old `frc.robot` packages, and the `swerve-*` rename that is due.
Attach Workshop-Site read-only and follow its "Workshop Content Stack" section. The `mech-*` chain is
the model for current conventions.

**Keep these lessons simple.** They're for local schools. P control per axis (`5-DriveToPoint`), a
trapezoid profile plus PID (`6-ProfiledToPoint`) and the inline camera-only command
(`7-InlineCommands`) stay as the ideas they teach. Don't bring in anything from the team's advanced
track (acceleration limiting from motor physics, the braking-distance speed rule, tag tracking in the
odometry frame, camera corrections), even where it looks like a fix. Bug fixes are fine. Point out
anything else you'd change and let the user decide.

**Before you touch a branch:**

- Propose the new branch names, one commit per lesson, and how the site pages that reference each
  branch will be updated. Wait for the user's OK.
- The site's `CLAUDE.md` describes the rebase-and-force-push workflow (`pnpm workshop-code` in the
  site repo). Never force-push without the user's OK for that branch.
- `./gradlew build` on every branch before pushing.

Report the branch list, what changed on each, and the site pages that now need an update.
