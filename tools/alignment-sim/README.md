# Alignment strategy simulator

A small Monte Carlo test bench for comparing **global** and **local** alignment strategies before
(and between) robot tests. It's mentor/research tooling, not robot code. It needs only Python 3 and
numpy.

```bash
python3 tools/alignment-sim/align_sim.py                     # baseline, 150 trials per strategy
python3 tools/alignment-sim/align_sim.py --scenario all      # every scenario (~2 min)
python3 tools/alignment-sim/align_sim.py --scenario element --trials 300
```

## What it models

- A swerve robot with 50 Hz control, a 10 ms actuation delay, and velocity lag. The controller uses
  the same law and limits as [`DriveToPose`](../../src/main/java/frc/robot/commands/DriveToPose.java).
- 250 Hz wheel odometry with a per-trial scale error, plus a near-perfect gyro.
- One camera with latency and jitter, field of view and range limits, and per-trial mounting error.
  Noise grows with distance: depth ∝ d², sideways ∝ d, and yaw ∝ d. Each trial also has a fixed
  yaw bias in the camera's pose solve.
- A field element whose real pose is off from the map.
- A fused "global" pose that blends odometry with MegaTag2-style updates from the target tag and
  from other field tags. The other tags carry their own per-trial bias.

Every trial starts about 2.8 m out, with a random sideways offset and heading. The robot is
already moving at 1.5 m/s, as if handed off from a path.

## Strategies

These are listed in the docstring at the top of `align_sim.py`. All of them drive on the global pose
until the target is locked; they differ in what they do after that.

| Strategy | What it means | In the robot code |
| --- | --- | --- |
| `global` | Drive to the map goal using the fused field pose | what the old `DriveToPose` would do |
| `global_tagfilter` | Same, but the fused pose only ever sees the target tag | 581/254/604's trick |
| `local_raw` | Servo on the newest raw robot-in-tag pose, with no latency compensation | the old `DriveToTag` |
| `local_fused_hist` | Place the tag with the fused pose at capture time | the `samplePoseAt` approach |
| `local_odom` | Place the tag with the **wheel-only** pose at capture time, then filter | `TagTracker` |
| `local_odom_mapyaw` | `local_odom`, but take the tag's yaw from the field map | 6328's 2025 "txTy" idea |
| `local_odom_prior` | `local_odom`, seeded from the map pose | — |
| `local_odom_taught` | `local_odom` with a taught `tagToGoal` | the Teach Tag Goal OpMode |

## Reading the output

| Column | Meaning |
| --- | --- |
| `pos err` / `yaw` | Final error against the **true** goal (median and 95th percentile) |
| `<2cm&<1deg` | Share of trials that end inside both limits |
| `settle` | Last time the error was outside 2 cm or 1° (4.00 s means it never settled) |
| `end speed` | Average speed over the last 0.5 s, which measures twitching at the end |

## Caveats — read before quoting numbers

- **Every error size in `Cfg` is an assumption.** The numbers include latency, noise, element
  offset, odometry scale and camera bias. Replace them with values measured from robot logs; see
  the test plan in [docs/alignment/README.md](../../docs/alignment/README.md).
- Biases here are constant within a trial, so `local_odom_taught` cancels them perfectly. On a real
  robot, biases change slightly with viewing angle, so expect teaching to help a lot but less
  than this.
- There is no wheel slip, no collisions, and no second camera. Only one tag is tracked.
