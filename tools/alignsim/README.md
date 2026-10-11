# alignsim: drive-to-point and alignment simulator

A Python test bench for answering "what's the best way to drive to a point and line up precisely
with an AprilTag or game piece" before writing robot code.

**Requirements:** Python 3.11+, `numpy`, `opencv-python-headless` and `pytest`. `matplotlib` is
needed for plots.

```bash
cd tools
python3 -m pytest -q alignsim/tests              # sanity tests (noise-free limits)
python3 alignsim/studies/s0_overview.py 100      # every strategy, default scenario
```

## Architecture (one trial = `World(scenario, seed).run(strategy)`)

| Module | What it models |
| --- | --- |
| `params.py` | Every number: robot, camera, field, loop timing. `ASSUMED` values get replaced from robot logs |
| `plant.py` | Swerve physics at 500 Hz, described below |
| `odometry.py` | WPILib swerve odometry (least squares + gyro) and a **faithful port of WPILib `PoseEstimator`**, including dropping newer stored vision updates when an older one is added |
| `camera.py` | Limelight-like camera, described below |
| `field.py` | Tags with a **map** pose and a **true** pose. The target element is reef-like (neighbor faces), with a rigid element error plus per-tag mounting error, and there are other field tags |
| `controllers.py` | Drive-to-point laws: `VectorBrake` (this repo), `Trailblazer581`, `PerAxis2026` (5712 2026), `Rolling6328`, `ProfiledPerAxis`, `LinearPathPID` |
| `trackers.py` | `GlobalVision` (the template's MT1/MT2 fusion) and local trackers: `RawTargetSpace`, `FrameTracker` (odom or fused frame; measured, map or blend yaw; Kalman or EMA filter), `TxTy6328` |
| `world.py` | Thread timing and the strategy pieces, described below |
| `experiment.py` | `run_many` (multiprocessing), `summarize`, `table` |

**`plant.py`** models each module's steering lag. The drive motor is a Kraken X60 FOC torque-speed
curve with a stator current limit and an onboard velocity loop. Wheel contact is stick/slip: a brush
tire model up to the friction circle, then kinetic sliding. So hard commands really spin wheels, and
**the encoders over-read when that happens**. It also has a gyro with noise, drift and scale error.

**`camera.py`** works like a Limelight:
- It projects true tag corners with the TRUE intrinsics and mount, then adds pixel noise plus
  motion blur.
- It solves with the BELIEVED intrinsics and mount: OpenCV IPPE_SQUARE, returning both solutions
  and the ambiguity.
- It produces MegaTag1 (multi-tag SQPNP) and MegaTag2 (gyro-constrained) poses.
- Timing covers exposure, capture and pipeline latency, the network, and clock-sync bias. Each frame
  carries the true capture time and the time the robot code would compute.

**`world.py`** runs the thread timing:
- physics at 500 Hz;
- the CTRE odometry thread at 250 Hz, which applies the newest request and updates the estimator;
- cameras at their own frame rate;
- the 50 Hz robot loop, which does global vision, then tracker update, goal, controller and
  request.

Strategy pieces:
- `Strategy(controller=..., tracker=TrackerSpec(...), vision=GlobalVision(...), approach=..., teach=...)`.
  The approach is `handoff`, `global`, `staged` or `shaped`.
- `Scenario(...)` sets the start distribution and the per-trial random `Errors`.

## Metrics: the robot "scores" the moment it declares itself done

- **Done** means the *measured* error is inside `pos_tol`/`yaw_tol`, at low speed, for `debounce`
  seconds.
- **`success_%`**: done, and the TRUE error is < 2 cm and < 1°.
- **`false_done_%`**: said done, but truly outside tolerance. This is the dangerous one.
- **`jitter`**: speed in cm/s at the end of the run.
- **`slip_ms`**: time any wheel was sliding.
- **`pk_acc`**: peak commanded acceleration.

## Adding a study

Put a script in `studies/`. Build `Scenario`/`Strategy` variants and call `run_many`. Write results
to `docs/alignment/studies/<name>.md`.

Don't change the core modules for a study. Subclass or wrap them in the study file. Report core
bugs instead, so every study stays comparable.

**Picklability:** `run_many` uses multiprocessing (fork), so tracker factories must be
`TrackerSpec(...)` and controllers must be classes or `functools.partial(...)`, not lambdas or
closures.
