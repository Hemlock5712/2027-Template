"""Q4b sensitivity on the move bench: actuation delay, 100 Hz odometry (SystemCore CAN 2.0), pose
noise, the plant's tread model, and driving on wheel odometry alone. Plus a VectorBrake kp sweep
under noise and delay (the sqrt law's gain near zero).

    cd tools && python3 alignsim/studies/controllers/q4_sensitivity.py [n]
"""

import math
import sys
from dataclasses import replace
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.params import LoopParams, RobotParams  # noqa: E402
from alignsim.studies.controllers.bench import BenchCfg, summarize_moves  # noqa: E402
from alignsim.studies.controllers.common import Timer, make_moves, md, run_batch, save  # noqa: E402
from alignsim.studies.controllers.q1b_fixes import LAWS as FIXED, VB_TUNED  # noqa: E402
from alignsim.studies.controllers.variants import VectorBrakeX  # noqa: E402

LAWS = {
    "VB tuned": VB_TUNED,
    "PerAxis2026 db 0.5": FIXED["PerAxis2026, deadbands 0.5 cm / 0.5°"],
    "581 fixed": FIXED["581 fixed (both)"],
    "6328 + limit": FIXED["6328 + vector limit"],
}
R = math.radians
LP = LoopParams()
CONDS = {
    "baseline (250 Hz, 2 ms)": BenchCfg(),
    "actuation +20 ms": BenchCfg(loop=replace(LP, actuation_delay=0.022)),
    "actuation +40 ms": BenchCfg(loop=replace(LP, actuation_delay=0.042)),
    "odometry 100 Hz": BenchCfg(loop=replace(LP, odom_period=0.01)),
    "100 Hz + 20 ms": BenchCfg(loop=replace(LP, odom_period=0.01, actuation_delay=0.022)),
    "white noise 1 cm / 0.5°": BenchCfg(noise_xy=0.01, noise_yaw=R(0.5)),
    "wander 2 cm / 1°, tau 0.3 s": BenchCfg(noise_xy=0.02, noise_yaw=R(1.0), noise_tau=0.3),
    "softer tread (lateral 1500)": BenchCfg(rp=replace(RobotParams(), lateral_stiffness=1500.0)),
    "drive on wheel odometry": BenchCfg(pose_src="odom"),
}
COLS = [("t_settle", "t settle", "{:.2f}"), ("settled_%", "settled%", "{:.0f}"), ("over_p95_cm", "over p95 cm", "{:.1f}"),
        ("yaw_over_p95", "yaw over p95°", "{:.1f}"), ("err_p95_cm", "err p95 cm", "{:.2f}"),
        ("jitter_cm_s", "jitter cm/s", "{:.2f}"), ("slip_ms", "slip ms", "{:.0f}"), ("odo_p95_cm", "odo p95 cm", "{:.1f}")]

VB_BASE = dict(vmax=4.0, decel=4.0, accel=5.0, wmax=6.0, wdecel=6.0, waccel=8.0)
KP = {f"kp {k}": partial(VectorBrakeX, kp=k, **VB_BASE) for k in (4.0, 8.0, 12.0)}
KP["sqrt only (kp inf)"] = partial(VectorBrakeX, kp=1e6, **VB_BASE)
KP_CONDS = ["white noise 1 cm / 0.5°", "actuation +40 ms", "100 Hz + 20 ms"]

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 40
    moves = make_moves(n, 11, speed=(0.0, 0.0)) + make_moves(n, 12, speed=(0.5, 3.0))
    out, tables = {}, []
    for law, fac in LAWS.items():
        rows = []
        for cname, cfg in CONDS.items():
            cfg = replace(cfg, t_end=6.0)
            with Timer(f"{law}/{cname}"):
                s = summarize_moves(run_batch(fac, moves, cfg), cfg.t_end)
            rows.append((cname, s))
            out[f"{law}/{cname}"] = s
        tables.append(md(rows, COLS, f"Q4 sensitivity: {law} ({n} rest + {n} moving starts)"))
        print(tables[-1], flush=True)
    rows = []
    for cname in KP_CONDS:
        cfg = replace(CONDS[cname], t_end=6.0)
        for k, fac in KP.items():
            with Timer(f"{k}/{cname}"):
                s = summarize_moves(run_batch(fac, moves, cfg), cfg.t_end)
            rows.append((f"{cname}: {k}", s))
            out[f"kp/{cname}/{k}"] = s
    tables.append(md(rows, COLS, "Q4 VectorBrake kp under noise and delay"))
    print(tables[-1], flush=True)
    save("q4_sensitivity", out)
    with open(__file__.replace("q4_sensitivity.py", "results/q4_sensitivity.md"), "w") as f:
        f.write("\n\n".join(tables) + "\n")
