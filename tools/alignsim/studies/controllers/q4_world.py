"""Q4c final-approach sensitivity (full World: cameras, estimator, odom-frame tracker): odometry
100 Hz vs 250 Hz and extra actuation delay for the top laws; sqrt-only VectorBrake (no kp zone).

    cd tools && python3 alignsim/studies/controllers/q4_world.py [n]
"""

import sys
from dataclasses import replace
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.params import LoopParams  # noqa: E402
from alignsim.studies.controllers.approach import APPROACH_COLS, approach_summary, run_approach  # noqa: E402
from alignsim.studies.controllers.common import Timer, md, save  # noqa: E402
from alignsim.studies.controllers.q1b_fixes import LAWS as FIXED, VB_TUNED  # noqa: E402
from alignsim.studies.controllers.variants import VectorBrakeX  # noqa: E402
from alignsim.world import Scenario  # noqa: E402

LP = LoopParams()
CONDS = {
    "odometry 100 Hz": Scenario(loop=replace(LP, odom_period=0.01)),
    "actuation +20 ms": Scenario(loop=replace(LP, actuation_delay=0.022)),
    "100 Hz + 20 ms": Scenario(loop=replace(LP, odom_period=0.01, actuation_delay=0.022)),
}
TOP = {
    "VB tuned": VB_TUNED,
    "PerAxis2026 db 0.5": FIXED["PerAxis2026, deadbands 0.5 cm / 0.5°"],
}
VB_T = dict(vmax=4.0, decel=4.0, accel=5.0, wmax=6.0, wdecel=6.0, waccel=8.0)
EXTRA = {
    "VB tuned but kp 12": partial(VectorBrakeX, kp=12.0, **VB_T),
    "VB sqrt only (kp inf)": partial(VectorBrakeX, kp=1e6, **VB_T),
}

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    out, rows = {}, []
    for cname, sc in CONDS.items():
        for law, fac in TOP.items():
            with Timer(f"{cname}/{law}"):
                s = approach_summary(run_approach(fac, n, seed0=0, sc=sc))
            rows.append((f"{law}, {cname}", s))
            out[f"{law}/{cname}"] = s
    for law, fac in EXTRA.items():
        with Timer(law):
            s = approach_summary(run_approach(fac, n, seed0=0))
        rows.append((f"{law}, default timing", s))
        out[law] = s
    t = md(rows, APPROACH_COLS, f"Q4 final approach sensitivity, n={n}")
    print(t)
    save("q4_world", out)
    with open(__file__.replace("q4_world.py", "results/q4_world.md"), "w") as f:
        f.write(t + "\n")
