"""Q4d: shipped VectorBrake with a tiny stop deadband (3 mm / 0.3 deg) on the final approach.

    cd tools && python3 alignsim/studies/controllers/q4_deadband.py [n]
"""

import math
import sys
from dataclasses import replace
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.params import LoopParams  # noqa: E402
from alignsim.studies.controllers.approach import APPROACH_COLS, approach_summary, run_approach  # noqa: E402
from alignsim.studies.controllers.common import Timer, md, save  # noqa: E402
from alignsim.studies.controllers.q4_world import VB_SHIP  # noqa: E402
from alignsim.world import Scenario  # noqa: E402

VB_DB = partial(VB_SHIP, deadband=0.003, wdeadband=math.radians(0.3))
CONDS = {"default timing": Scenario(),
         "100 Hz + 20 ms": Scenario(loop=replace(LoopParams(), odom_period=0.01, actuation_delay=0.022))}

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 30
    rows, out = [], {}
    for cname, sc in CONDS.items():
        with Timer(cname):
            s = approach_summary(run_approach(VB_DB, n, seed0=0, sc=sc))
        rows.append((f"VB ship + deadband 3 mm / 0.3°, {cname}", s))
        out[cname] = s
    t = md(rows, APPROACH_COLS, f"Q4 VectorBrake stop deadband, n={n}")
    print(t)
    save("q4_deadband", out)
    with open(__file__.replace("q4_deadband.py", "results/q4_deadband.md"), "w") as f:
        f.write(t + "\n")
