"""Q2 final approach under realistic localization: default Scenario, odom-frame tracker, controller
varied. Also a late goal jump (1-5 cm, when the robot first gets within 5 cm of the goal).

    cd tools && python3 alignsim/studies/controllers/q2_approach.py [n] [n_jump]
"""

import sys
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.controllers import VectorBrake  # noqa: E402
from alignsim.studies.controllers.approach import APPROACH_COLS, approach_summary, run_approach  # noqa: E402
from alignsim.studies.controllers.common import Timer, md, save  # noqa: E402
from alignsim.studies.controllers.laws import MATCHED  # noqa: E402
from alignsim.studies.controllers.variants import Trailblazer581X, VectorBrakeX  # noqa: E402

LAWS = dict(MATCHED)
LAWS["VectorBrake (repo defaults)"] = VectorBrake
LAWS["VectorBrake tuned (kp 8, rot 6/8)"] = partial(VectorBrakeX, vmax=4.0, decel=4.0, accel=5.0, kp=8.0,
                                                     wmax=6.0, wdecel=6.0, waccel=8.0)
LAWS["581 fixed (vector accel 5, no floor)"] = partial(Trailblazer581X, vmax=4.0, amax=5.0, wmax=6.0,
                                                        walpha=12.0, floor=0.0, vec_accel=5.0)

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 100
    nj = int(sys.argv[2]) if len(sys.argv) > 2 else 60
    out, tables = {}, []
    for title, kw, nn in (("no injected jump", {}, n),
                          ("goal jumps 1-5 cm when 5 cm out", {"jump": (0.01, 0.05), "trigger": 0.05}, nj)):
        rows = []
        for law, fac in LAWS.items():
            with Timer(f"{title}/{law}"):
                rs = run_approach(fac, nn, seed0=0, **kw)
            s = approach_summary(rs)
            rows.append((law, s))
            out[f"{title}/{law}"] = {"summary": s}
        tables.append(md(rows, APPROACH_COLS, f"Q2 final approach, odom-frame tracker, {title}, n={nn}"))
        print(tables[-1], flush=True)
    save("q2_approach", out)
    with open(__file__.replace("q2_approach.py", "results/q2_approach.md"), "w") as f:
        f.write("\n\n".join(tables) + "\n")
