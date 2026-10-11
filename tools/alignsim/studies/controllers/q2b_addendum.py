"""Q2b addendum: final approach for the shipped VectorBrake (kp 6) and minimal fixes of the laws that
failed Q2 (6328 + vector limit; PerAxis2026 with 0.5 cm / 0.5° deadbands).

    cd tools && python3 alignsim/studies/controllers/q2b_addendum.py [n] [n_jump]
"""

import math
import sys
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.studies.controllers.approach import APPROACH_COLS, approach_summary, run_approach  # noqa: E402
from alignsim.studies.controllers.common import Timer, md, save  # noqa: E402
from alignsim.studies.controllers.laws import MATCHED  # noqa: E402
from alignsim.studies.controllers.q1b_fixes import LAWS as FIXED, VB_TUNED  # noqa: E402

LAWS = {
    "VectorBrake tuned (kp 6, rot 6/8)": VB_TUNED,
    "6328 + vector limit": FIXED["6328 + vector limit"],
    "PerAxis2026, deadbands 0.5 cm / 0.5°": partial(MATCHED["PerAxis2026"], pos_tol=0.005, rot_tol=math.radians(0.5)),
}

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 80
    nj = int(sys.argv[2]) if len(sys.argv) > 2 else 40
    out, tables = {}, []
    for title, kw, nn in (("no injected jump", {}, n),
                          ("goal jumps 1-5 cm when 5 cm out", {"jump": (0.01, 0.05), "trigger": 0.05}, nj)):
        rows = []
        for law, fac in LAWS.items():
            with Timer(f"{title}/{law}"):
                s = approach_summary(run_approach(fac, nn, seed0=0, **kw))
            rows.append((law, s))
            out[f"{title}/{law}"] = s
        tables.append(md(rows, APPROACH_COLS, f"Q2b addendum, {title}, n={nn}"))
        print(tables[-1], flush=True)
    save("q2b_addendum", out)
    with open(__file__.replace("q2b_addendum.py", "results/q2b_addendum.md"), "w") as f:
        f.write("\n\n".join(tables) + "\n")
