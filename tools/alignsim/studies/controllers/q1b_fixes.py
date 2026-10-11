"""Q1b: the same long moves with the tuned VectorBrake and with each failing law's minimal fix
(vector acceleration limit, no 0.5 m/s floor), to separate 'bad law' from 'missing limiter'.

    cd tools && python3 alignsim/studies/controllers/q1b_fixes.py [n]
"""

import sys
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.studies.controllers.bench import BenchCfg, summarize_moves  # noqa: E402
from alignsim.studies.controllers.common import MOVE_COLS, Timer, make_moves, md, run_batch, save  # noqa: E402
from alignsim.studies.controllers.laws import MATCHED  # noqa: E402
from alignsim.studies.controllers.variants import Trailblazer581X, VecLimited, VectorBrakeX  # noqa: E402

T581 = dict(vmax=4.0, amax=5.0, wmax=6.0, walpha=12.0)
VB_TUNED = partial(VectorBrakeX, vmax=4.0, decel=4.0, accel=5.0, kp=8.0, wmax=6.0, wdecel=6.0, waccel=8.0)
LAWS = {
    "VectorBrake tuned (kp 8, rot 6/8)": VB_TUNED,
    "581, floor 0 only": partial(Trailblazer581X, floor=0.0, **T581),
    "581, vector accel 5 only": partial(Trailblazer581X, vec_accel=5.0, **T581),
    "581 fixed (both)": partial(Trailblazer581X, floor=0.0, vec_accel=5.0, **T581),
    "6328 + vector limit": partial(VecLimited, MATCHED["Rolling6328"]),
    "LinearPathPID + vector limit": partial(VecLimited, MATCHED["LinearPathPID"]),
}

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 150
    cfg = BenchCfg(t_end=7.0)
    starts = {"rest": make_moves(n, 11, speed=(0.0, 0.0)), "moving": make_moves(n, 12, speed=(0.5, 3.0))}
    out, tables = {}, []
    for start, moves in starts.items():
        rows = []
        for law, fac in LAWS.items():
            with Timer(f"{start}/{law}"):
                r = run_batch(fac, moves, cfg)
            s = summarize_moves(r, cfg.t_end)
            rows.append((law, s))
            out[f"{start}/{law}"] = s
        tables.append(md(rows, MOVE_COLS, f"Q1b fixes, start {start}, n={n} (same moves as Q1)"))
        print(tables[-1], flush=True)
    save("q1b_fixes", out)
    with open(__file__.replace("q1b_fixes.py", "results/q1b_fixes.md"), "w") as f:
        f.write("\n\n".join(tables) + "\n")
