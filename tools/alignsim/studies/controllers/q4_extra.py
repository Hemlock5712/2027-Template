"""Q4 extras: VectorBrake heading-limit sweep at kp 8 (n=60+60) and the heading reaction margin
(wreact) under delay (n=30+30). Writes results/q4_tune_vb_rot.json and results/q4_wreact.json.

    cd tools && python3 alignsim/studies/controllers/q4_extra.py
"""

import sys
from dataclasses import replace
from functools import partial

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

from alignsim.studies.controllers.bench import BenchCfg, summarize_moves  # noqa: E402
from alignsim.studies.controllers.common import make_moves, md, run_batch, save  # noqa: E402
from alignsim.studies.controllers.q4_sensitivity import COLS as SCOLS, CONDS  # noqa: E402
from alignsim.studies.controllers.q4_tune_vb import COLS, vb  # noqa: E402
from alignsim.studies.controllers.variants import VectorBrakeX  # noqa: E402

ROT = {
    "kp8 rot 6/8": vb(kp=8.0, wdecel=6.0, waccel=8.0),
    "kp8 rot 5/8": vb(kp=8.0, wdecel=5.0, waccel=8.0),
    "kp8 rot 6/10": vb(kp=8.0, wdecel=6.0, waccel=10.0),
    "kp8 rot 8/10": vb(kp=8.0, wdecel=8.0, waccel=10.0),
    "kp8 rot 8/12": vb(kp=8.0, wdecel=8.0, waccel=12.0),
    "kp8 rot 6/8 kp_w 4": vb(kp=8.0, wdecel=6.0, waccel=8.0, kp_w=4.0),
    "kp8 rot 6/8 wmax 4": vb(kp=8.0, wdecel=6.0, waccel=8.0, wmax=4.0),
}
T = dict(vmax=4.0, decel=4.0, accel=5.0, kp=6.0, wmax=6.0, wdecel=6.0, waccel=8.0)

if __name__ == "__main__":
    moves = make_moves(60, 11, speed=(0.0, 0.0)) + make_moves(60, 12, speed=(0.5, 3.0))
    rows = [(k, summarize_moves(run_batch(f, moves, BenchCfg(t_end=6.0)), 6.0)) for k, f in ROT.items()]
    print(md(rows, COLS + [("yaw_p95", "yaw end p95", "{:.2f}")]))
    save("q4_tune_vb_rot", dict(rows))
    moves = make_moves(30, 11, speed=(0.0, 0.0)) + make_moves(30, 12, speed=(0.5, 3.0))
    rows = []
    for c in ("baseline (250 Hz, 2 ms)", "100 Hz + 20 ms", "actuation +40 ms"):
        for wr in (0.0, 0.05, 0.1):
            for react in ((0.05, 0.1) if wr == 0.1 else (0.05,)):
                cfg = replace(CONDS[c], t_end=6.0)
                fac = partial(VectorBrakeX, wreact=wr, react=react, **T)
                rows.append((f"{c}: wreact {wr}, react {react}", summarize_moves(run_batch(fac, moves, cfg), 6.0)))
    print(md(rows, SCOLS))
    save("q4_wreact", dict(rows))
