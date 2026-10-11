"""Q1 long moves (path legs): 1-6 m, heading change 0-180 deg, from rest and moving 0.5-3 m/s in a
random direction. Perfect localization (controller sees the true pose at the last odometry tick).

    cd tools && python3 alignsim/studies/controllers/q1_long.py [n]
"""

import sys

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

import numpy as np  # noqa: E402

from alignsim.studies.controllers.bench import BenchCfg, summarize_moves  # noqa: E402
from alignsim.studies.controllers.common import MOVE_COLS, Timer, make_moves, md, run_batch, save  # noqa: E402
from alignsim.studies.controllers.laws import MATCHED, PORTED  # noqa: E402

if __name__ == "__main__":
    n = int(sys.argv[1]) if len(sys.argv) > 1 else 150
    cfg = BenchCfg(t_end=7.0)
    starts = {
        "rest": make_moves(n, 11, speed=(0.0, 0.0)),
        "moving": make_moves(n, 12, speed=(0.5, 3.0)),
    }
    out = {}
    tables = []
    for set_name, laws in (("matched", MATCHED), ("ported", PORTED)):
        for start, moves in starts.items():
            rows = []
            for law, fac in laws.items():
                with Timer(f"{set_name}/{start}/{law}"):
                    r = run_batch(fac, moves, cfg)
                s = summarize_moves(r, cfg.t_end)
                # moving away (> 90 deg between start velocity and goal direction)
                away = [x for x in r if x["vrel"] > np.pi / 2]
                if start == "moving" and away:
                    sa = summarize_moves(away, cfg.t_end)
                    s["away_t_settle"] = sa["t_settle"]
                    s["away_over_p95_cm"] = sa["over_p95_cm"]
                    s["away_odo_p95_cm"] = sa["odo_p95_cm"]
                rows.append((law, s))
                out[f"{set_name}/{start}/{law}"] = {"summary": s, "rows": r}
            tables.append(md(rows, MOVE_COLS, f"Q1 {set_name} limits, start {start}, n={n}"))
            if start == "moving":
                tables.append(md(rows, [("away_t_settle", "away: t settle", "{:.2f}"),
                                        ("away_over_p95_cm", "away: over p95 cm", "{:.1f}"),
                                        ("away_odo_p95_cm", "away: odo p95 cm", "{:.1f}")],
                                 f"Q1 {set_name}, moving starts heading away from the goal"))
            print(tables[-1], flush=True)
    save("q1_long", out)
    with open(__file__.replace("q1_long.py", "results/q1_long.md"), "w") as f:
        f.write("\n\n".join(tables) + "\n")
