"""Small PNGs for docs/alignment/studies/img/: paths of each law (from rest and from a moving
start) and a route with stop-and-go vs pass-through.

    cd tools && python3 alignsim/studies/controllers/plots.py
"""

import math
import os
import sys

sys.path.insert(0, __file__.rsplit("/alignsim/", 1)[0])

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from alignsim.studies.controllers.bench import BenchCfg, Leg, Move, run_move  # noqa: E402
from alignsim.studies.controllers.laws import MATCHED  # noqa: E402
from alignsim.studies.controllers.q1b_fixes import VB_TUNED  # noqa: E402
from alignsim.studies.controllers.q3_route import ROUTES, build, route_moves  # noqa: E402

OUT = os.path.join(__file__.rsplit("/tools/", 1)[0], "docs/alignment/studies/img")
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"


def style(ax, title):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=INK, fontsize=9, loc="left")
    ax.tick_params(colors=MUTED, labelsize=7)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.grid(color=GRID, linewidth=0.6)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)", color=MUTED, fontsize=7)
    ax.set_ylabel("y (m)", color=MUTED, fontsize=7)


def path(fac, move, t_end=4.0):
    r = run_move(fac, move, BenchCfg(t_end=t_end), 3, record=True)
    xs = [tr[1][0] for tr in r["trace"]]
    ys = [tr[1][1] for tr in r["trace"]]
    return xs, ys


def law_paths():
    moves = {
        "From rest: 4 m diagonal + 90° turn": Move((0, 0, 0), (0, 0, 0), [Leg((3.3, 2.3, math.pi / 2))]),
        "Moving 3 m/s sideways, goal 3 m ahead": Move((0, 0, 0), (0, 3.0, 0), [Leg((3.0, 0.0, 0.0))]),
    }
    fig, axes = plt.subplots(1, 2, figsize=(8.0, 3.6), dpi=110, facecolor=SURFACE)
    for ax, (title, mv) in zip(axes, moves.items()):
        style(ax, title)
        for (name, fac), col in zip(MATCHED.items(), SERIES):
            xs, ys = path(fac, mv)
            ax.plot(xs, ys, color=col, linewidth=1.6, label=name)
        g = mv.legs[0].goal
        ax.plot([0, g[0]], [0, g[1]], color=MUTED, linewidth=0.8, linestyle=":")
        ax.plot([g[0]], [g[1]], marker="x", color=INK, markersize=7)
    axes[0].legend(fontsize=7, frameon=False, labelcolor=INK, loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "controllers_paths.png"), facecolor=SURFACE)


def route_paths():
    pts = route_moves(ROUTES["dogleg (90° turn)"], 1, 31)[0]
    fig, ax = plt.subplots(figsize=(4.2, 3.6), dpi=110, facecolor=SURFACE)
    style(ax, "Dogleg, tuned VectorBrake")
    for (label, es, sw), col in zip((("stop-and-go", 0.0, 0.15), ("pass 1.5 m/s, switch 15 cm", 1.5, 0.15),
                                     ("corner rule (r 0.4 m, ve 1.2)", 2.5, "corner")), SERIES):
        r = run_move(VB_TUNED, build(pts, es, sw), BenchCfg(t_end=6.0), 3, record=True)
        ax.plot([tr[1][0] for tr in r["trace"]], [tr[1][1] for tr in r["trace"]], color=col, linewidth=1.6,
                label=f"{label}: {r['t_done']:.2f} s")
    ax.plot([0] + [p[0] for p in pts], [0] + [p[1] for p in pts], color=MUTED, linewidth=0.8, linestyle=":")
    ax.plot([p[0] for p in pts], [p[1] for p in pts], "x", color=INK, markersize=7)
    ax.legend(fontsize=7, frameon=False, labelcolor=INK, loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "controllers_route.png"), facecolor=SURFACE)


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    law_paths()
    route_paths()
    print("wrote", OUT)
