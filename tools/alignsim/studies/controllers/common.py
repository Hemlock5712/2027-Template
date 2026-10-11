"""Shared helpers: run a batch of moves, markdown tables, JSON results."""

import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(HERE, "results")
TOOLS = HERE.rsplit("/alignsim/", 1)[0]
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

from alignsim.studies.controllers.bench import run_move, sample_move, summarize_moves  # noqa: E402


def make_moves(n, seed, **kw):
    rng = np.random.default_rng(seed)
    return [sample_move(rng, **kw) for _ in range(n)]


def run_batch(factory, moves, cfg, seed0=1000):
    rows = []
    for i, (mv, info) in enumerate(moves):
        r = run_move(factory, mv, cfg, seed0 + i)
        r.update(info)
        rows.append(r)
    return rows


def md(rows, cols, title=None):
    """rows: list of (label, dict); cols: list of (key, header, fmt)."""
    out = []
    if title:
        out.append(f"**{title}**\n")
    out.append("| | " + " | ".join(h for _, h, _ in cols) + " |")
    out.append("|---|" + "---:|" * len(cols))
    for label, s in rows:
        cells = []
        for k, _, f in cols:
            v = s.get(k, float("nan"))
            cells.append("-" if v is None or (isinstance(v, float) and math.isnan(v)) else f.format(v))
        out.append(f"| {label} | " + " | ".join(cells) + " |")
    return "\n".join(out)


MOVE_COLS = [
    ("t_in5", "t 5cm", "{:.2f}"), ("t_settle", "t settle", "{:.2f}"), ("settled_%", "settled%", "{:.0f}"),
    ("over_p95_cm", "over p95 cm", "{:.1f}"), ("err_p95_cm", "err p95 cm", "{:.2f}"),
    ("yaw_p95", "yaw p95°", "{:.2f}"), ("slip_ms", "slip ms", "{:.0f}"), ("odo_p95_cm", "odo err p95 cm", "{:.1f}"),
    ("pk_acc", "pk acc", "{:.1f}"), ("pk_acc_max", "max acc", "{:.0f}"), ("pk_jerk", "pk jerk", "{:.0f}"),
    ("dev_p95_cm", "path dev p95 cm", "{:.0f}"),
]


def save(name, obj):
    os.makedirs(RESULTS, exist_ok=True)

    def clean(o):
        if isinstance(o, dict):
            return {k: clean(v) for k, v in o.items() if k != "trace"}
        if isinstance(o, (list, tuple)):
            return [clean(v) for v in o]
        if isinstance(o, float) and math.isnan(o):
            return None
        if isinstance(o, (np.floating,)):
            return float(o)
        return o

    with open(os.path.join(RESULTS, name + ".json"), "w") as f:
        json.dump(clean(obj), f)


class Timer:
    def __init__(self, label):
        self.label = label

    def __enter__(self):
        self.t0 = time.time()

    def __exit__(self, *a):
        print(f"  [{self.label}: {time.time() - self.t0:.0f}s]", flush=True)
