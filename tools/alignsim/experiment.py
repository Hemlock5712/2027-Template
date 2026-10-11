"""Monte Carlo runner + summary statistics."""

import math
import multiprocessing as mp

import numpy as np

from .world import run_trial

POS_OK, YAW_OK = 0.02, math.radians(1.0)


def run_many(scenario, strategy, n=100, seed0=0, processes=4):
    args = [(scenario, strategy, seed0 + i) for i in range(n)]
    if processes <= 1:
        return [run_trial(a) for a in args]
    with mp.get_context("fork").Pool(processes) as pool:
        return pool.map(run_trial, args, chunksize=max(1, n // (processes * 4)))


def _err(e):
    return math.hypot(e[0], e[1]), abs(e[1]), abs(e[0]), abs(math.degrees(e[2]))


def summarize(rows, pos_ok=POS_OK, yaw_ok=YAW_OK):
    """Scoring view: the robot 'scores' the moment it declares itself done."""
    n = len(rows)
    done = [r for r in rows if r["done"]]
    e_done = np.array([_err(r["err_done"]) for r in done]) if done else np.zeros((0, 4))
    e_end = np.array([_err(r["err_end"]) for r in rows])
    good = sum(1 for r in done if math.hypot(*r["err_done"][:2]) < pos_ok and abs(r["err_done"][2]) < yaw_ok)
    out = {
        "n": n,
        "finished_%": 100 * len(done) / n,
        "success_%": 100 * good / n,  # finished AND truly within tolerance
        "false_done_%": 100 * (len(done) - good) / n,  # said done, but truly outside tolerance
        "t_done_med": float(np.median([r["t_done"] for r in done])) if done else float("nan"),
        "pos_done_med_cm": float(np.median(e_done[:, 0]) * 100) if done else float("nan"),
        "pos_done_p95_cm": float(np.percentile(e_done[:, 0], 95) * 100) if done else float("nan"),
        "lat_done_p95_cm": float(np.percentile(e_done[:, 1], 95) * 100) if done else float("nan"),
        "yaw_done_p95_deg": float(np.percentile(e_done[:, 3], 95)) if done else float("nan"),
        "pos_end_med_cm": float(np.median(e_end[:, 0]) * 100),
        "pos_end_p95_cm": float(np.percentile(e_end[:, 0], 95) * 100),
        "end_speed_cm_s": float(np.mean([r["end_speed"] for r in rows]) * 100),
        "slip_ms": float(np.mean([r["slip_time"] for r in rows]) * 1000),
        "peak_acc": float(np.median([r["peak_cmd_accel"] for r in rows])),
    }
    return out


COLS = [("success_%", "succ%", "{:5.1f}"), ("false_done_%", "false%", "{:5.1f}"),
        ("finished_%", "fin%", "{:5.1f}"), ("t_done_med", "t_done", "{:5.2f}"),
        ("pos_done_med_cm", "pos_med", "{:6.2f}"), ("pos_done_p95_cm", "pos_p95", "{:6.2f}"),
        ("lat_done_p95_cm", "lat_p95", "{:6.2f}"), ("yaw_done_p95_deg", "yaw_p95", "{:6.2f}"),
        ("end_speed_cm_s", "jitter", "{:5.2f}"), ("slip_ms", "slip_ms", "{:6.1f}"),
        ("peak_acc", "pk_acc", "{:5.1f}")]


def table(results, title=None):
    """results: list of (label, summary dict) -> markdown table string."""
    lines = []
    if title:
        lines.append(f"**{title}**\n")
    lines.append("| strategy | " + " | ".join(c[1] for c in COLS) + " |")
    lines.append("|---|" + "---|" * len(COLS))
    for label, s in results:
        cells = [fmt.format(s[k]) if not (isinstance(s[k], float) and math.isnan(s[k])) else "-" for k, _, fmt in COLS]
        lines.append(f"| {label} | " + " | ".join(cells) + " |")
    return "\n".join(lines)
