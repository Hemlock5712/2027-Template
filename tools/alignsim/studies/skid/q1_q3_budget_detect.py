"""Q1 odometry error budget + Q3 skid detection. Cameras off: the robot drives to the map goal on
wheel odometry alone, so every centimeter of error is odometry.

    python3 alignsim/studies/skid/q1_q3_budget_detect.py [n_scale] [--period 0.010]
"""

import functools
import math
import os
import pickle
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from skidsim import SIGNALS, Config, Push, SkidWorld, pct, window_error  # noqa: E402

from alignsim.controllers import VectorBrake  # noqa: E402
from alignsim.params import LoopParams  # noqa: E402
from alignsim.world import Errors, Scenario, Strategy  # noqa: E402

OUT = os.environ.get("SKID_OUT", os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
KEEP = ("t", "e", "slip") + SIGNALS + ("speed",)


def scen(period=0.004, **kw):
    base = dict(cameras=(), start_dist=4.0, t_end=4.0, errors=Errors(wheel_radius=0.0),
                loop=LoopParams(odom_period=period))
    base.update(kw)
    return Scenario(**base)


def cases(period):
    acc = lambda a: Strategy(controller=functools.partial(VectorBrake, accel=a))  # noqa: E731
    c = {}
    c["clean"] = (scen(period), Strategy(), Config(radius_error=0.0), 100)
    c["clean, big turn (start heading +-90 deg)"] = (scen(period, start_heading=math.radians(90)), Strategy(),
                                                     Config(radius_error=0.0), 100)
    for r in (-0.02, -0.01, 0.01, 0.02):
        c[f"wheel radius {r * 100:+.0f}%"] = (scen(period), Strategy(), Config(radius_error=r), 30)
    for a in (8, 10, 12):
        c[f"accel limit {a} m/s^2"] = (scen(period), acc(a), Config(radius_error=0.0), 80)
    for ang in (90, 180):
        for a in (6, 12):
            c[f"reversal {ang} deg at 2 m/s, accel {a}"] = (
                scen(period, start_speed=2.0), acc(a), Config(radius_error=0.0, start_vel_angle=math.radians(ang)), 60)
    for f in (300, 500, 800):
        for d in (0.2, 0.5):
            c[f"push {f} N x {d} s"] = (scen(period), Strategy(), Config(
                radius_error=0.0, pushes=(Push(phase="time", t=(0.4, 1.6), force=(f, f), dur=(d, d)),)), 40)
    return c


def run_case(sc, st, cfg, n):
    cfg = Config(**{**cfg.__dict__, "record": True})
    rows = []
    for seed in range(n):
        r = SkidWorld(sc, seed, cfg).run(st)
        s = r.pop("series")
        t_stop = r["t_done"] if r["done"] else float(s[-1, 0])
        r["drift_all"] = window_error(s, 0.0, t_stop)
        r["drift_1s"] = window_error(s, t_stop - 1.0, t_stop)
        r["t_stop"] = t_stop
        cols = {k: i for i, k in enumerate(("t", "tx", "ty", "tth", "ox", "oy", "oth", "e", "slip")
                                            + SIGNALS + ("speed",))}
        r["s"] = s[:, [cols[k] for k in KEEP]]
        rows.append(r)
    return rows


def budget_table(data):
    lines = ["| case | n | approach s | drift whole approach cm (med / p95 / max) | drift last 1 s cm "
             "(med / p95) | heading drift deg p95 | slip ms |", "|---|---|---|---|---|---|---|"]
    for name, rows in data.items():
        da = np.array([r["drift_all"][0] for r in rows]) * 100
        d1 = np.array([r["drift_1s"][0] for r in rows]) * 100
        hy = np.array([math.degrees(r["drift_all"][1]) for r in rows])
        ts = np.median([r["t_stop"] for r in rows])
        sl = np.mean([r["slip_time"] for r in rows]) * 1000
        lines.append(f"| {name} | {len(rows)} | {ts:.1f} | {np.median(da):.2f} / {pct(da, 95):.2f} / {da.max():.1f} | "
                     f"{np.median(d1):.2f} / {pct(d1, 95):.2f} | {pct(hy, 95):.3f} | {sl:.0f} |")
    return "\n".join(lines)


# ---------------------------------------------------------------------------- detection analysis
def episodes(s, dt, rate=0.05, gap=0.04):
    """Truth: intervals where odometry misses > `rate` m/s of motion; returns (t0, t1, error m)."""
    t, e = s[:, 0], s[:, 1]
    bad = e / dt > rate
    out = []
    i = 0
    n = len(t)
    while i < n:
        if not bad[i]:
            i += 1
            continue
        j = i
        k = i
        while k < n and (bad[k] or (t[k] - t[j] < gap)):
            if bad[k]:
                j = k
            k += 1
        out.append((float(t[i]), float(t[j]), float(e[i:j + 1].sum())))
        i = j + 1
    return out


def detection(data, dt, harm=0.005, thresholds=None):
    """Per detector + threshold: % of harmful episodes caught (within -20 ms .. +100 ms of the
    episode), median catch delay, % of clean runs with any alarm, alarms per minute of clean driving."""
    clean_keys = [k for k in data if k.startswith("clean")]
    event_keys = [k for k in data if not k.startswith("clean") and not k.startswith("wheel radius")]
    col = {k: i for i, k in enumerate(KEEP)}
    res = {}
    for sig, ths in thresholds.items():
        c = col[sig]
        for th in ths:
            caught, total, delays = 0, 0, []
            fa_runs, runs, fa_events, minutes = 0, 0, 0, 0.0
            by_case = {}
            for key in clean_keys + event_keys:
                for r in data[key]:
                    s = r["s"]
                    t = s[:, 0]
                    over = s[:, c] > th
                    eps = [ep for ep in episodes(s, dt) if ep[2] > harm]
                    if key in event_keys:
                        for (a, b, err) in eps:
                            m = (t >= a - 0.02) & (t <= b + 0.1)
                            hit = bool(over[m].any())
                            total += 1
                            caught += hit
                            bc = by_case.setdefault(key, [0, 0])
                            bc[0] += hit
                            bc[1] += 1
                            if hit:
                                delays.append(float(t[m][over[m]][0]) - a)
                    if key not in clean_keys:
                        continue
                    # false alarms (clean runs only): rising edges of the alarm, 0.15 s hold
                    edges, last = 0, -1e9
                    for tk in t[over]:
                        if tk - last > 0.15:
                            edges += 1
                        last = tk
                    fa_events += edges
                    minutes += (t[-1] - t[0]) / 60
                    runs += 1
                    fa_runs += bool(over.any())
            res[(sig, th)] = {
                "caught_%": 100 * caught / max(total, 1), "episodes": total,
                "delay_ms": 1000 * float(np.median(delays)) if delays else float("nan"),
                "clean_runs_alarm_%": 100 * fa_runs / max(runs, 1),
                "fa_per_min": fa_events / max(minutes, 1e-9),
                "by_case": {k: 100 * v[0] / v[1] for k, v in by_case.items()},
            }
    return res


THRESHOLDS = {
    "skid_ratio": (1.2, 1.3, 1.5, 2.0),
    "skid_dev": (0.1, 0.15, 0.25, 0.4),
    "ls_resid": (0.03, 0.05, 0.1, 0.2),
    "imu_acc": (2.0, 3.0, 4.0, 6.0),
    "imu_vel": (0.08, 0.1, 0.12, 0.2),
    "cur_frac": (0.6, 0.8, 0.95, 0.99),
    "cmd_err": (0.3, 0.5, 0.8, 1.2),
}


def detection_table(res, cases_shown):
    lines = ["| detector | threshold | harmful episodes caught % | median delay ms | clean runs with an alarm % "
             "| false alarms / min | " + " | ".join(cases_shown) + " |",
             "|---|---|---|---|---|---|" + "---|" * len(cases_shown)]
    for (sig, th), v in res.items():
        bc = [f"{v['by_case'].get(k, float('nan')):.0f}" for k in cases_shown]
        lines.append(f"| {sig} | {th:g} | {v['caught_%']:.0f} (of {v['episodes']}) | {v['delay_ms']:.0f} | "
                     f"{v['clean_runs_alarm_%']:.0f} | {v['fa_per_min']:.2f} | " + " | ".join(bc) + " |")
    return "\n".join(lines)


def detect_rules(data, dt, rules):
    """A detector = OR of (signal > threshold) rules. Catch % by episode size, share of all missed
    odometry error that it catches, delay, and clean-run alarms."""
    col = {k: i for i, k in enumerate(KEEP)}
    clean_keys = [k for k in data if k.startswith("clean")]
    event_keys = [k for k in data if not k.startswith("clean") and not k.startswith("wheel radius")]
    sizes = (0.005, 0.01, 0.02)
    caught = {h: [0, 0] for h in sizes}
    err_tot = err_caught = 0.0
    delays, clean_alarm, clean_runs = [], 0, 0
    for key in clean_keys + event_keys:
        for r in data[key]:
            s = r["s"]
            t = s[:, 0]
            over = np.zeros(len(t), bool)
            for sig, th in rules:
                over |= s[:, col[sig]] > th
            if key in clean_keys:
                clean_runs += 1
                clean_alarm += bool(over.any())
                continue
            for (a, b, err) in episodes(s, dt):
                m = (t >= a - 0.02) & (t <= b + 0.1)
                hit = bool(over[m].any())
                for h in sizes:
                    if err > h:
                        caught[h][0] += hit
                        caught[h][1] += 1
                if err > 0.005:
                    err_tot += err
                    err_caught += err * hit
                    if hit:
                        delays.append(float(t[m][over[m]][0]) - a)
    return {**{f"caught_{int(h * 1000)}mm_%": 100 * c[0] / max(c[1], 1) for h, c in caught.items()},
            "error_share_%": 100 * err_caught / max(err_tot, 1e-12),
            "delay_ms": 1000 * float(np.median(delays)) if delays else float("nan"),
            "clean_alarm_%": 100 * clean_alarm / max(clean_runs, 1)}


RULES = {
    "Orbit skid ratio > 1.3": [("skid_ratio", 1.3)],
    "module deviation > 0.25 m/s": [("skid_dev", 0.25)],
    "LS residual > 0.05 m/s": [("ls_resid", 0.05)],
    "stator current > 95% of limit": [("cur_frac", 0.95)],
    "module cmd-vs-measured > 0.5 m/s": [("cmd_err", 0.5)],
    "IMU accel disagreement > 3 m/s^2": [("imu_acc", 3.0)],
    "IMU velocity disagreement > 0.12 m/s": [("imu_vel", 0.12)],
    "SHIP: IMU vel > 0.12 OR IMU acc > 3 OR module dev > 0.25": [("imu_vel", 0.12), ("imu_acc", 3.0),
                                                                  ("skid_dev", 0.25)],
    "kinematic only: ratio > 1.3 OR dev > 0.25 OR LS > 0.05": [("skid_ratio", 1.3), ("skid_dev", 0.25),
                                                               ("ls_resid", 0.05)],
}


def rules_table(data, dt):
    lines = ["| detector | episodes > 5 mm caught % | > 1 cm | > 2 cm | share of missed odometry error caught % "
             "| median delay ms | clean runs with an alarm % |", "|---|---|---|---|---|---|---|"]
    for name, rules in RULES.items():
        v = detect_rules(data, dt, rules)
        lines.append(f"| {name} | {v['caught_5mm_%']:.0f} | {v['caught_10mm_%']:.0f} | {v['caught_20mm_%']:.0f} | "
                     f"{v['error_share_%']:.0f} | {v['delay_ms']:.0f} | {v['clean_alarm_%']:.0f} |")
    return "\n".join(lines)


if __name__ == "__main__":
    scale = float(sys.argv[1]) if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else 1.0
    period = float(sys.argv[sys.argv.index("--period") + 1]) if "--period" in sys.argv else 0.004
    tag = f"{int(round(1 / period))}hz"
    os.makedirs(OUT, exist_ok=True)
    path = os.path.join(OUT, f"q1q3_{tag}.pkl")
    if os.path.exists(path) and "--rerun" not in sys.argv:
        data = pickle.load(open(path, "rb"))
    else:
        data = {}
        for name, (sc, st, cfg, n) in cases(period).items():
            t0 = time.time()
            data[name] = run_case(sc, st, cfg, max(5, int(n * scale)))
            print(f"{name}: {time.time() - t0:.0f}s", flush=True)
        pickle.dump(data, open(path, "wb"))
    print(budget_table(data))
    print()
    res = detection(data, period, thresholds=THRESHOLDS)
    shown = ["accel limit 12 m/s^2", "reversal 90 deg at 2 m/s, accel 12", "push 500 N x 0.5 s", "push 800 N x 0.2 s",
             "push 800 N x 0.5 s"]
    print(detection_table(res, shown))
    print()
    print(rules_table(data, period))
