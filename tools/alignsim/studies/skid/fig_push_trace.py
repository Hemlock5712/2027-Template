"""Figure: one 800 N side push while driving. Odometry error with and without IMU bridging, and
what the Orbit skid ratio and the IMU check see. Writes docs/alignment/studies/img/skid-push-trace.png.

    python3 alignsim/studies/skid/fig_push_trace.py
"""

import math
import os
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from skidsim import SERIES_COLS, Config, Push, SkidWorld  # noqa: E402

from alignsim.geom import relative  # noqa: E402
from alignsim.world import Errors, Scenario, Strategy  # noqa: E402

ROOT = __file__.rsplit("/tools/", 1)[0]
OUT = os.path.join(ROOT, "docs", "alignment", "studies", "img", "skid-push-trace.png")
SURFACE, INK, MUTED, GRID = "#fcfcfb", "#0b0b0b", "#52514e", "#e4e3df"
BLUE, ORANGE = "#2a78d6", "#eb6834"


def run(odom):
    sc = Scenario(cameras=(), start_dist=4.0, t_end=2.6, errors=Errors(wheel_radius=0.0))
    push = Push(phase="time", t=(1.0, 1.0), force=(800, 800), dur=(0.2, 0.2), lever=0.0, direction="lateral")
    r = SkidWorld(sc, 3, Config(record=True, radius_error=0.0, odom=odom, pushes=(push,))).run(Strategy())
    s = r["series"].astype(float)
    c = {k: i for i, k in enumerate(SERIES_COLS)}
    t = s[:, 0]
    i0 = int(np.searchsorted(t, 0.9))
    err = []
    for k in range(len(t)):
        if k < i0:
            err.append(0.0)
            continue
        dt = relative(tuple(s[i0, 1:4]), tuple(s[k, 1:4]))
        do = relative(tuple(s[i0, 4:7]), tuple(s[k, 4:7]))
        err.append(100 * math.hypot(do[0] - dt[0], do[1] - dt[1]))
    return t, np.array(err), s[:, c["skid_ratio"]], s[:, c["imu_vel"]], r


def main():
    t, e_ls, ratio, imu_vel, r = run("ls")
    t2, e_br, _, _, _ = run("bridge")
    plt.rcParams.update({"font.size": 9, "axes.edgecolor": MUTED, "axes.labelcolor": INK, "xtick.color": MUTED,
                         "ytick.color": MUTED, "text.color": INK})
    fig, axes = plt.subplots(3, 1, figsize=(7.0, 6.0), sharex=True, facecolor=SURFACE,
                             gridspec_kw={"height_ratios": [1.4, 1, 1]})
    for ax in axes:
        ax.set_facecolor(SURFACE)
        ax.axvspan(1.0, 1.2, color=GRID, zorder=0)
        ax.grid(axis="y", color=GRID, linewidth=0.8)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    ax = axes[0]
    ax.plot(t, e_ls, color=BLUE, linewidth=2)
    ax.plot(t2, e_br, color=ORANGE, linewidth=2)
    ax.text(t[-1], e_ls[-1] + 0.4, "wheel odometry (WPILib) ", color=INK, va="bottom", ha="right", fontsize=8)
    ax.text(t2[-1], e_br[-1] + 0.4, "with IMU bridging ", color=INK, va="bottom", ha="right", fontsize=8)
    ax.set_ylabel("odometry error since\n0.9 s (cm)")
    ax.set_ylim(-0.5, max(e_ls.max(), e_br.max()) * 1.18)
    ax.set_title("800 N side push for 0.2 s (gray band) while driving at ~3 m/s", loc="left", fontsize=10)
    ax = axes[1]
    ax.plot(t, ratio, color=BLUE, linewidth=2)
    ax.axhline(1.3, color=MUTED, linestyle="--", linewidth=1)
    ax.text(0.02, 1.3, "threshold 1.3", color=MUTED, va="bottom", fontsize=8)
    ax.set_ylim(0.95, max(1.6, float(ratio.max()) + 0.05))
    ax.set_ylabel("Orbit skid ratio")
    ax = axes[2]
    ax.plot(t, imu_vel, color=BLUE, linewidth=2)
    ax.axhline(0.12, color=MUTED, linestyle="--", linewidth=1)
    ax.text(0.02, 0.12, "threshold 0.12 m/s", color=MUTED, va="bottom", fontsize=8)
    ax.set_ylabel("IMU vs wheel\nvelocity gap (m/s)")
    ax.set_xlabel("time (s)")
    ax.set_xlim(0.0, t[-1])
    fig.tight_layout()
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    fig.savefig(OUT, dpi=110, facecolor=SURFACE)
    print(OUT, "final error: wheel %.1f cm, bridged %.1f cm, ratio peak %.2f, imu peak %.2f"
          % (e_ls[-1], e_br[-1], ratio.max(), imu_vel.max()))


if __name__ == "__main__":
    main()
