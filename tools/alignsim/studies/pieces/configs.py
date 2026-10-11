"""Strategies and stress scenarios for the game-piece study (shared by run_study.py and plots)."""

import functools
import os
import sys
from dataclasses import replace

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from piece_world import PieceScenario, piece_camera  # noqa: E402
from strategies import BearingPursuit, Hybrid, JavaPiece, KalmanPiece, Strafe2026  # noqa: E402

P = functools.partial

STRATEGIES = {
    "A": ("Java PieceTracker + DriveToPiece, as written", JavaPiece),
    "A'": ("A, placed with the odometry pose now (no latency compensation)", P(JavaPiece, tracker_kw={"at_capture": False})),
    "A+": ("A with reach 0.35 (goal past the bumper)", P(JavaPiece, reach=0.35)),
    "B0": ("2026: driver forward + (-kP * tx) strafe", Strafe2026),
    "B": ("bearing pursuit: gyro at capture - tx, speed from ty", BearingPursuit),
    "B'": ("B with the gyro now (no latency compensation)", P(BearingPursuit, comp=False)),
    "C": ("constant-velocity Kalman in odom, lead + feedforward", KalmanPiece),
    "D": ("hybrid: KF far, commit at the image edge, heading frozen, drive through", P(Hybrid, reach=0.30, commit=0.3)),
    "R": ("recommended: C + no 4 m gate + reach 0.40", P(Hybrid, reach=0.40, tracker_kw={"edge_reject": False})),
}

BASE = PieceScenario()
BLIND_CAM = (piece_camera(x=0.20, z=0.60, pitch_down_deg=5.0),)  # loses the piece ~0.5 m out

SCENARIOS = {
    "base": BASE,
    "pitch 1 deg": replace(BASE, pitch_bias_deg=1.0),
    "pitch 2 deg": replace(BASE, pitch_bias_deg=2.0),
    "noise 1 px": replace(BASE, detector_noise=1.0),
    "noise 5 px": replace(BASE, detector_noise=5.0),
    "noise 5 px + 30% dropouts": replace(BASE, detector_noise=5.0, p_miss=0.3),
    "latency 70 ms + clock 20 ms": replace(BASE, clock_bias_s=0.02, cameras=(piece_camera(pipeline=0.06),)),
    "late: 0.9-1.8 m, 2-3 m/s": replace(BASE, range0=(0.9, 1.8), speed0=(2.0, 3.0), vel_dir_deg=90.0),
    "blind last 0.5 m": replace(BASE, cameras=BLIND_CAM),
    "blind + pitch 2 deg": replace(BASE, cameras=BLIND_CAM, pitch_bias_deg=2.0),
    "blind + height 0.05-0.15": replace(BASE, cameras=BLIND_CAM, h_true=(0.05, 0.15)),
    "height 0.05-0.15 (code 0.10)": replace(BASE, h_true=(0.05, 0.15)),
    "rolling 0.3-1 m/s": replace(BASE, piece_speed=(0.3, 1.0)),
    "two pieces 0.4-0.8 m apart": replace(BASE, second_piece=(0.4, 0.8)),
    "shove in last 0.6 m": replace(BASE, shove=(900.0, 0.15, 0.1, 0.6), mu_scale=0.85),
    "blind + shove": replace(BASE, cameras=BLIND_CAM, shove=(900.0, 0.15, 0.1, 0.5), mu_scale=0.85),
    "ALL (static piece)": replace(BASE, cameras=(piece_camera(x=0.20, z=0.60, pitch_down_deg=5.0, pipeline=0.04),),
                                  pitch_bias_deg=1.0, detector_noise=3.0, p_miss=0.15, clock_bias_s=0.01,
                                  range0=(1.0, 3.5), speed0=(0.0, 3.0), vel_dir_deg=90.0, h_true=(0.07, 0.13),
                                  shove=(700.0, 0.15, 0.1, 0.6), mu_scale=0.85),
}
SCENARIOS["ALL + rolling"] = replace(SCENARIOS["ALL (static piece)"], piece_speed=(0.3, 1.0))
