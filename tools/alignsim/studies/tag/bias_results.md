# Camera-bias calibration study

n=80 per row. `python3 alignsim/studies/tag/bias.py 80 2`. Hard = element 4 cm / 2 deg, camera mount yaw/pitch 0.8 deg, PnP yaw bias 1 deg, clock bias 5 ms, global pose 8 cm off.

**default, n=80**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| geo, no calibration | 100.0 |   0.0 | 100.0 |  1.86 |   0.38 |   0.82 |   0.63 |   0.66 |  0.02 |   15.7 |   6.0 |
| geo, taught goal |  91.2 |   8.8 | 100.0 |  1.86 |   0.82 |   1.56 |   1.00 |   1.27 |  0.02 |   22.5 |   6.0 |
| geo, per-camera correction | 100.0 |   0.0 | 100.0 |  1.86 |   0.50 |   1.00 |   0.70 |   0.72 |  0.03 |   22.4 |   6.0 |
| geo, recency (q x4), taught goal |  91.2 |   8.8 | 100.0 |  1.85 |   0.84 |   1.56 |   1.05 |   1.24 |  0.08 |   23.1 |   6.0 |
| geo, recency (q x4), per-camera |  98.8 |   1.2 | 100.0 |  1.86 |   0.50 |   1.00 |   0.72 |   0.75 |  0.08 |   23.1 |   6.0 |
**hard, n=80**

| strategy | succ% | false% | fin% | t_done | pos_med | pos_p95 | lat_p95 | yaw_p95 | jitter | slip_ms | pk_acc |
|---|---|---|---|---|---|---|---|---|---|---|---|
| geo, no calibration |  66.2 |  33.8 | 100.0 |  1.86 |   0.79 |   1.82 |   1.76 |   1.95 |  0.16 |   19.4 |   6.0 |
| geo, taught goal |  63.8 |  36.2 | 100.0 |  1.86 |   1.24 |   2.64 |   2.56 |   1.84 |  0.10 |   18.6 |   6.0 |
| geo, per-camera correction | 100.0 |   0.0 | 100.0 |  1.86 |   0.72 |   1.32 |   1.19 |   0.53 |  0.09 |   18.9 |   6.0 |
| geo, recency (q x4), taught goal |  63.8 |  36.2 | 100.0 |  1.86 |   1.26 |   2.62 |   2.55 |   1.88 |  0.14 |   19.5 |   6.0 |
| geo, recency (q x4), per-camera | 100.0 |   0.0 | 100.0 |  1.86 |   0.70 |   1.46 |   1.31 |   0.57 |  0.11 |   19.8 |   6.0 |

[exited with code 0]
