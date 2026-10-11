**Q2 final approach, odom-frame tracker, no injected jump, n=100**

| | success % | false done % | finished % | t done | t done p90 | pos@done p95 cm | yaw@done p95° | jitter cm/s | slip ms | pk acc |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| VectorBrake | 100 | 0 | 100 | 1.77 | 1.81 | 0.83 | 0.66 | 0.02 | 8 | 5.0 |
| Trailblazer581 | 100 | 0 | 100 | 1.82 | 1.85 | 0.76 | 0.66 | 0.01 | 107 | 10.6 |
| PerAxis2026 | 50 | 38 | 88 | 1.50 | 1.53 | 1.35 | 1.70 | 0.00 | 0 | 5.0 |
| Rolling6328 | 21 | 79 | 100 | 1.26 | 1.28 | 3.68 | 0.59 | 0.00 | 311 | 26.2 |
| ProfiledPerAxis | 100 | 0 | 100 | 1.97 | 2.02 | 0.72 | 0.56 | 0.03 | 129 | 9.4 |
| LinearPathPID | 57 | 16 | 73 | 1.82 | 1.86 | 1.40 | 1.41 | 0.03 | 48 | 8.1 |
| VectorBrake (repo defaults) | 100 | 0 | 100 | 1.86 | 1.88 | 0.80 | 0.63 | 0.03 | 32 | 6.0 |
| VectorBrake tuned (kp 8, rot 6/8) | 100 | 0 | 100 | 1.52 | 1.54 | 0.73 | 0.63 | 0.90 | 0 | 5.0 |
| 581 fixed (vector accel 5, no floor) | 99 | 1 | 100 | 2.13 | 2.20 | 0.90 | 0.63 | 0.01 | 21 | 5.0 |

**Q2 final approach, odom-frame tracker, goal jumps 1-5 cm when 5 cm out, n=60**

| | success % | false done % | finished % | t done | t done p90 | pos@done p95 cm | yaw@done p95° | jitter cm/s | slip ms | pk acc |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| VectorBrake | 100 | 0 | 100 | 1.78 | 1.89 | 0.72 | 0.65 | 0.05 | 8 | 5.0 |
| Trailblazer581 | 100 | 0 | 100 | 1.80 | 1.95 | 0.73 | 0.62 | 0.00 | 106 | 10.7 |
| PerAxis2026 | 47 | 42 | 88 | 1.52 | 1.59 | 1.85 | 1.67 | 0.00 | 1 | 5.0 |
| Rolling6328 | 10 | 90 | 100 | 1.28 | 1.40 | 5.69 | 0.70 | 0.00 | 318 | 35.7 |
| ProfiledPerAxis | 100 | 0 | 100 | 1.97 | 2.01 | 0.71 | 0.57 | 0.01 | 132 | 10.7 |
| LinearPathPID | 58 | 17 | 75 | 1.82 | 1.86 | 1.49 | 1.49 | 0.04 | 47 | 8.1 |
| VectorBrake (repo defaults) | 100 | 0 | 100 | 1.85 | 1.97 | 0.77 | 0.58 | 0.02 | 30 | 6.0 |
| VectorBrake tuned (kp 8, rot 6/8) | 100 | 0 | 100 | 1.59 | 1.65 | 0.65 | 0.54 | 0.88 | 0 | 5.0 |
| 581 fixed (vector accel 5, no floor) | 100 | 0 | 100 | 2.14 | 2.23 | 0.89 | 0.59 | 0.02 | 20 | 5.0 |
