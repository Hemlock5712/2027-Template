**Q4 sensitivity: VB tuned (30 rest + 30 moving starts)**

| | t settle | settled% | over p95 cm | yaw over p95° | err p95 cm | jitter cm/s | slip ms | odo p95 cm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline (250 Hz, 2 ms) | 1.88 | 100 | 0.1 | 0.0 | 0.05 | 1.00 | 27 | 1.4 |
| actuation +20 ms | 1.85 | 100 | 0.3 | 0.8 | 0.13 | 2.24 | 30 | 1.3 |
| actuation +40 ms | 1.82 | 100 | 2.5 | 4.2 | 0.21 | 2.78 | 32 | 1.2 |
| odometry 100 Hz | 1.87 | 100 | 0.1 | 0.0 | 0.07 | 1.72 | 35 | 2.2 |
| 100 Hz + 20 ms | 1.84 | 100 | 0.7 | 2.1 | 0.15 | 2.43 | 38 | 2.1 |
| white noise 1 cm / 0.5° | 1.85 | 100 | 0.9 | 0.5 | 0.64 | 4.05 | 32 | 1.3 |
| wander 2 cm / 1°, tau 0.3 s | 5.71 | 47 | 5.6 | 2.4 | 3.56 | 10.16 | 46 | 1.4 |
| softer tread (lateral 1500) | 1.88 | 100 | 0.1 | 0.0 | 0.04 | 1.03 | 16 | 3.6 |
| drive on wheel odometry | 1.88 | 100 | 0.1 | 0.2 | 1.37 | 0.03 | 27 | 1.4 |

**Q4 sensitivity: PerAxis2026 db 0.5 (30 rest + 30 moving starts)**

| | t settle | settled% | over p95 cm | yaw over p95° | err p95 cm | jitter cm/s | slip ms | odo p95 cm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline (250 Hz, 2 ms) | 1.92 | 100 | 1.8 | 3.0 | 0.30 | 0.00 | 5 | 1.6 |
| actuation +20 ms | 1.91 | 100 | 6.0 | 5.9 | 0.49 | 0.00 | 9 | 1.7 |
| actuation +40 ms | 2.03 | 100 | 12.4 | 10.5 | 0.43 | 0.00 | 18 | 1.7 |
| odometry 100 Hz | 1.90 | 100 | 4.0 | 6.6 | 0.28 | 0.00 | 6 | 3.1 |
| 100 Hz + 20 ms | 1.97 | 100 | 7.8 | 6.2 | 0.46 | 0.00 | 15 | 3.0 |
| white noise 1 cm / 0.5° | 2.31 | 98 | 6.6 | 9.9 | 1.30 | 8.80 | 305 | 1.6 |
| wander 2 cm / 1°, tau 0.3 s | 5.80 | 35 | 5.4 | 5.3 | 3.96 | 15.45 | 349 | 1.7 |
| softer tread (lateral 1500) | 1.92 | 100 | 1.8 | 3.2 | 0.28 | 0.00 | 0 | 4.0 |
| drive on wheel odometry | 1.91 | 100 | 2.0 | 3.2 | 1.66 | 0.00 | 5 | 1.6 |

**Q4 sensitivity: 581 fixed (30 rest + 30 moving starts)**

| | t settle | settled% | over p95 cm | yaw over p95° | err p95 cm | jitter cm/s | slip ms | odo p95 cm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline (250 Hz, 2 ms) | 2.37 | 100 | 18.3 | 0.2 | 0.00 | 0.00 | 154 | 1.2 |
| actuation +20 ms | 2.46 | 100 | 26.8 | 1.2 | 0.00 | 0.00 | 196 | 1.4 |
| actuation +40 ms | 2.54 | 100 | 35.0 | 3.8 | 0.00 | 0.00 | 248 | 2.1 |
| odometry 100 Hz | 2.42 | 100 | 21.6 | 0.5 | 0.00 | 0.05 | 185 | 1.8 |
| 100 Hz + 20 ms | 2.51 | 100 | 30.1 | 1.8 | 0.00 | 0.00 | 228 | 2.2 |
| white noise 1 cm / 0.5° | 2.40 | 100 | 21.1 | 0.8 | 0.43 | 2.07 | 298 | 1.4 |
| wander 2 cm / 1°, tau 0.3 s | 5.75 | 50 | 20.5 | 3.1 | 3.82 | 6.65 | 226 | 1.3 |
| softer tread (lateral 1500) | 2.37 | 100 | 18.3 | 0.3 | 0.00 | 0.00 | 192 | 2.4 |
| drive on wheel odometry | 2.35 | 100 | 19.1 | 0.2 | 1.18 | 0.00 | 155 | 1.2 |

**Q4 sensitivity: 6328 + limit (30 rest + 30 moving starts)**

| | t settle | settled% | over p95 cm | yaw over p95° | err p95 cm | jitter cm/s | slip ms | odo p95 cm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| baseline (250 Hz, 2 ms) | 2.96 | 100 | 78.6 | 16.8 | 0.88 | 0.00 | 205 | 1.5 |
| actuation +20 ms | 3.19 | 100 | 94.4 | 28.7 | 0.79 | 0.00 | 227 | 1.5 |
| actuation +40 ms | 3.33 | 100 | 108.4 | 40.1 | 0.67 | 0.00 | 249 | 1.7 |
| odometry 100 Hz | 3.02 | 100 | 80.2 | 21.6 | 0.87 | 0.00 | 223 | 1.9 |
| 100 Hz + 20 ms | 3.24 | 100 | 96.3 | 31.2 | 0.75 | 0.00 | 250 | 1.9 |
| white noise 1 cm / 0.5° | 5.00 | 92 | 78.2 | 23.5 | 0.35 | 1.16 | 253 | 1.7 |
| wander 2 cm / 1°, tau 0.3 s | 5.68 | 55 | 77.1 | 22.1 | 3.18 | 3.20 | 258 | 1.6 |
| softer tread (lateral 1500) | 2.97 | 100 | 79.8 | 17.4 | 0.88 | 0.00 | 245 | 3.0 |
| drive on wheel odometry | 2.88 | 100 | 78.8 | 17.1 | 1.14 | 0.00 | 207 | 1.5 |

**Q4 VectorBrake kp under noise and delay**

| | t settle | settled% | over p95 cm | yaw over p95° | err p95 cm | jitter cm/s | slip ms | odo p95 cm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| white noise 1 cm / 0.5°: kp 4.0 | 2.04 | 100 | 0.7 | 0.5 | 0.43 | 2.57 | 25 | 1.4 |
| white noise 1 cm / 0.5°: kp 8.0 | 1.82 | 100 | 1.3 | 0.5 | 0.72 | 5.02 | 50 | 1.3 |
| white noise 1 cm / 0.5°: kp 12.0 | 1.86 | 100 | 2.0 | 0.6 | 1.06 | 6.30 | 121 | 1.4 |
| white noise 1 cm / 0.5°: sqrt only (kp inf) | 2.01 | 100 | 2.2 | 0.6 | 1.31 | 9.28 | 173 | 1.3 |
| actuation +40 ms: kp 4.0 | 1.84 | 100 | 0.8 | 4.3 | 0.20 | 2.37 | 31 | 1.2 |
| actuation +40 ms: kp 8.0 | 1.87 | 100 | 2.5 | 4.3 | 0.23 | 2.68 | 32 | 1.2 |
| actuation +40 ms: kp 12.0 | 1.88 | 100 | 2.5 | 4.3 | 0.25 | 3.46 | 33 | 1.2 |
| actuation +40 ms: sqrt only (kp inf) | 1.88 | 100 | 2.5 | 4.2 | 0.43 | 5.37 | 33 | 1.2 |
| 100 Hz + 20 ms: kp 4.0 | 1.93 | 100 | 0.2 | 2.1 | 0.14 | 2.30 | 38 | 2.2 |
| 100 Hz + 20 ms: kp 8.0 | 1.83 | 100 | 1.5 | 2.1 | 0.15 | 2.61 | 38 | 2.2 |
| 100 Hz + 20 ms: kp 12.0 | 1.83 | 100 | 1.5 | 2.1 | 0.12 | 2.59 | 38 | 2.2 |
| 100 Hz + 20 ms: sqrt only (kp inf) | 1.83 | 100 | 1.5 | 2.1 | 0.29 | 4.89 | 38 | 2.1 |
