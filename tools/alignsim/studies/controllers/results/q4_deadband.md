**Q4 VectorBrake stop deadband, n=30**

| | success % | false done % | finished % | t done | t done p90 | pos@done p95 cm | yaw@done p95° | jitter cm/s | slip ms | pk acc |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| VB ship + deadband 3 mm / 0.3°, default timing | 100 | 0 | 100 | 1.58 | 1.62 | 0.71 | 0.74 | 0.00 | 0 | 5.0 |
| VB ship + deadband 3 mm / 0.3°, 100 Hz + 20 ms | 100 | 0 | 100 | 1.46 | 1.58 | 0.89 | 0.71 | 0.00 | 1 | 5.0 |
