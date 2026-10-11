**Q4 final approach sensitivity, n=40**

| | success % | false done % | finished % | t done | t done p90 | pos@done p95 cm | yaw@done p95° | jitter cm/s | slip ms | pk acc |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| VB ship (kp 6, rot 6/8, wreact 0.05), odometry 100 Hz | 100 | 0 | 100 | 1.56 | 1.59 | 0.76 | 0.66 | 1.58 | 2 | 5.0 |
| PerAxis2026 db 0.5, odometry 100 Hz | 100 | 0 | 100 | 1.53 | 1.57 | 0.74 | 0.65 | 0.00 | 0 | 5.0 |
| VB ship (kp 6, rot 6/8, wreact 0.05), actuation +20 ms | 100 | 0 | 100 | 1.48 | 1.51 | 0.76 | 0.68 | 2.12 | 0 | 5.0 |
| PerAxis2026 db 0.5, actuation +20 ms | 100 | 0 | 100 | 1.51 | 1.62 | 0.95 | 0.47 | 0.00 | 0 | 5.0 |
| VB ship (kp 6, rot 6/8, wreact 0.05), 100 Hz + 20 ms | 100 | 0 | 100 | 1.46 | 1.58 | 0.89 | 0.62 | 2.45 | 2 | 5.0 |
| PerAxis2026 db 0.5, 100 Hz + 20 ms | 100 | 0 | 100 | 1.59 | 1.77 | 0.93 | 0.60 | 0.00 | 3 | 5.0 |
| VB tuned but kp 12, default timing | 100 | 0 | 100 | 1.47 | 1.49 | 0.74 | 0.59 | 1.24 | 0 | 5.0 |
| VB sqrt only (kp inf), default timing | 100 | 0 | 100 | 1.48 | 1.56 | 0.73 | 0.59 | 1.84 | 0 | 5.0 |
