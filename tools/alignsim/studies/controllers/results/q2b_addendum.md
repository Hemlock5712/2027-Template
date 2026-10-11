**Q2b addendum, no injected jump, n=80**

| | success % | false done % | finished % | t done | t done p90 | pos@done p95 cm | yaw@done p95° | jitter cm/s | slip ms | pk acc |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| VectorBrake tuned (kp 6, rot 6/8) | 100 | 0 | 100 | 1.59 | 1.62 | 0.77 | 0.62 | 0.06 | 0 | 5.0 |
| 6328 + vector limit | 100 | 0 | 100 | 2.55 | 2.69 | 1.27 | 0.64 | 0.00 | 62 | 5.0 |
| PerAxis2026, deadbands 0.5 cm / 0.5° | 100 | 0 | 100 | 1.54 | 1.57 | 0.75 | 0.68 | 0.00 | 0 | 5.0 |

**Q2b addendum, goal jumps 1-5 cm when 5 cm out, n=40**

| | success % | false done % | finished % | t done | t done p90 | pos@done p95 cm | yaw@done p95° | jitter cm/s | slip ms | pk acc |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| VectorBrake tuned (kp 6, rot 6/8) | 100 | 0 | 100 | 1.61 | 1.68 | 0.77 | 0.57 | 0.07 | 0 | 5.0 |
| 6328 + vector limit | 100 | 0 | 100 | 2.59 | 2.73 | 1.31 | 0.66 | 0.00 | 67 | 5.0 |
| PerAxis2026, deadbands 0.5 cm / 0.5° | 100 | 0 | 100 | 1.58 | 1.65 | 0.78 | 0.64 | 0.00 | 1 | 5.0 |
