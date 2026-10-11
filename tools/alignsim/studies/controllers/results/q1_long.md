**Q1 matched limits, start rest, n=150**

| | t 5cm | t settle | settled% | over p95 cm | err p95 cm | yaw p95° | slip ms | odo err p95 cm | pk acc | max acc | pk jerk | path dev p95 cm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| VectorBrake | 1.72 | 1.88 | 100 | 0.1 | 0.00 | 0.00 | 159 | 1.0 | 5.0 | 5 | 260 | 9 |
| Trailblazer581 | 1.68 | 1.86 | 100 | 0.1 | 0.00 | 0.00 | 244 | 1.0 | 25.0 | 25 | 1000 | 10 |
| PerAxis2026 | 1.68 | 1.70 | 55 | 2.1 | 1.22 | 1.69 | 6 | 1.1 | 5.0 | 5 | 499 | 58 |
| Rolling6328 | 1.44 | 1.58 | 100 | 3.4 | 0.91 | 0.07 | 372 | 3.0 | 25.7 | 35 | 1026 | 9 |
| ProfiledPerAxis | 1.63 | 2.08 | 100 | 5.1 | 0.00 | 0.00 | 247 | 1.5 | 6.7 | 8 | 288 | 79 |
| LinearPathPID | 1.44 | 1.90 | 100 | 6.6 | 0.00 | 0.00 | 242 | 1.4 | 7.6 | 9 | 332 | 7 |

**Q1 matched limits, start moving, n=150**

| | t 5cm | t settle | settled% | over p95 cm | err p95 cm | yaw p95° | slip ms | odo err p95 cm | pk acc | max acc | pk jerk | path dev p95 cm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| VectorBrake | 1.94 | 2.10 | 100 | 0.1 | 0.00 | 0.00 | 161 | 1.7 | 5.0 | 5 | 210 | 106 |
| Trailblazer581 | 1.77 | 1.95 | 100 | 0.1 | 0.00 | 0.01 | 525 | 145.7 | 105.3 | 302 | 5059 | 53 |
| PerAxis2026 | 1.87 | 1.89 | 51 | 1.6 | 1.21 | 1.72 | 7 | 1.6 | 5.0 | 5 | 500 | 124 |
| Rolling6328 | 1.55 | 1.67 | 100 | 3.2 | 0.91 | 0.07 | 517 | 73.8 | 59.7 | 141 | 2964 | 51 |
| ProfiledPerAxis | 1.92 | 2.39 | 100 | 5.3 | 0.00 | 0.00 | 248 | 2.3 | 6.5 | 86 | 270 | 165 |
| LinearPathPID | 1.60 | 2.03 | 100 | 6.5 | 0.00 | 0.00 | 403 | 60.7 | 52.5 | 150 | 2471 | 59 |

**Q1 matched, moving starts heading away from the goal**

| | away: t settle | away: over p95 cm | away: odo p95 cm |
|---|---:|---:|---:|
| VectorBrake | 2.25 | 0.1 | 1.7 |
| Trailblazer581 | 2.09 | 0.1 | 155.4 |
| PerAxis2026 | 2.16 | 0.3 | 1.5 |
| Rolling6328 | 1.77 | 3.1 | 94.7 |
| ProfiledPerAxis | 2.61 | 5.3 | 2.4 |
| LinearPathPID | 2.22 | 6.5 | 62.8 |

**Q1 ported limits, start rest, n=150**

| | t 5cm | t settle | settled% | over p95 cm | err p95 cm | yaw p95° | slip ms | odo err p95 cm | pk acc | max acc | pk jerk | path dev p95 cm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| VectorBrake | 1.78 | 1.94 | 100 | 0.1 | 0.00 | 0.00 | 301 | 2.8 | 6.0 | 6 | 250 | 8 |
| Trailblazer581 | 1.78 | 1.97 | 100 | 0.1 | 0.00 | 0.00 | 406 | 5.5 | 25.0 | 25 | 1050 | 8 |
| PerAxis2026 | 1.30 | 1.37 | 57 | 2.7 | 1.32 | 1.57 | 733 | 9.5 | 10.8 | 11 | 1077 | 75 |
| Rolling6328 | 1.60 | 1.64 | 100 | 1.8 | 0.95 | 0.07 | 165 | 1.8 | 20.7 | 29 | 796 | 8 |
| ProfiledPerAxis | 1.79 | 2.22 | 100 | 4.5 | 0.00 | 0.00 | 9 | 1.2 | 5.7 | 7 | 251 | 80 |
| LinearPathPID | 1.92 | 2.35 | 100 | 4.4 | 0.00 | 0.00 | 1 | 0.5 | 4.6 | 5 | 158 | 4 |

**Q1 ported limits, start moving, n=150**

| | t 5cm | t settle | settled% | over p95 cm | err p95 cm | yaw p95° | slip ms | odo err p95 cm | pk acc | max acc | pk jerk | path dev p95 cm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| VectorBrake | 1.98 | 2.14 | 100 | 0.1 | 0.00 | 0.00 | 290 | 2.9 | 6.0 | 6 | 246 | 89 |
| Trailblazer581 | 1.86 | 2.04 | 100 | 0.1 | 0.00 | 0.03 | 632 | 141.7 | 104.5 | 301 | 5060 | 54 |
| PerAxis2026 | 1.45 | 1.36 | 59 | 1.4 | 1.33 | 1.73 | 795 | 18.8 | 10.8 | 11 | 1078 | 96 |
| Rolling6328 | 1.68 | 1.71 | 100 | 1.7 | 0.93 | 0.07 | 365 | 60.3 | 59.7 | 141 | 2964 | 49 |
| ProfiledPerAxis | 2.14 | 2.58 | 100 | 4.6 | 0.00 | 0.00 | 21 | 1.9 | 5.7 | 92 | 225 | 168 |
| LinearPathPID | 2.19 | 2.63 | 100 | 4.5 | 0.00 | 0.00 | 177 | 56.6 | 52.3 | 150 | 2471 | 89 |

**Q1 ported, moving starts heading away from the goal**

| | away: t settle | away: over p95 cm | away: odo p95 cm |
|---|---:|---:|---:|
| VectorBrake | 2.28 | 0.1 | 2.9 |
| Trailblazer581 | 2.16 | 0.1 | 151.5 |
| PerAxis2026 | 1.43 | 1.5 | 21.0 |
| Rolling6328 | 1.84 | 1.7 | 73.3 |
| ProfiledPerAxis | 2.86 | 4.6 | 1.9 |
| LinearPathPID | 2.92 | 4.5 | 49.3 |
