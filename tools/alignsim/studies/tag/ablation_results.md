# Error-source ablation (before the back-off teach fix)

n=60 per cell. `python3 alignsim/studies/tag/ablation.py 60 2`

**Success % (finished AND truly < 2 cm & < 1 deg), n=60**

| error source | global tag-only | fused hist | odom KF | odom KF taught | geo | geo taught |
|---|---|---|---|---|---|---|
| none | 100 | 100 | 100 | 100 | 100 | 100 |
| element 4cm/2deg | 38 | 100 | 100 | 100 | 100 | 100 |
| cam mount yaw 0.8deg | 100 | 95 | 95 | 93 | 92 | 92 |
| cam mount pitch 0.8deg | 100 | 100 | 100 | 100 | 100 | 100 |
| cam mount xy 5mm | 100 | 100 | 100 | 100 | 100 | 100 |
| focal length 1% | 100 | 100 | 100 | 100 | 100 | 100 |
| principal pt 4px | 100 | 100 | 100 | 100 | 100 | 100 |
| pnp yaw bias 1deg | 100 | 82 | 83 | 80 | 82 | 78 |
| clock bias 10ms | 100 | 100 | 100 | 100 | 100 | 100 |
| global pose 8cm off | 63 | 100 | 100 | 100 | 100 | 100 |
| wheel radius 2% | 100 | 100 | 100 | 100 | 100 | 100 |

**95th-percentile position error at done (cm) / yaw (deg)**

| error source | global tag-only | fused hist | odom KF | odom KF taught | geo | geo taught |
|---|---|---|---|---|---|---|
| none | 0.2 / 0.0 | 0.2 / 0.1 | 0.1 / 0.1 | 0.1 / 0.1 | 0.2 / 0.1 | 0.2 / 0.1 |
| element 4cm/2deg | 3.7 / 3.9 | 0.2 / 0.1 | 0.2 / 0.1 | 0.1 / 0.1 | 0.1 / 0.1 | 0.1 / 0.1 |
| cam mount yaw 0.8deg | 0.9 / 0.0 | 0.6 / 1.0 | 0.5 / 1.0 | 0.5 / 1.1 | 0.5 / 1.0 | 0.5 / 1.1 |
| cam mount pitch 0.8deg | 0.2 / 0.0 | 0.2 / 0.1 | 0.2 / 0.1 | 0.4 / 0.2 | 0.2 / 0.1 | 0.4 / 0.2 |
| cam mount xy 5mm | 1.1 / 0.0 | 1.1 / 0.1 | 1.0 / 0.1 | 1.0 / 0.1 | 0.9 / 0.1 | 1.0 / 0.1 |
| focal length 1% | 0.4 / 0.0 | 0.6 / 0.1 | 0.5 / 0.1 | 0.4 / 0.2 | 0.5 / 0.2 | 0.4 / 0.2 |
| principal pt 4px | 0.3 / 0.0 | 0.3 / 0.2 | 0.3 / 0.2 | 0.3 / 0.2 | 0.3 / 0.2 | 0.3 / 0.2 |
| pnp yaw bias 1deg | 0.2 / 0.0 | 1.3 / 1.3 | 1.3 / 1.3 | 1.3 / 1.4 | 1.3 / 1.3 | 1.3 / 1.4 |
| clock bias 10ms | 0.4 / 0.0 | 0.4 / 0.1 | 0.4 / 0.1 | 0.3 / 0.1 | 0.3 / 0.1 | 0.3 / 0.1 |
| global pose 8cm off | 1.9 / 2.0 | 0.2 / 0.1 | 0.2 / 0.1 | 0.1 / 0.1 | 0.1 / 0.1 | 0.2 / 0.1 |
| wheel radius 2% | 0.2 / 0.0 | 0.2 / 0.1 | 0.2 / 0.1 | 0.2 / 0.1 | 0.2 / 0.1 | 0.2 / 0.1 |

Note: the 'taught' columns here were run before teaching was fixed (the tag is out of view at the goal, so teaching silently fell back to the nominal goal). See tag study report for corrected numbers.
