"""Controller factories for the study: 'ported' = class defaults (what copying each team's code
gives), 'matched' = the same translation/heading limits for every law, so differences are the law.

Matched limits: vmax 4.0 m/s, accel 5 m/s^2 (VectorBrake plans braking at 4 and limits at 5),
heading 6 rad/s and 12 rad/s^2. Per-axis laws get vmax/sqrt2 and accel/sqrt2 per axis so no
direction exceeds the vector limits.
"""

import math
from functools import partial

from alignsim.controllers import (LinearPathPID, PerAxis2026, ProfiledPerAxis, Rolling6328,
                                  Trailblazer581, VectorBrake)

from .variants import (LinearPathPT, ProfiledPerAxisPT, Rolling6328PT, Trailblazer581X,
                       VectorBrakeX)

V, A, W, AW = 4.0, 5.0, 6.0, 12.0
S2 = math.sqrt(2)

PORTED = {
    "VectorBrake": VectorBrake,
    "Trailblazer581": Trailblazer581,
    "PerAxis2026": PerAxis2026,
    "Rolling6328": Rolling6328,
    "ProfiledPerAxis": ProfiledPerAxis,
    "LinearPathPID": LinearPathPID,
}

MATCHED = {
    "VectorBrake": partial(VectorBrake, vmax=V, decel=4.0, accel=A, wmax=W, wdecel=10.0, waccel=AW),
    "Trailblazer581": partial(Trailblazer581, vmax=V, amax=A, wmax=W, walpha=AW),
    "PerAxis2026": partial(PerAxis2026, vmax=V, friction_accel=A),
    "Rolling6328": partial(Rolling6328, vmax=V, amax=A, wmax=W, walpha=AW),
    "ProfiledPerAxis": partial(ProfiledPerAxis, vmax=V / S2, amax=A / S2, wmax=W, walpha=AW),
    "LinearPathPID": partial(LinearPathPID, vmax=V, amax=A, wmax=W, walpha=AW),
}

# pass-through capable versions (end_speed attribute set per leg by the bench)
MATCHED_PT = {
    "VectorBrake": MATCHED["VectorBrake"],
    "Trailblazer581": partial(Trailblazer581X, vmax=V, amax=A, wmax=W, walpha=AW),
    "PerAxis2026": MATCHED["PerAxis2026"],
    "Rolling6328": partial(Rolling6328PT, vmax=V, amax=A, wmax=W, walpha=AW),
    "ProfiledPerAxis": partial(ProfiledPerAxisPT, vmax=V / S2, amax=A / S2, wmax=W, walpha=AW),
    "LinearPathPID": partial(LinearPathPT, vmax=V, amax=A, wmax=W, walpha=AW),
}

__all__ = ["PORTED", "MATCHED", "MATCHED_PT", "VectorBrakeX", "Trailblazer581X", "V", "A", "W", "AW"]
