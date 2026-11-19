"""Read controller-written startup metadata without querying a live feedback node."""

import json
import math
from pathlib import Path


def load_posture_setup(path):
    data = json.loads(Path(path).read_text())
    if data.get("schema") != 1:
        raise ValueError("Unknown posture setup schema")
    target = data["target_joint_positions_rad"]
    limits = data["motor_limits_nm"]
    if (
        len(target) != 6
        or len(limits) != 6
        or not all(math.isfinite(x) for x in (*target, *limits))
        or min(limits) <= 0
    ):
        raise ValueError("Invalid actual posture targets or motor limits")
    return target, limits
