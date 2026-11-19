"""Gate and ramp the optional double-support ankle correction; no ROS."""

import math
from my_robot_description.sensing.balance import balance_proposal


def active_correction(
    pitch, rate, imu_age, contacts, elapsed, config, ramp_seconds=1.0
):
    if not math.isfinite(ramp_seconds) or ramp_seconds <= 0:
        raise ValueError("Positive balance ramp required")
    eligible = (
        elapsed is not None
        and math.isfinite(elapsed)
        and elapsed >= 0
        and imu_age is not None
        and math.isfinite(imu_age)
        and -0.01 <= imu_age <= 0.05
        and len(contacts) == 2
        and all(
            s["ground"] and math.isfinite(s["age"]) and -0.01 <= s["age"] <= 0.05
            for s in contacts
        )
    )
    if not eligible:
        return dict(balance_nm=[0.0] * 6, balance_gate=False, balance_ramp=0.0)
    ramp = min(1.0, elapsed / ramp_seconds)
    proposal = balance_proposal(pitch, rate, config)
    return dict(
        balance_nm=[ramp * x for x in proposal["joint_proposal_nm"]],
        balance_gate=True,
        balance_ramp=ramp,
    )
