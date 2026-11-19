"""Distinguish fresh feedback after dropped messages from stale feedback."""

import math


def startup_timing_ready(stamp, receipt):
    """Wait for initialized ROS time and feedback within the normal age bounds.

    Joint messages can be discovered before /clock. Zero ROS time is not a
    valid reference for activating the effort controller. This gate only waits
    before activation; integration_timing still rejects bad active feedback.
    """
    if not all(math.isfinite(x) and x > 0 for x in (stamp, receipt)):
        return False
    try:
        integration_timing(stamp, stamp, receipt)
    except ValueError:
        return False
    return True


def integration_timing(previous, stamp, receipt, *, wait_for_startup=False):
    if not all(math.isfinite(x) for x in (previous, stamp, receipt)):
        raise ValueError("Nonfinite feedback time")
    dt = stamp - previous
    age = receipt - stamp
    if dt < 0:
        raise ValueError("Feedback time moved backwards")
    if not -0.02 <= age <= 0.05:
        if wait_for_startup:
            return None  # Discard; never integrate or command from this sample.
        raise ValueError(
            f"Joint feedback timing invalid: source_age={age:.6f} s, source_gap={dt:.6f} s"
        )
    return dict(
        feedback_gap_s=dt,
        joint_source_age_s=age,
        integral_frozen_gap=dt > 0.05,
        integral_dt_s=0.0 if dt > 0.05 else dt,
    )
