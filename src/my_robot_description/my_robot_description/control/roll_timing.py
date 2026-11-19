"""Check latest IMU freshness against the shared clock, not another sensor stamp."""

import math


def imu_source_age(clock_time, imu_stamp, *, wait_for_startup=False):
    if not all(math.isfinite(t) for t in [clock_time, imu_stamp]):
        raise ValueError("Finite IMU/clock times required")
    age = clock_time - imu_stamp
    if not -0.02 <= age <= 0.05:
        if wait_for_startup:
            return None
        raise ValueError(
            f"Pelvis IMU freshness invalid: clock={clock_time:.6f}, imu={imu_stamp:.6f}, age={age:.6f} s"
        )
    return age
