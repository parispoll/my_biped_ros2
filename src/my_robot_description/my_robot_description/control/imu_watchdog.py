"""Separate first-message discovery from an established IMU stream's watchdog."""


def check_imu_watchdog(started_wall, last_receipt_wall, now_wall):
    """All arguments are monotonic wall seconds; None means no sample yet.

    Allow up to ten seconds for initial discovery. After the first valid sample,
    retain the original two-second stream-loss deadline, even during startup.
    """
    if last_receipt_wall is None:
        if now_wall - started_wall > 10:
            raise RuntimeError(
                "Balance observer did not receive its first IMU sample within ten wall seconds"
            )
    elif now_wall - last_receipt_wall > 2:
        raise RuntimeError("Balance observer IMU absent for two wall seconds")
