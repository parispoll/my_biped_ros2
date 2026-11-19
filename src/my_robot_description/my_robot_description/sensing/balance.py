"""Sagittal balance proposals only: no ROS, publishers, or actuator commands."""

from dataclasses import dataclass
import math
from my_robot_description.control.posture_effort import quaternion_tilt
from my_robot_description.model.torque import clamp_torque


@dataclass(frozen=True)
class BalanceConfig:
    # Total body restoring gains, split equally between two grounded ankles.
    kp: float = 20.0  # N m/rad
    kd: float = 4.0  # N m s/rad
    ankle_limit: float = 0.5  # N m per ankle, proposal only
    target_pitch: float = 0.0  # rad, world upright

    def validate(self):
        if not all(
            math.isfinite(v)
            for v in (self.kp, self.kd, self.ankle_limit, self.target_pitch)
        ):
            raise ValueError("Balance settings must be finite")
        if min(self.kp, self.kd) < 0 or self.ankle_limit <= 0:
            raise ValueError(
                "Balance gains must be nonnegative and proposal limit positive"
            )
        return self


def sagittal_imu(x, y, z, w, wx, wy, wz):
    """World-referenced ZYX pitch and its derivative from body-frame gyro.

    IMU axes must match base_link. pitch_dot = cos(roll)*wy - sin(roll)*wz.
    """
    quaternion_tilt(x, y, z, w)  # Reuse the existing orientation validity check.
    if not all(math.isfinite(v) for v in (wx, wy, wz)):
        raise ValueError("Invalid IMU angular velocity")
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = (v / norm for v in (x, y, z, w))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return pitch, math.cos(roll) * wy - math.sin(roll) * wz


def balance_proposal(pitch, pitch_rate, config=BalanceConfig()):
    """Positive pitch is forward (+X); positive ankle motor torque opposes it.

    Fixed flat feet imply body_pitch = -(hip+knee+ankle). The ankle motor's
    equal/opposite reaction acts on the upper robot; do not negate it twice.
    This double-support approximation is a candidate law, not a tuned controller.
    """
    config.validate()
    if not all(math.isfinite(v) for v in (pitch, pitch_rate)):
        raise ValueError("Invalid pitch feedback")
    error = pitch - config.target_pitch
    proportional = config.kp * error
    derivative = config.kd * pitch_rate
    raw = 0.5 * (proportional + derivative)
    ankle = clamp_torque(raw, config.ankle_limit)
    return dict(
        pitch_error_rad=error,
        proportional_total_nm=proportional,
        derivative_total_nm=derivative,
        raw_per_ankle_nm=raw,
        proposal_per_ankle_nm=ankle,
        proposal_clipped=abs(raw - ankle) > 1e-12,
        joint_proposal_nm=[0.0, 0.0, ankle, 0.0, 0.0, ankle],
    )
