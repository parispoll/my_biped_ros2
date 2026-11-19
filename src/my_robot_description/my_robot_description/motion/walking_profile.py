"""Explicit gait length/timing settings; motor and sensor limits remain separate."""

from dataclasses import dataclass


@dataclass(frozen=True)
class WalkingProfile:
    step_count: int = 3
    step_length_m: float = 0.05
    time_scale: float = 1.0
    transfer_time_scale: float = 0.0
    velocity_feedforward: bool = False

    @property
    def effective_transfer_scale(self):
        return self.transfer_time_scale or self.time_scale

    def validate(self):
        if type(self.step_count) is not int or not 3 <= self.step_count <= 20:
            raise ValueError("Walking count must be an integer from 3 to 20")
        if self.step_length_m not in (0.05, 0.075, 0.1) or self.time_scale not in (
            1.0,
            0.5,
            0.4,
            0.25,
        ):
            raise ValueError("Use 50/75/100 mm length and 1/0.5/0.4/0.25 timing scale")
        if type(self.transfer_time_scale) not in (
            float,
            int,
        ) or self.transfer_time_scale not in (0.0, 0.25, 0.375, 0.5, 1.0):
            raise ValueError(
                "Transfer timing must be 0 (inherit), 0.25, 0.375, 0.5 or 1"
            )
        if type(self.velocity_feedforward) is not bool:
            raise ValueError("Velocity feedforward must be true or false")
        return self

    @property
    def joint_speed_limit_rad_s(self):
        # Explicit experiment envelope, far below the unchanged 2.4 rad/s motor
        # limit. The faster profile must pass measured sensor and torque checks.
        return 0.05 if self.time_scale == 1 else 0.25
