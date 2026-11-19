"""Bounded torque-valued integral state with conditional-integration anti-windup."""

import math


class BoundedIntegral:
    def __init__(self, gains, limits):
        if (
            len(gains) != 6
            or len(limits) != 6
            or any(not math.isfinite(x) or x < 0 for x in (*gains, *limits))
        ):
            raise ValueError(
                "Integral gains and torque limits require six finite nonnegative entries"
            )
        if any(k > 0 and limit == 0 for k, limit in zip(gains, limits)):
            raise ValueError(
                "Enabled integral gain requires a positive contribution limit"
            )
        self.gains = list(gains)
        self.limits = list(limits)
        self.state = [0.0] * 6

    def update(self, error, dt, base_request, motor_limits):
        if not math.isfinite(dt) or not 0 <= dt <= 0.05:
            raise ValueError("Integral timestep must be 0–0.05 simulated second")
        if any(len(v) != 6 for v in (error, base_request, motor_limits)):
            raise ValueError("Integral update requires six joint entries")
        if (
            any(
                not math.isfinite(x)
                for v in (error, base_request, motor_limits)
                for x in v
            )
            or min(motor_limits) <= 0
        ):
            raise ValueError("Invalid integral feedback or motor limit")
        for i, (gain, limit, e, base, motor) in enumerate(
            zip(self.gains, self.limits, error, base_request, motor_limits)
        ):
            if gain == 0:
                self.state[i] = 0.0
                continue
            candidate = max(-limit, min(limit, self.state[i] + gain * e * dt))
            increment = candidate - self.state[i]
            output = base + candidate
            # Freeze only increments that push an already clipped request farther out.
            if (output > motor and increment > 0) or (
                output < -motor and increment < 0
            ):
                continue
            self.state[i] = candidate
        return self.state.copy()
