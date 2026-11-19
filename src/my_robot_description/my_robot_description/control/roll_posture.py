"""Pure ten-joint PID torque calculation used by standing and walking adapters."""

import math
from my_robot_description.model.roll_stance import JOINT_NAMES, MOTOR_CAPS
from my_robot_description.model.roll_actuators import ACTIVE

KP = [40.0, 80.0, 100.0, 60.0, 40.0] * 2
# Added actuator inertia needs explicit motor velocity damping at roll axes.
# Legacy gains remain unchanged; this P54 profile is verified separately.
KD = (
    [2.0, 0.1, 0.15, 0.05, 2.0]
    if ACTIVE["name"] == "robotis_p54"
    else [0.1, 0.1, 0.15, 0.05, 0.1]
) * 2
# Pitch integral gains are one quarter of the transferred six-joint values.
KI = [20.0, 20.0, 25.0, 15.0, 20.0] * 2
INTEGRAL_CAPS = [0.5, 1.0, 1.0, 1.0, 0.5] * 2
# Optional slow lateral experiment; neutral-standing defaults remain KI above.
LATERAL_KI = [5.0, 20.0, 25.0, 15.0, 5.0] * 2


class RollPosture:
    def __init__(self, plan, integral_gains=None):
        if plan["joint_names"] != JOINT_NAMES or plan["motor_caps_nm"] != MOTOR_CAPS:
            raise ValueError("Named ten-joint motor profile required")
        self.ki = list(KI if integral_gains is None else integral_gains)
        if len(self.ki) != 10 or any(not math.isfinite(v) or v < 0 for v in self.ki):
            raise ValueError("Ten finite nonnegative integral gains required")
        self.target = plan["target_rad"]
        self.ff = plan["feedforward_nm"]
        self.integral = [0.0] * 10

    def update(
        self,
        position,
        velocity,
        dt=0.0,
        *,
        derivative_gains=None,
        proportional_gains=None
    ):
        if not math.isfinite(dt) or not 0 <= dt <= 0.05:
            raise ValueError("Integral timestep must be 0–0.05 s")
        if any(
            len(v) != 10 or not all(math.isfinite(x) for x in v)
            for v in [position, velocity, self.target, self.ff]
        ):
            raise ValueError("Ten finite joint entries required")
        derivative = list(KD if derivative_gains is None else derivative_gains)
        if len(derivative) != 10 or any(
            not math.isfinite(v) or v < 0 for v in derivative
        ):
            raise ValueError("Ten finite nonnegative derivative gains required")
        proportional = list(KP if proportional_gains is None else proportional_gains)
        if len(proportional) != 10 or any(
            not math.isfinite(v) or v < 0 for v in proportional
        ):
            raise ValueError("Ten finite nonnegative proportional gains required")
        # Position error is target minus measured angle (rad). P restores posture;
        # -Kd * velocity damps motion. Feedforward estimates gravity support.
        error = [a - b for a, b in zip(self.target, position)]
        p = [k * e for k, e in zip(proportional, error)]
        d = [-k * v for k, v in zip(derivative, velocity)]
        base = [f + a + b for f, a, b in zip(self.ff, p, d)]
        # Integrate only when the extra torque will not push further into a cap.
        # This is anti-windup: a saturated motor must not accumulate hidden torque.
        for i, (e, b, cap, ki, ic) in enumerate(
            zip(error, base, MOTOR_CAPS, self.ki, INTEGRAL_CAPS)
        ):
            candidate = max(-ic, min(ic, self.integral[i] + ki * e * dt))
            increment = candidate - self.integral[i]
            if (b + candidate > cap and increment > 0) or (
                b + candidate < -cap and increment < 0
            ):
                continue
            self.integral[i] = candidate
        # Requested torque may exceed a cap; published command is always clipped.
        # Logging both lets the independent audit detect saturation.
        raw = [b + i for b, i in zip(base, self.integral)]
        return dict(
            proportional_gains=proportional,
            derivative_gains=derivative,
            error=error,
            feedforward=self.ff,
            proportional=p,
            damping=d,
            integral=self.integral.copy(),
            requested=raw,
            command=[max(-c, min(c, r)) for c, r in zip(MOTOR_CAPS, raw)],
        )
