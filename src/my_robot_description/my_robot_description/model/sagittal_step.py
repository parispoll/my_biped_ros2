"""Offline sagittal leg geometry and smooth swing targets; no ROS or commands.

Coordinates are ankle-joint origin relative to its hip in the pelvis frame.
Positive joint pitch about +Y moves a downward link toward negative X.
The positive-knee branch matches this robot's knee limits.
"""

import math


def ankle_position(model, angles):
    """Forward kinematics returning ankle X/Z and foot pitch, relative to pelvis."""
    hip, knee, ankle = angles
    if not all(math.isfinite(q) for q in angles):
        raise ValueError("Finite joint angles required")
    return (
        -model.thigh * math.sin(hip) - model.shin * math.sin(hip + knee),
        -model.thigh * math.cos(hip) - model.shin * math.cos(hip + knee),
        hip + knee + ankle,
    )


def leg_angles(model, x, z, foot_pitch=0.0):
    """Solve a reachable ankle target and reject singular or out-of-limit poses."""
    if not all(math.isfinite(v) for v in (x, z, foot_pitch)):
        raise ValueError("Finite target required")
    a, b = model.thigh, model.shin
    if min(a, b) <= 0:
        raise ValueError("Positive leg lengths required")
    radius = math.hypot(x, z)
    if radius <= abs(a - b) + 1e-9 or radius >= a + b - 1e-9:
        raise ValueError("Target unreachable or at a straight/folded leg singularity")
    knee = math.acos(
        max(-1.0, min(1.0, (radius * radius - a * a - b * b) / (2 * a * b)))
    )
    hip = math.atan2(-x, -z) - math.atan2(b * math.sin(knee), a + b * math.cos(knee))
    angles = (hip, knee, foot_pitch - hip - knee)
    for name, q, (lower, upper) in zip(("hip", "knee", "ankle"), angles, model.limits):
        if not lower <= q <= upper:
            raise ValueError(f"{name} target outside model limits")
    return angles


def swing_target(start_x, ankle_z, advance, clearance, phase):
    """Flat-foot swing with zero endpoint velocity/acceleration in normalized time.

    This is a geometric path, not a dynamically feasible gait. Pelvis stays fixed.
    The raised sine-squared bump of a quintic phase stays nonnegative.
    """
    if not all(math.isfinite(v) for v in (start_x, ankle_z, advance, clearance, phase)):
        raise ValueError("Finite swing parameters required")
    if not 0 <= phase <= 1 or clearance <= 0:
        raise ValueError("Phase in [0,1] and positive clearance required")
    smooth = phase**3 * (10 + phase * (-15 + 6 * phase))
    lift = clearance * math.sin(math.pi * smooth) ** 2
    return start_x + advance * smooth, ankle_z + lift, 0.0


def preview_step(model, advance=0.03, clearance=0.02, duration=2.0, samples=101):
    """Return a checked one-foot swing from the existing knee=0.4 stance.

    The other leg holds its geometric stance. No contact transition, torque,
    balance, collision, actuator-speed or support feasibility claim is made.
    """
    if not math.isfinite(duration) or duration <= 0:
        raise ValueError("Positive duration required")
    if isinstance(samples, bool) or not isinstance(samples, int) or samples < 3:
        raise ValueError("At least three samples required")
    stance = model.pose(0.4)
    x0, z0, _ = ankle_position(model, stance)
    result = []
    for i in range(samples):
        phase = i / (samples - 1)
        target = swing_target(x0, z0, advance, clearance, phase)
        swing = leg_angles(model, *target)
        result.append(
            dict(
                time_s=phase * duration,
                phase=phase,
                ankle_target_xz_pitch=target,
                swing_hip_knee_ankle_rad=swing,
                stance_hip_knee_ankle_rad=stance,
            )
        )
    return result
