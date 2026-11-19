"""Independent evidence checks for ten-joint standing and passive damping."""

import math
from my_robot_description.model.roll_stance import JOINT_NAMES, MOTOR_CAPS
from my_robot_description.control.roll_posture import KP, KD, INTEGRAL_CAPS


def audit_posture(rows, plan, start, end):
    selected = [r for r in rows if start <= r["sim_time"] <= end]
    if len(selected) < 2:
        raise ValueError("Missing roll posture evidence")
    residual = 0.0
    caps_ok = True
    for r in selected:
        if not r["active"] or not r["ready"] or r.get("integral_frozen_gap", False):
            raise ValueError("Inactive/unsettled/gapped standing sample")
        for i, cap in enumerate(MOTOR_CAPS):
            actual_error = plan["target_rad"][i] - r["position"][i]
            expected = (
                plan["feedforward_nm"][i]
                + KP[i] * actual_error
                - KD[i] * r["velocity"][i]
                + r["integral"][i]
            )
            values = [
                actual_error,
                expected,
                r["command"][i],
                r["requested"][i],
                r["integral"][i],
            ]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("Nonfinite motor evidence")
            residual = max(
                residual,
                abs(actual_error - r["error"][i]),
                abs(expected - r["requested"][i]),
                abs(max(-cap, min(cap, expected)) - r["command"][i]),
            )
            caps_ok &= (
                abs(r["command"][i]) <= cap + 1e-9
                and abs(r["integral"][i]) <= INTEGRAL_CAPS[i] + 1e-9
            )
    gaps = [b["sim_time"] - a["sim_time"] for a, b in zip(selected, selected[1:])]
    max_error = max(abs(e) for r in selected for e in r["error"])
    speed = max(abs(v) for r in selected for v in r["velocity"])
    coverage = (
        0 < min(gaps)
        and max(gaps) <= 0.1
        and selected[0]["sim_time"] - start <= 0.1
        and end - selected[-1]["sim_time"] <= 0.1
    )
    return dict(
        passed=caps_ok
        and residual < 1e-9
        and max_error < 0.002
        and speed < 0.002
        and coverage,
        samples=len(selected),
        max_error_rad=max_error,
        peak_speed_rad_s=speed,
        max_gap_s=max(gaps),
        max_composition_error_nm=residual,
        peak_commands_nm=[
            max(abs(r["command"][i]) for r in selected) for i in range(10)
        ],
        final_position_rad=selected[-1]["position"],
        final_command_nm=selected[-1]["command"],
        saturated_samples=sum(
            any(abs(a - b) > 1e-9 for a, b in zip(r["command"], r["requested"]))
            for r in selected
        ),
    )


def audit_damping(rows):
    expected = {n: b for n, b in zip(JOINT_NAMES, [0.2, 2.5, 3, 2.5, 0.2] * 2)}
    caps = dict(zip(JOINT_NAMES, MOTOR_CAPS))
    if {r["joint"] for r in rows} != set(expected):
        raise ValueError("Missing ten-joint damping coverage")
    error = 0.0
    power = -float("inf")
    motor_ok = True
    for r in rows:
        b, v, tau, p, block, motor, total, peak = [
            float(r[k])
            for k in [
                "coefficient",
                "velocity",
                "torque",
                "power",
                "max_power_in_block",
                "motor_nm",
                "total_nm",
                "max_abs_motor_in_block_nm",
            ]
        ]
        if (
            not all(math.isfinite(x) for x in [b, v, tau, p, block, motor, total, peak])
            or b != expected[r["joint"]]
        ):
            raise ValueError("Invalid roll damping evidence")
        if not 1 <= int(r["steps"]) <= 10:
            raise ValueError("Invalid physics sample block")
        error = max(error, abs(tau + b * v), abs(p - tau * v), abs(total - motor - tau))
        power = max(power, p, block)
        motor_ok &= abs(motor) <= peak + 1e-9 and peak <= caps[r["joint"]] + 1e-9
    return dict(
        passed=error < 1e-9 and power <= 1e-12 and motor_ok,
        samples=len(rows),
        max_constitutive_error=error,
        max_sampled_block_power_w=power,
        motor_input_caps_passed=motor_ok,
        note="Physical damping law and sampled motor input bounds checked; not a full discrete energy convergence proof.",
    )
