"""Audit the constitutive viscous law separately from controller torque limits."""

import math


def validate_damping_trace(rows):
    if not rows:
        raise ValueError("Missing per-physics-step damping evidence")
    names = {r["joint"] for r in rows}
    expected = {
        f"{s}_{j}": b
        for s in ("left", "right")
        for j, b in (("hip", 2.5), ("knee", 3.0), ("ankle", 2.5))
    }
    if names != set(expected):
        raise ValueError("Damping evidence must cover all six joints")
    maximum = -float("inf")
    error = 0.0
    motor_ok = True
    motor_logged = all("motor_nm" in r for r in rows)
    for r in rows:
        b, v, tau, power, block = [
            float(r[k])
            for k in (
                "coefficient",
                "velocity",
                "torque",
                "power",
                "max_power_in_block",
            )
        ]
        if (
            not all(math.isfinite(x) for x in (b, v, tau, power, block))
            or b != expected[r["joint"]]
        ):
            raise ValueError("Invalid damping data or changed physical coefficient")
        error = max(error, abs(tau + b * v), abs(power - tau * v))
        maximum = max(maximum, power, block)
        if motor_logged:
            motor, total, peak = [
                float(r[k])
                for k in ("motor_nm", "total_nm", "max_abs_motor_in_block_nm")
            ]
            if not all(math.isfinite(x) for x in (motor, total, peak)):
                raise ValueError("Invalid observed motor input")
            error = max(error, abs(total - motor - tau))
            cap = 10.0 if r["joint"].endswith("knee") else 6.0
            motor_ok = (
                motor_ok
                and abs(motor) <= cap + 1e-9
                and abs(motor) <= peak + 1e-9
                and peak <= cap + 1e-9
            )
        if not 1 <= int(r["steps"]) <= 10:
            raise ValueError("Invalid damping sample block")
    return {
        "passed": error < 1e-9 and maximum <= 1e-12 and motor_ok,
        "max_constitutive_error": error,
        "motor_input_bounds_checked": motor_logged,
        "motor_input_bounds_passed": motor_ok if motor_logged else None,
        "max_sampled_and_block_power_w": maximum,
        "samples": len(rows),
        "note": "Nonpositive tau*v at sampled input velocities; this is not a full discrete energy-convergence proof.",
    }
