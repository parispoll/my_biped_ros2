"""Checks for steady-state physics commands and DART's split-step torque balance."""

import math


def stationary_physics_window(joint_rows):
    """Static gravity is inapplicable to a window containing joint motion."""
    if not joint_rows or any(not rows for rows in joint_rows):
        raise ValueError("Missing stationary-window data")
    for rows in joint_rows:
        q = [float(r["q_rad"]) for r in rows]
        v = [float(r["v_rad_s"]) for r in rows]
        if any(not math.isfinite(x) for x in (*q, *v)):
            raise ValueError("Non-finite stationary-window data")
        if max(abs(x) for x in v) >= 1e-6 or max(q) - min(q) >= 1e-6:
            return False
    return True


def audit_explicit_joint(physics, dart, damping, index):
    """Check each observed input sum without assuming the integral is stationary."""
    if min(len(physics), len(dart), len(damping)) < 20:
        raise ValueError("Insufficient explicit-path samples")
    by_time = {round(float(r["sim_time"]), 9): r for r in damping}
    errors = []
    cap = [6.0, 10.0, 6.0] * 2
    for rows, keys in (
        (physics, ("sim_time", "input_last_nm", "ty_world", "external_fx_last_n")),
        (
            damping,
            ("sim_time", "motor_nm", "total_nm", "torque", "max_abs_motor_in_block_nm"),
        ),
        (
            dart,
            (
                "fd_command_nm",
                "fd_force_nm",
                "implicit_damping_nm",
                "damping_nm_s_rad",
                "actuator_type",
                "friction_nm",
                "transmitted_ty_nm",
            ),
        ),
    ):
        if any(not math.isfinite(float(r[k])) for r in rows for k in keys):
            raise ValueError("Non-finite explicit-path data")
    for row in physics:
        damper = by_time.get(round(float(row["sim_time"]), 9))
        if damper is None:
            raise ValueError("Missing matching explicit damper sample")
        errors.append(abs(float(row["input_last_nm"]) - float(damper["total_nm"])))
        errors.append(
            abs(
                float(damper["total_nm"])
                - float(damper["motor_nm"])
                - float(damper["torque"])
            )
        )
    closure = max(abs(split_step_balance(r)["closure_error_nm"]) for r in dart)
    force_difference = max(
        abs(float(r["fd_force_nm"]) - float(r["fd_command_nm"])) for r in dart
    )
    engine_damping = max(
        abs(float(r["implicit_damping_nm"])) + abs(float(r["damping_nm_s_rad"]))
        for r in dart
    )
    max_motor = max(float(r["max_abs_motor_in_block_nm"]) for r in damping)
    modes = sorted({int(r["actuator_type"]) for r in dart})
    friction_ok = all(
        abs(split_step_balance(r)["constraint_torque_nm"])
        <= float(r["friction_nm"]) + 1e-6
        for r in dart
    )
    mean = lambda rows, key: sum(float(r[key]) for r in rows) / len(rows)
    return {
        "passed": max(errors) < 1e-9
        and closure < 1e-6
        and force_difference < 1e-9
        and engine_damping == 0
        and max_motor <= cap[index] + 1e-9
        and modes == [0]
        and friction_ok,
        "max_motor_plus_passive_input_error_nm": max(errors),
        "max_split_step_closure_error_nm": closure,
        "max_dart_command_force_difference_nm": force_difference,
        "max_backend_damping": engine_damping,
        "max_motor_input_nm": max_motor,
        "motor_cap_nm": cap[index],
        "actuator_modes": modes,
        "mean_motor_input_nm": mean(damping, "motor_nm"),
        "constraint_within_coulomb_bound": friction_ok,
        "dart_force_before_constraints_nm": mean(dart, "fd_force_nm"),
        "implicit_damping_nm": mean(dart, "implicit_damping_nm"),
        "joint_constraint_torque_nm": sum(
            split_step_balance(r)["constraint_torque_nm"] for r in dart
        )
        / len(dart),
        "transmitted_joint_moment_nm": mean(dart, "transmitted_ty_nm"),
        "gazebo_transmitted_world_y_nm": mean(physics, "ty_world"),
        "external_force_final_n": max(
            abs(float(r["external_fx_last_n"])) for r in physics
        ),
        "note": "ECM equals observed motor plus explicit passive torque at matching simulation stamps; DART force and wrench closure are checked separately. No stationary-command assumption.",
    }


def final_getter_window(rows, duration=1.0):
    """Select a duration, not a fixed sample count, from every-tenth-step logs."""
    if not rows or not math.isfinite(duration) or duration <= 0:
        raise ValueError("Getter window needs data and a positive finite duration")
    timesteps = {float(row["dt_s"]) for row in rows}
    if len(timesteps) != 1:
        raise ValueError("Variable DART timestep is not supported")
    dt = timesteps.pop()
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("DART timestep must be positive and finite")
    end = int(rows[-1]["sequence"])
    selected = [
        row for row in rows if end - int(row["sequence"]) <= duration / dt + 1e-6
    ]
    span = (end - int(selected[0]["sequence"])) * dt
    if span < duration - 0.05:
        raise ValueError("Getter window has insufficient simulated-time coverage")
    if any(
        int(b["sequence"]) - int(a["sequence"]) != 10
        for a, b in zip(selected, selected[1:])
    ):
        raise ValueError("Missing or unordered getter samples")
    return selected


def split_step_balance(row):
    """Generalized constraint impulse/dt is a torque, not a motor command."""
    dt = float(row["dt_s"])
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("Positive finite DART timestep required")
    keys = (
        "fd_force_nm",
        "implicit_damping_nm",
        "constraint_impulse_nm_s",
        "transmitted_ty_nm",
    )
    if any(not math.isfinite(float(row[key])) for key in keys):
        raise ValueError("Non-finite DART force data")
    constraint = float(row["constraint_impulse_nm_s"]) / dt
    predicted = (
        float(row["fd_force_nm"]) + float(row["implicit_damping_nm"]) + constraint
    )
    return {
        "constraint_torque_nm": constraint,
        "predicted_transmitted_nm": predicted,
        "closure_error_nm": predicted - float(row["transmitted_ty_nm"]),
    }


def audit_joint(physics, dart, controller, index):
    """Compare separate steady windows, accepting only matching stationary poses."""
    if min(len(physics), len(dart), len(controller)) < 20:
        raise ValueError("Insufficient steady audit samples")
    mean = lambda rows, key: sum(float(r[key]) for r in rows) / len(rows)
    command = sum(r["command"][index] for r in controller) / len(controller)
    controller_q = sum(r["position"][index] for r in controller) / len(controller)
    balances = [split_step_balance(r) for r in dart]
    max_speed = max(abs(float(r["v_rad_s"])) for r in [*physics, *dart])
    q_difference = max(
        abs(controller_q - mean(physics, "q_rad")),
        abs(controller_q - mean(dart, "q_rad")),
    )
    if max_speed > 0.001 or q_difference > 1e-6:
        raise ValueError("Audit windows do not share a stationary posture")
    closure = max(abs(b["closure_error_nm"]) for b in balances)
    input_difference = max(
        abs(command - mean(physics, "input_mean_nm")),
        abs(command - mean(dart, "fd_command_nm")),
        abs(command - mean(dart, "fd_force_nm")),
    )
    input_spread = max(
        abs(float(r["input_max_nm"]) - float(r["input_min_nm"])) for r in physics
    )
    modes = sorted({int(r["actuator_type"]) for r in dart})
    friction_ok = all(
        abs(b["constraint_torque_nm"]) <= float(r["friction_nm"]) + 1e-6
        for b, r in zip(balances, dart)
    )
    damping_residual = max(
        abs(
            float(r["implicit_damping_nm"])
            + float(r["damping_nm_s_rad"])
            * (
                float(r["fd_velocity_rad_s"])
                + float(r["dt_s"]) * float(r["fd_acceleration_rad_s2"])
            )
        )
        for r in dart
    )
    limit_margin = min(
        min(
            float(r["q_rad"]) - float(r["lower_rad"]),
            float(r["upper_rad"]) - float(r["q_rad"]),
        )
        for r in physics
    )
    return {
        "ros_command_nm": command,
        "gazebo_input_mean_nm": mean(physics, "input_mean_nm"),
        "dart_input_command_nm": mean(dart, "fd_command_nm"),
        "dart_force_before_constraints_nm": mean(dart, "fd_force_nm"),
        "implicit_damping_nm": mean(dart, "implicit_damping_nm"),
        "joint_constraint_torque_nm": sum(b["constraint_torque_nm"] for b in balances)
        / len(balances),
        "transmitted_joint_moment_nm": mean(dart, "transmitted_ty_nm"),
        "gazebo_transmitted_world_y_nm": mean(physics, "ty_world"),
        "max_split_step_closure_error_nm": closure,
        "max_input_command_difference_nm": input_difference,
        "max_input_spread_within_block_nm": input_spread,
        "max_damping_formula_error_nm": damping_residual,
        "max_final_speed_rad_s": max_speed,
        "mean_predicted_free_velocity_rad_s": mean(dart, "fd_velocity_rad_s")
        + mean(dart, "dt_s") * mean(dart, "fd_acceleration_rad_s2"),
        "actuator_modes": modes,
        "force_mode_confirmed": modes == [0],
        "constraint_within_coulomb_bound": friction_ok,
        "joint_limit_margin_rad": limit_margin,
        "external_force_final_n": max(
            abs(float(r["external_fx_last_n"])) for r in physics
        ),
        "post_step_force_buffer_mean_nm": mean(dart, "force_nm"),
        "passed": closure < 1e-6
        and input_difference < 1e-5
        and input_spread < 1e-5
        and damping_residual < 1e-8
        and modes == [0]
        and friction_ok
        and limit_margin > 0.01,
    }
