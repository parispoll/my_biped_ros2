"""Audit the small bilateral COM-shift experiment without claiming foot forces."""

import math
from my_robot_description.motion.lateral_shift import ShiftProgram, TOTAL_TIME
from my_robot_description.model.roll_stance import MOTOR_CAPS
from my_robot_description.control.roll_posture import KP, KD, INTEGRAL_CAPS, LATERAL_KI
from my_robot_description.validation.com_audit import audit_com


def mean(values):
    if not values:
        raise ValueError("Missing phase samples")
    return sum(values) / len(values)


def audit_shift(
    controller,
    com,
    xml,
    displacement,
    program=None,
    contact_policy=None,
    foot_reference=None,
):
    program = program or ShiftProgram(xml, displacement)
    total_time = program.setup["total_time_s"]
    windows = program.setup.get(
        "analysis_windows", dict(hold=(11.0, 13.0), recovery=(22.0, 25.00001))
    )
    shifted = [r for r in controller if r.get("shift") is not None]
    if not shifted:
        raise ValueError("No shift program evidence")
    origin = shifted[0]["sim_time"] - shifted[0]["shift"]["elapsed_s"]
    end = origin + total_time
    rows = [r for r in shifted if origin <= r["sim_time"] <= end]
    residual = 0.0
    contact_ok = True
    motor_ok = True
    for r in rows:
        if abs(r["sim_time"] - origin - r["shift"]["elapsed_s"]) > 1e-9:
            raise ValueError("Shift program clock inconsistency")
        if r.get("integral_gains") != LATERAL_KI:
            raise ValueError("Unexpected shift integral gains")
        point = program.sample(r["shift"]["elapsed_s"])
        if program.setup.get("program_kind") == "load_feedback":
            state = r.get("load_feedback")
            if state is None:
                raise ValueError("Missing load-feedback command evidence")
            point = program.command_sample(
                r["shift"]["elapsed_s"], state["correction_m"]
            )
        if point["phase"] != r["shift"]["phase"]:
            raise ValueError("Recorded phase disagrees with time")
        if not r["active"] or not r["ready"] or r.get("integral_frozen_gap", False):
            raise ValueError("Inactive/unready/gapped shift feedback")
        contact_ok &= (
            contact_policy(r)
            if contact_policy
            else set(r["shift_contacts"]) == {"left", "right"}
            and all(
                c["ground"] and -0.02 <= c["age_s"] <= 0.05
                for c in r["shift_contacts"].values()
            )
        )
        derivative = (
            program.derivative_gains(r["shift"]["elapsed_s"])
            if hasattr(program, "derivative_gains")
            else KD
        )
        proportional = (
            program.proportional_gains(r["shift"]["elapsed_s"])
            if hasattr(program, "proportional_gains")
            else KP
        )
        if (
            hasattr(program, "derivative_gains")
            and r.get("derivative_gains") != derivative
        ):
            raise ValueError("Recorded zero-load damping schedule disagrees with plan")
        if (
            hasattr(program, "proportional_gains")
            and r.get("proportional_gains") != proportional
        ):
            raise ValueError(
                "Recorded zero-load stiffness schedule disagrees with plan"
            )
        for i, cap in enumerate(MOTOR_CAPS):
            goal = point["target_rad"][i]
            ff = point["feedforward_nm"][i]
            error = goal - r["position"][i]
            expected = (
                ff
                + proportional[i] * error
                - derivative[i] * r["velocity"][i]
                + r["integral"][i]
            )
            values = [
                expected,
                r["command"][i],
                r["requested"][i],
                r["integral"][i],
                r["target_rad"][i],
            ]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("Nonfinite shift motor evidence")
            residual = max(
                residual,
                abs(goal - r["target_rad"][i]),
                abs(ff - r["feedforward"][i]),
                abs(error - r["error"][i]),
                abs(expected - r["requested"][i]),
                abs(max(-cap, min(cap, expected)) - r["command"][i]),
            )
            motor_ok &= (
                abs(r["command"][i]) <= cap + 1e-9
                and abs(r["integral"][i]) <= INTEGRAL_CAPS[i] + 1e-9
            )
    times = [r["sim_time"] for r in rows]
    gaps = [b - a for a, b in zip(times, times[1:])]
    coverage = (
        len(rows) > 2
        and min(gaps) > 0
        and max(gaps) <= 0.1
        and times[0] - origin <= 0.1
        and end - times[-1] <= 0.1
    )
    observations = [r for r in com if origin <= r["sim_time"] <= end]
    com_audit = audit_com(com, origin, end)
    # Expected mass comes from independent URDF planning, never the observation.
    com_audit["coverage_passed"] = com_audit["passed"]
    com_audit["expected_mass_kg"] = program.planner.base["mass_kg"]
    com_audit["mass_matches_model"] = (
        abs(com_audit.get("mass_kg", 0) - com_audit["expected_mass_kg"]) < 1e-9
    )
    com_audit["passed"] &= com_audit["mass_matches_model"]

    def center(r, side, axis):
        return sum(c[axis] for c in r["feet"][side]) / 4

    def relative_y(r):
        return r["com"][1] - (center(r, "left", 1) + center(r, "right", 1)) / 2

    baseline = [r for r in observations if r["sim_time"] < origin + 2.5]
    hold = [
        r
        for r in observations
        if origin + windows["hold"][0] <= r["sim_time"] < origin + windows["hold"][1]
    ]
    returned = [
        r
        for r in observations
        if origin + windows["recovery"][0] <= r["sim_time"] <= end
    ]
    y0 = mean([relative_y(r) for r in baseline])
    hold_change = mean([relative_y(r) for r in hold]) - y0
    final_change = mean([relative_y(r) for r in returned]) - y0
    final_peak = max(abs(relative_y(r) - y0) for r in returned)
    predicted = program.setup["endpoint"]["predicted_com_y_m"]
    foot0 = {
        s: [mean([center(r, s, i) for r in baseline]) for i in range(3)]
        for s in ["left", "right"]
    }
    foot_drift = max(
        math.sqrt(
            sum(
                (
                    center(r, s, i)
                    - foot0[s][i]
                    - (
                        foot_reference(r["sim_time"] - origin, s)
                        if foot_reference and i == 2
                        else 0.0
                    )
                )
                ** 2
                for i in range(3)
            )
        )
        for r in observations
        for s in ["left", "right"]
    )
    sole_range = max(
        max(c[2] for c in r["feet"][s]) - min(c[2] for c in r["feet"][s])
        for r in observations
        for s in ["left", "right"]
    )
    ground_height = max(
        abs(
            c[2]
            - (foot_reference(r["sim_time"] - origin, s) if foot_reference else 0.0)
        )
        for r in observations
        for s, corners in r["feet"].items()
        for c in corners
    )
    max_error = max(abs(e) for r in rows for e in r["error"])
    max_speed = max(abs(v) for r in rows for v in r["velocity"])
    held = [
        r
        for r in rows
        if windows["hold"][0] <= r["shift"]["elapsed_s"] < windows["hold"][1]
        or windows["recovery"][0] <= r["shift"]["elapsed_s"] <= total_time
    ]
    held_error = max(abs(e) for r in held for e in r["error"])
    movement_ok = (
        hold_change * displacement > 0
        and abs(hold_change - predicted) <= 0.002
        and final_peak <= 0.001
    )
    if program.setup.get("program_kind") == "bilateral_lowering":
        movement_ok = abs(hold_change) <= 0.001 and final_peak <= 0.001
    passed = (
        coverage
        and contact_ok
        and motor_ok
        and residual < 1e-9
        and com_audit["passed"]
        and movement_ok
        and foot_drift <= 0.001
        and sole_range <= 0.003
        and ground_height <= 0.003
        and max_error <= 0.01
        and held_error <= 0.002
        and max_speed <= 0.05
    )
    return dict(
        passed=bool(passed),
        origin_sim_time=origin,
        end_sim_time=end,
        samples=len(rows),
        controller_coverage_passed=coverage,
        contacts_fresh_and_grounded=(
            bool(contact_ok)
            if contact_policy is None
            else all(
                all(c["ground"] for c in r["shift_contacts"].values()) for r in rows
            )
        ),
        contact_policy_passed=bool(contact_ok),
        motor_caps_passed=bool(motor_ok),
        max_motor_target_composition_error_nm=residual,
        max_tracking_error_rad=max_error,
        held_tracking_error_rad=held_error,
        peak_joint_speed_rad_s=max_speed,
        saturated_samples=sum(
            any(abs(a - b) > 1e-9 for a, b in zip(r["requested"], r["command"]))
            for r in rows
        ),
        requested_pelvis_shift_m=displacement,
        predicted_com_shift_m=predicted,
        measured_com_shift_relative_feet_m=hold_change,
        final_com_offset_relative_baseline_m=final_change,
        max_final_com_offset_m=final_peak,
        max_foot_center_drift_m=foot_drift,
        max_sole_height_range_m=sole_range,
        max_abs_sole_height_m=ground_height,
        com_coverage=com_audit,
        predicted_foot_forces_n=program.setup["endpoint"]["predicted_force_n"],
        foot_forces_measured=False,
        note="Measured lateral COM shift and planted-foot geometry; foot forces are static estimates, not measured load transfer or single-leg balance.",
    )


def audit_sensors(
    rows,
    imu,
    start,
    end,
    expected_height,
    displacement,
    windows=None,
    height_program=None,
    contact_policy=None,
):
    windows = windows or dict(hold=(11.0, 13.0), recovery=(22.0, 25.00001))
    """Validate the exact program window from already-running read-only capture."""
    rows = [r for r in rows if start <= r["sim_time"] <= end]
    raw = [r for r in imu if start <= r["sim_time"] <= end]
    if len(rows) < 2 or len(raw) < 2:
        raise ValueError("Missing lateral sensor capture")
    times = [r["sim_time"] for r in rows]
    gaps = [b - a for a, b in zip(times, times[1:])]
    imu_times = [r["sim_time"] for r in raw]
    imu_gaps = [b - a for a, b in zip(imu_times, imu_times[1:])]
    coverage = (
        0 < min(gaps)
        and max(gaps) <= 0.1
        and times[0] - start <= 0.1
        and end - times[-1] <= 0.1
    )
    imu_coverage = (
        0 < min(imu_gaps)
        and max(imu_gaps) <= 0.05
        and imu_times[0] - start <= 0.05
        and end - imu_times[-1] <= 0.05
    )
    max_age = {
        key: max(r["sim_time"] - r[key]["sim_time"] for r in rows)
        for key in ["pose", "imu"]
    }
    min_age = {
        key: min(r["sim_time"] - r[key]["sim_time"] for r in rows)
        for key in ["pose", "imu"]
    }
    fresh = all(
        min_age[k] >= -0.02 and max_age[k] <= (0.1 if k == "pose" else 0.05)
        for k in max_age
    )
    contacts_ok = (
        all(contact_policy(r) for r in rows)
        if contact_policy
        else all(
            set(r["contacts"]) == {"left", "right"}
            and all(
                c["ground"] and -0.02 <= r["sim_time"] - c["sim_time"] <= 0.05
                for c in r["contacts"].values()
            )
            for r in rows
        )
    )
    poses = [r["pose"] for r in rows]
    # A fresh pose received inside the window can originate just before it.
    # The reference before motion starts is the baseline, not negative time.
    reference_height = lambda p: (
        height_program.sample(max(0.0, p["sim_time"] - start))["pelvis_height_m"]
        if height_program is not None
        else expected_height
    )
    pose_ok = all(
        len(p["position"]) == 3
        and all(math.isfinite(v) for v in [*p["position"], p["tilt_radians"]])
        and abs(p["position"][2] - reference_height(p)) <= 0.005
        and p["tilt_radians"] <= 0.05
        for p in poses
    )
    # Retain the original small-motion conservative IMU limits; no relaxation.
    imu_ok = all(
        all(
            math.isfinite(v)
            for v in [r["tilt_radians"], r["angular_speed"], *r["acceleration"]]
        )
        and r["tilt_radians"] <= 0.05
        and r["angular_speed"] <= 0.05
        and max(
            abs(r["acceleration"][0]),
            abs(r["acceleration"][1]),
            abs(r["acceleration"][2] - 9.81),
        )
        <= 0.25
        for r in raw
    )
    baseline = [p for p in poses if p["sim_time"] < start + 2.5]
    hold = [
        p
        for p in poses
        if start + windows["hold"][0] <= p["sim_time"] < start + windows["hold"][1]
    ]
    pelvis0 = mean([p["position"][1] for p in baseline])
    measured_pelvis = mean([p["position"][1] for p in hold]) - pelvis0
    tail = [p for p in poses if start + windows["recovery"][0] <= p["sim_time"] <= end]
    final_pelvis_peak = max(abs(p["position"][1] - pelvis0) for p in tail)
    pelvis_ok = (
        abs(measured_pelvis - displacement) <= 0.002 and final_pelvis_peak <= 0.001
    )
    return dict(
        passed=bool(
            coverage
            and imu_coverage
            and fresh
            and contacts_ok
            and pose_ok
            and imu_ok
            and pelvis_ok
        ),
        samples=len(rows),
        raw_imu_samples=len(raw),
        coverage_passed=coverage,
        raw_imu_coverage_passed=imu_coverage,
        max_gap_s=max(gaps),
        max_raw_imu_gap_s=max(imu_gaps),
        max_source_age_s=max_age,
        contacts_fresh_and_grounded=(
            contacts_ok
            if contact_policy is None
            else all(all(c["ground"] for c in r["contacts"].values()) for r in rows)
        ),
        contact_policy_passed=contacts_ok,
        pose_passed=pose_ok,
        imu_passed=imu_ok,
        max_pose_tilt_rad=max(p["tilt_radians"] for p in poses),
        min_height_m=min(p["position"][2] for p in poses),
        max_height_m=max(p["position"][2] for p in poses),
        measured_pelvis_shift_m=measured_pelvis,
        max_final_pelvis_offset_m=final_pelvis_peak,
        pelvis_target_and_return_passed=pelvis_ok,
        max_imu_angular_speed_rad_s=max(r["angular_speed"] for r in raw),
        peak_acceleration_deviation_m_s2=max(
            max(
                abs(r["acceleration"][0]),
                abs(r["acceleration"][1]),
                abs(r["acceleration"][2] - 9.81),
            )
            for r in raw
        ),
        note="Independent Gazebo model pose, raw ROS contacts and full-rate received IMU; recorders start before warm-up. No force validation.",
    )
