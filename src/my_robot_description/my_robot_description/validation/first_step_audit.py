"""Independent raw-contact, sole, pose and motor replay for a forward step."""

import csv
import json
from bisect import bisect_right
from my_robot_description.motion.zero_load import paired_contact_states
from my_robot_description.model.roll_stance import MOTOR_CAPS
from my_robot_description.validation.roll_audit import audit_damping
from my_robot_description.validation.com_audit import audit_com
from my_robot_description.validation.motion_audit import (
    coverage,
    audit_joint_feedback,
    audit_body_measurements,
)


def audit_first_step(folder, controller, com, program):
    read = lambda name: [
        json.loads(s) for s in (folder / name).read_text().splitlines()
    ]
    active = [r for r in controller if r.get("shift") is not None]
    start = active[0]["sim_time"] - active[0]["shift"]["elapsed_s"]
    end = start + program.setup["total_time_s"]
    rows = [r for r in active if start <= r["sim_time"] <= end]
    obs = [r for r in com if start - 0.1 <= r["sim_time"] <= end]
    pairs = paired_contact_states(read("contact-wrenches.jsonl"), start, end)
    times = [r["sim_time"] for r in obs]
    geometry_ok = True
    held = []
    landing = []
    final = []
    margins = []
    height_error = 0.0
    xy_error = 0.0
    for pair in pairs:
        i = bisect_right(times, pair["sim_time"]) - 1
        if i < 0:
            raise ValueError("Missing first-step geometry before contact stamp")
        r = obs[i]
        t = pair["sim_time"] - start
        p = program.sample(t)
        geometry_ok &= program.guard(
            {s: pair[s] for s in ("left", "right")}, pair["sim_time"], t, r
        )["eligible"]
        for side in ("left", "right"):
            corners = r["feet"][side]
            h = p["swing_clearance_m"] if side == program.swing else 0.0
            height_error = max(height_error, *(abs(c[2] - h) for c in corners))
            for k in (0, 1):
                expected = program.feet[side][k] + (
                    p["swing_advance_m"] if side == program.swing and k == 0 else 0.0
                )
                xy_error = max(xy_error, abs(sum(c[k] for c in corners) / 4 - expected))
        if 132.5 <= t < 151.5:
            held.append((pair, r))
            corners = r["feet"][program.support]
            margins.append(
                min(
                    *[r["com"][k] - min(c[k] for c in corners) for k in (0, 1)],
                    *[max(c[k] for c in corners) - r["com"][k] for k in (0, 1)]
                )
            )
        if 168.0 <= t < 169.0:
            landing.append(pair)
        if 238.0 <= t <= 241.0:
            final.append((pair, r))
    joints = audit_joint_feedback(rows, program, start)
    body = audit_body_measurements(folder, program, start, end)
    center = lambda r, s, k: sum(c[k] for c in r["feet"][s]) / 4
    final_advance = (
        sum(
            center(r, program.swing, 0) - program.feet[program.swing][0]
            for _, r in final
        )
        / len(final)
        if final
        else None
    )
    final_share = (
        sum(
            p[program.swing]["force_world_n"][2] / p["total_vertical_n"]
            for p, _ in final
        )
        / len(final)
        if final
        else None
    )
    landed = bool(landing) and all(
        p[program.swing]["state"] in ("touching_unloaded", "loaded_contact")
        for p in landing
    )
    raised = len(held) >= 20 and all(
        p[program.swing]["state"] == "no_contact" for p, _ in held
    )
    min_height = (
        min(min(c[2] for c in r["feet"][program.swing]) for _, r in held)
        if held
        else None
    )
    final_ok = (
        len(final) >= 20
        and abs(final_advance - 0.05) <= 0.001
        and abs(final_share - 0.5) <= 0.02
        and all(
            p[s]["state"] == "loaded_contact"
            for p, _ in final
            for s in ("left", "right")
        )
    )
    com_check = audit_com(com, start, end)
    com_check["passed"] &= (
        abs(com_check.get("mass_kg", 0) - program.planner.base["mass_kg"]) < 1e-9
    )
    with (folder / "viscous.csv").open() as f:
        damping = audit_damping(list(csv.DictReader(f)))
    checks = dict(
        controller=bool(
            coverage(rows, start, end, 0.1)
            and joints.live_ok
            and joints.motor_ok
            and joints.composition_error_nm < 1e-9
            and joints.peak_speed_rad_s <= 0.05
            and joints.max_tracking_error_rad <= 0.01
        ),
        contacts_geometry=bool(coverage(pairs, start, end, 0.05) and geometry_ok),
        sensors=bool(body.passed),
        com=bool(com_check["passed"]),
        physical_damping=bool(damping["passed"]),
        forward_step=bool(
            raised
            and landed
            and final_ok
            and min_height >= 0.0295
            and min(margins) >= 0.01
        ),
    )
    return dict(
        passed=all(checks.values()),
        checks=checks,
        outcome=(
            "completed_one_forward_step_and_staggered_reload"
            if all(checks.values())
            else "completed_step_checks_not_met"
        ),
        first_step=dict(
            supporting_foot=program.support,
            swing_foot=program.swing,
            requested_advance_m=0.05,
            measured_final_advance_m=final_advance,
            held_minimum_clearance_m=min_height,
            held_fresh_no_contact=raised,
            landing_contact_confirmed=landed,
            final_landed_foot_load_fraction=final_share,
            minimum_swing_support_com_margin_m=min(margins) if margins else None,
            maximum_sole_height_tracking_error_m=height_error,
            maximum_foot_xy_tracking_error_m=xy_error,
            final_samples=len(final),
            maximum_pose_tracking_error_m=body.max_pose_error_m,
            peak_joint_speed_rad_s=joints.peak_speed_rad_s,
            max_joint_tracking_error_rad=joints.max_tracking_error_rad,
            peak_motor_utilization=max(
                abs(t) / c for r in rows for t, c in zip(r["command"], MOTOR_CAPS)
            ),
            motor_composition_error_nm=joints.composition_error_nm,
            saturated_samples=sum(
                any(abs(a - b) > 1e-9 for a, b in zip(r["requested"], r["command"]))
                for r in rows
            ),
            origin_sim_time=start,
            end_sim_time=end,
            note="One forward foot placement and measured 50/50 load in a new staggered stance; no alternating gait or disturbance-recovery claim.",
        ),
        com_coverage=com_check,
        physical_damping=damping,
        foot_forces_measured=True,
    )
