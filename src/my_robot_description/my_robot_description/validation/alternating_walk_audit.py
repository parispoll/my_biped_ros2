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


def audit_alternating_walk(folder, controller, com, program):
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
    height_error = 0.0
    xy_error = 0.0
    evidence = {
        step["index"]: dict(held=[], landing=[], final=[], margins=[])
        for step in program.steps
    }
    for pair in pairs:
        i = bisect_right(times, pair["sim_time"]) - 1
        if i < 0:
            raise ValueError("Missing walking geometry before contact stamp")
        r = obs[i]
        t = pair["sim_time"] - start
        p = program.sample(t)
        geometry_ok &= program.guard(
            {s: pair[s] for s in ("left", "right")}, pair["sim_time"], t, r
        )["eligible"]
        for side in ("left", "right"):
            corners = r["feet"][side]
            goal = p[side + "_foot_xyz_m"]
            height_error = max(
                height_error, *(abs(c[2] - (goal[2] - 0.045)) for c in corners)
            )
            xy_error = max(
                xy_error,
                *(abs(sum(c[k] for c in corners) / 4 - goal[k]) for k in (0, 1))
            )
        step = program.steps[p["step_index"] - 1]
        e = evidence[step["index"]]
        if step["advance_start"] + 0.5 <= t < step["lower_start"] - 0.5:
            e["held"].append((pair, r))
            corners = r["feet"][step["support"]]
            e["margins"].append(
                min(
                    *[r["com"][k] - min(c[k] for c in corners) for k in (0, 1)],
                    *[max(c[k] for c in corners) - r["com"][k] for k in (0, 1)]
                )
            )
        if step["reload_start"] - 1 <= t < step["reload_start"]:
            e["landing"].append(pair)
        if step["end"] - 3 <= t <= step["end"]:
            e["final"].append((pair, r))
    joints = audit_joint_feedback(rows, program, start)
    body = audit_body_measurements(folder, program, start, end)
    center = lambda r, s, k: sum(c[k] for c in r["feet"][s]) / 4
    step_results = []
    for step in program.steps:
        e = evidence[step["index"]]
        held = e["held"]
        final = e["final"]
        swing = step["swing"]
        support = step["support"]
        advance = (
            sum(center(r, swing, 0) - step["start_feet"][swing][0] for _, r in final)
            / len(final)
            if final
            else None
        )
        share = (
            sum(p[swing]["force_world_n"][2] / p["total_vertical_n"] for p, _ in final)
            / len(final)
            if final
            else None
        )
        clear = (
            min(min(c[2] for c in r["feet"][swing]) for _, r in held) if held else None
        )
        raised = len(held) >= 20 and all(
            p[swing]["state"] == "no_contact" for p, _ in held
        )
        landing = len(e["landing"]) >= 20 and all(
            p[swing]["state"] in ("touching_unloaded", "loaded_contact")
            for p in e["landing"]
        )
        expected = step["end_feet"][swing][0] - step["start_feet"][swing][0]
        final_ok = (
            len(final) >= 20
            and abs(advance - expected) <= 0.001
            and abs(share - 0.5) <= 0.02
            and all(
                p[s]["state"] == "loaded_contact"
                for p, _ in final
                for s in ("left", "right")
            )
        )
        margin = min(e["margins"]) if e["margins"] else None
        step_results.append(
            dict(
                index=step["index"],
                supporting_foot=support,
                swing_foot=swing,
                passed=bool(
                    raised
                    and landing
                    and final_ok
                    and clear >= 0.0295
                    and margin >= 0.01
                ),
                requested_foot_travel_m=expected,
                measured_foot_travel_m=advance,
                held_minimum_clearance_m=clear,
                held_fresh_no_contact=raised,
                landing_contact_confirmed=landing,
                final_landed_foot_load_fraction=share,
                minimum_support_com_margin_m=margin,
                held_samples=len(held),
                final_samples=len(final),
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
            and joints.peak_speed_rad_s
            <= program.setup.get("joint_speed_audit_limit_rad_s", 0.05)
            and joints.max_tracking_error_rad <= 0.01
        ),
        contacts_geometry=bool(coverage(pairs, start, end, 0.05) and geometry_ok),
        sensors=bool(body.passed),
        com=bool(com_check["passed"]),
        physical_damping=bool(damping["passed"]),
        alternating_steps=all(s["passed"] for s in step_results),
    )
    tail = [r for r in obs if end - 3 <= r["sim_time"] <= end]
    initial = [r for r in obs if start + 1 <= r["sim_time"] < start + 2.5]
    advance_com = sum(r["com"][0] for r in tail) / len(tail) - sum(
        r["com"][0] for r in initial
    ) / len(initial)
    return dict(
        passed=all(checks.values()),
        checks=checks,
        outcome=(
            "completed_alternating_steps"
            if all(checks.values())
            else "completed_walking_checks_not_met"
        ),
        alternating_walk=dict(
            step_count=program.setup["step_count"],
            verified_steps=sum(s["passed"] for s in step_results),
            steps=step_results,
            measured_com_forward_progress_m=advance_com,
            maximum_sole_height_tracking_error_m=height_error,
            maximum_foot_xy_tracking_error_m=xy_error,
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
            note="Finite measured alternating gait. No continuous indefinite walking or disturbance-recovery claim.",
        ),
        com_coverage=com_check,
        physical_damping=damping,
        foot_forces_measured=True,
    )
