"""Summarize resolution changes versus repeat variation, without claiming an error bound."""

import math
import statistics

PITCH_TOLERANCE_DEG = 0.01
JOINT_TOLERANCE_RAD = 0.0002


def summarize_resolution_study(trials):
    steps = sorted({r["physics_step_s"] for r in trials}, reverse=True)
    if steps != [0.001, 0.0005, 0.00025]:
        raise ValueError("Resolution screen requires 1, 0.5 and 0.25 ms")
    identities = [(r["physics_step_s"], r["repeat"]) for r in trials]
    if len(set(identities)) != len(identities):
        raise ValueError("Duplicate repeat identity")
    groups = []
    for step in steps:
        rows = [r for r in trials if r["physics_step_s"] == step]
        if len(rows) < 2:
            raise ValueError(
                "At least two independent trials per resolution are required"
            )
        values = {
            key: [r[key] for r in rows]
            for key in (
                "final_pitch_change_deg",
                "final_joint_change_rad",
                "physics_input_impulse_n_s",
                "ankle_implicit_damping_nm",
            )
        }
        if any(not math.isfinite(x) for v in values.values() for x in v):
            raise ValueError("Non-finite comparison measurement")
        groups.append(
            {
                "physics_step_s": step,
                "n": len(rows),
                **{
                    key: {
                        "mean": statistics.mean(v),
                        "min": min(v),
                        "max": max(v),
                        "range": max(v) - min(v),
                    }
                    for key, v in values.items()
                },
                "recoveries_passed": sum(r["recovery_passed"] for r in rows),
                "all_physics_audits_passed": all(
                    r["physics_audit_passed"] for r in rows
                ),
            }
        )
    changes = []
    for coarse, fine in zip(groups, groups[1:]):
        changes.append(
            {
                "from_step_s": coarse["physics_step_s"],
                "to_step_s": fine["physics_step_s"],
                "pitch_mean_change_deg": abs(
                    fine["final_pitch_change_deg"]["mean"]
                    - coarse["final_pitch_change_deg"]["mean"]
                ),
                "joint_mean_change_rad": abs(
                    fine["final_joint_change_rad"]["mean"]
                    - coarse["final_joint_change_rad"]["mean"]
                ),
            }
        )
    last = changes[-1]
    fine_repeat_ok = all(
        g["final_pitch_change_deg"]["range"] <= PITCH_TOLERANCE_DEG
        and g["final_joint_change_rad"]["range"] <= JOINT_TOLERANCE_RAD
        for g in groups[-2:]
    )
    impulses = [r["physics_input_impulse_n_s"] for r in trials]
    if min(impulses) <= 0:
        raise ValueError("Each push must have positive observed input impulse")
    spread = (max(impulses) - min(impulses)) / statistics.mean(impulses)
    audit_ok = all(g["all_physics_audits_passed"] for g in groups)
    change_ok = (
        last["pitch_mean_change_deg"] <= PITCH_TOLERANCE_DEG
        and last["joint_mean_change_rad"] <= JOINT_TOLERANCE_RAD
    )
    return {
        "groups": groups,
        "refinement_changes": changes,
        "screen_tolerances": {
            "pitch_deg": PITCH_TOLERANCE_DEG,
            "joint_rad": JOINT_TOLERANCE_RAD,
        },
        "fine_repeat_variation_within_tolerance": fine_repeat_ok,
        "finest_resolution_change_within_tolerance": change_ok,
        "input_impulse_relative_spread": spread,
        "input_impulses_within_five_percent": spread <= 0.05,
        "all_physics_audits_passed": audit_ok,
        "resolution_screen_passed": change_ok
        and fine_repeat_ok
        and spread <= 0.05
        and audit_ok,
        "limitation": "Two pushes per resolution provide ranges, not confidence intervals or a formal convergence proof.",
    }
