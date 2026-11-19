#!/usr/bin/env python3
"""Save a P54 2 mm foot-lift feasibility preview. Never starts Gazebo or ROS nodes."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import hashlib
import json
import os

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))


def snapshot():
    return {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted((ROOT / "src/my_robot_description").rglob("*"))
        if p.is_file()
        and p.suffix in (".py", ".xacro", ".yaml", ".cpp", ".sdf", ".config")
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clearance-mm", type=int, choices=[2, 10, 30], default=2)
    parser.add_argument("--support", choices=["left", "right"], default="left")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    os.environ["BIPED_ACTUATOR_PROFILE"] = "robotis_p54"
    import xacro
    from my_robot_description.motion.foot_lift import (
        FootLiftPreview,
        gravity_gradient_check,
    )
    from my_robot_description.model.roll_stance import JOINT_NAMES, MOTOR_CAPS

    before = snapshot()
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    plan = FootLiftPreview(xml, args.support, args.clearance_mm / 1000)
    checks = [
        gravity_gradient_check(xml, plan.table[i]["target_rad"], args.support)
        for i in (0, plan.intervals // 2, plan.intervals)
    ]
    samples = [
        plan.sample(i / 10) for i in range(round(plan.setup["total_time_s"] * 10) + 1)
    ]
    speed = max(
        abs(b["target_rad"][j] - a["target_rad"][j]) / (b["elapsed_s"] - a["elapsed_s"])
        for a, b in zip(samples, samples[1:])
        for j in range(10)
    )
    static_peak = max(r["peak_static_motor_utilization"] for r in plan.table)
    margin = min(r["support_margin_m"] for r in plan.table)
    corner_error = max(
        abs(c[k] - (r["requested_clearance_m"] if s == plan.swing and k == 2 else 0.0))
        for r in plan.table
        for s, points in r["sole_corner_displacements_m"].items()
        for c in points
        for k in range(3)
    )
    unchanged = before == snapshot()
    passed = (
        all(c["passed"] for c in checks)
        and unchanged
        and speed < 0.05
        and corner_error < 1e-9
    )
    result = dict(
        completed=True,
        passed=passed,
        preview_kind="foot_lift_feasibility",
        simulated=False,
        motors_commanded=False,
        foot_lift_measured=False,
        supporting_foot=args.support,
        swing_foot=plan.swing,
        requested_clearance_m=plan.clearance_m,
        predicted_clearance_m=plan.table[-1]["requested_clearance_m"],
        minimum_predicted_support_margin_m=margin,
        peak_static_motor_utilization=static_peak,
        sampled_peak_reference_speed_rad_s=speed,
        maximum_sole_corner_target_error_m=corner_error,
        virtual_work_checks=checks,
        source_unchanged=unchanged,
        plan=plan.setup,
        scope="Offline feasibility passed does not validate a physical foot lift or dynamic balance.",
    )
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    (folder / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
    (folder / "preview.json").write_text(
        json.dumps(dict(table=plan.table, samples=samples), indent=2) + "\n"
    )
    (folder / "source-hashes.json").write_text(json.dumps(before, indent=2) + "\n")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 9))
    axes[0].plot(
        [r["elapsed_s"] for r in samples],
        [r["requested_clearance_m"] * 1000 for r in samples],
    )
    axes[0].set(
        xlabel="Preview time (s), from zero-load endpoint",
        ylabel="Planned swing clearance (mm)",
    )
    axes[0].axvspan(
        plan.move_time_s,
        plan.move_time_s + plan.hold_time_s,
        alpha=0.12,
        color="green",
        label="Proposed lifted hold",
    )
    axes[0].legend()
    for i, name in enumerate(JOINT_NAMES):
        axes[1].plot(
            [r["requested_clearance_m"] * 1000 for r in plan.table],
            [100 * abs(r["feedforward_nm"][i]) / MOTOR_CAPS[i] for r in plan.table],
            label=name,
        )
    axes[1].axhline(80, color="red", ls="--", label="Existing static planning guard")
    axes[1].set(
        xlabel="Planned swing clearance (mm)", ylabel="Predicted motor utilization (%)"
    )
    axes[1].legend(fontsize=7, ncol=3)
    axes[2].plot(
        [r["requested_clearance_m"] * 1000 for r in plan.table],
        [r["support_margin_m"] * 1000 for r in plan.table],
    )
    axes[2].axhline(
        10, color="red", ls="--", label="Existing geometric margin requirement"
    )
    axes[2].set(
        xlabel="Planned swing clearance (mm)", ylabel="Predicted COM sole margin (mm)"
    )
    axes[2].legend()
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.suptitle(
        f"P54 first-lift OFFLINE preview — {args.support} support\n"
        "Kinematic/static predictions only; no simulation or measured foot lift"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(folder / "response.png", dpi=140)
    plt.close(fig)
    print(
        json.dumps(
            {
                k: v
                for k, v in result.items()
                if k not in ("plan", "virtual_work_checks")
            },
            indent=2,
        )
    )
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
