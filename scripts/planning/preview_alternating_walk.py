#!/usr/bin/env python3
"""Offline full alternating-stance preflight, without simulation or commands."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse, json, os
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--support", choices=["left", "right"], default="left")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument(
        "--walk-transfer-scale",
        type=float,
        choices=[0.0, 0.25, 0.375, 0.5, 1.0],
        default=0.0,
    )
    p.add_argument("--walk-velocity-feedforward", action="store_true")
    p.add_argument("--walk-steps", type=int, default=3)
    p.add_argument("--walk-step-mm", type=int, choices=[50, 75, 100], default=50)
    p.add_argument(
        "--walk-time-scale", type=float, choices=[1.0, 0.5, 0.4, 0.25], default=1.0
    )
    a = p.parse_args()
    os.environ["BIPED_ACTUATOR_PROFILE"] = "robotis_p54"
    import xacro
    from my_robot_description.motion.alternating_walk import AlternatingWalkProgram
    from my_robot_description.motion.foot_lift import gravity_gradient_check
    from scripts.studies.run_duration_comparison import source_snapshot

    folder = a.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    snapshot = source_snapshot()
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    from my_robot_description.motion.profiled_walk import ProfiledWalkProgram
    from my_robot_description.motion.walking_profile import WalkingProfile

    program = ProfiledWalkProgram(
        xml,
        a.support,
        WalkingProfile(
            a.walk_steps,
            a.walk_step_mm / 1000,
            a.walk_time_scale,
            a.walk_transfer_scale,
            a.walk_velocity_feedforward,
        ),
    )
    gradients = [
        gravity_gradient_check(xml, s["table"][i]["target_rad"], s["support"])
        for s in program.segments
        if s["phase"] == "advance"
        for i in (0, 100, 200)
    ]
    if not all(g["passed"] for g in gradients):
        raise ValueError("Independent walking gravity gradient failed")
    rows = [
        program.sample(float(t))
        for t in np.linspace(0, program.total, round(program.total * 5) + 1)
    ]
    report = dict(
        completed=True,
        passed=source_snapshot() == snapshot,
        preview_kind="alternating_walk",
        supporting_foot=a.support,
        step_count=program.setup["step_count"],
        motion_plan=program.setup,
        gravity_checks=gradients,
        source_unchanged=source_snapshot() == snapshot,
        scope="Offline geometry/static/speed preflight only. No executed alternating gait.",
    )
    (folder / "trajectory.json").write_text(json.dumps(rows))
    (folder / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    from scripts.plots.plot_alternating_walk import plot

    plot(folder)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
