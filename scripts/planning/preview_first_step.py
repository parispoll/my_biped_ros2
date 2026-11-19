#!/usr/bin/env python3
"""Offline first-step geometry/static/speed preflight; no motor commands."""

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
    a = p.parse_args()
    os.environ["BIPED_ACTUATOR_PROFILE"] = "robotis_p54"
    import xacro
    from my_robot_description.motion.first_step import FirstStepProgram
    from my_robot_description.motion.foot_lift import gravity_gradient_check
    from scripts.studies.run_duration_comparison import source_snapshot

    folder = a.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    snapshot = source_snapshot()
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    program = FirstStepProgram(xml, a.support)
    gradients = [
        gravity_gradient_check(xml, program.paths[name][i]["target_rad"], a.support)
        for name in ("lift", "advance", "lower_foot")
        for i in (0, 100, 200)
    ]
    if not all(g["passed"] for g in gradients):
        raise ValueError("Independent single-support gravity check failed")
    trajectory = [program.sample(float(t)) for t in np.linspace(0, 241, 1206)]
    report = dict(
        completed=True,
        passed=source_snapshot() == snapshot,
        preview_kind="first_step",
        supporting_foot=a.support,
        step_advance_m=0.05,
        clearance_m=0.03,
        motion_plan=program.setup,
        gravity_checks=gradients,
        source_unchanged=source_snapshot() == snapshot,
        scope="Offline preflight only; no executed step or measured load transfer.",
    )
    (folder / "trajectory.json").write_text(json.dumps(trajectory))
    (folder / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    from scripts.plots.plot_first_step import plot

    plot(folder)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
