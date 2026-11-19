#!/usr/bin/env python3
"""Derive a fixed-foot partial unloading target; no simulation or motor commands."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))
import xacro
from my_robot_description.motion.partial_unload import PartialUnloadProgram
from my_robot_description.model.roll_geometry import link_transforms, model_com
from my_robot_description.model.roll_stance import JOINT_NAMES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fraction", type=float, choices=[0.4, 0.35, 0.3], default=0.35
    )
    parser.add_argument("--direction", choices=["left", "right"], default="left")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    program = PartialUnloadProgram(xml, args.fraction, args.direction)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "plan.json").write_text(json.dumps(program.setup, indent=2) + "\n")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, axes = plt.subplots(1, 2, figsize=(10, 6))
    for ax, point, title in zip(
        axes,
        [program.table[0], program.table[-1]],
        ["Centered standing", "Planned partial unloading"],
    ):
        q = dict(zip(JOINT_NAMES, point["target_rad"]))
        t = link_transforms(xml, q)
        height = program.planner.base["expected_pelvis_height_m"]
        d = point["pelvis_y_m"]
        ax.add_patch(
            Rectangle((d - 0.11, height - 0.08), 0.22, 0.16, color="#cc6a56", alpha=0.4)
        )
        for side, c in [("left", "#167d9a"), ("right", "#c77a22")]:
            names = [
                f"{side}_{n}"
                for n in [
                    "hip_roll_carrier",
                    "thigh",
                    "shin",
                    "ankle_pitch_carrier",
                    "foot",
                ]
            ]
            ax.plot(
                [t[n][1][3] + d for n in names],
                [t[n][2][3] + height for n in names],
                "-o",
                color=c,
                label=side,
            )
            foot = t[f"{side}_foot"]
            y = foot[1][3] + d
            z = foot[2][3] + height
            ax.add_patch(
                Rectangle((y - 0.06, z - 0.045), 0.12, 0.045, facecolor=c, alpha=0.4)
            )
        _, com = model_com(xml, q)
        com[1] += d
        com[2] += height
        ax.plot([com[1], com[1]], [0, com[2]], "--", color="#555", alpha=0.6)
        ax.scatter(
            [com[1]], [com[2]], color="#222", marker="x", s=70, label="Planned COM"
        )
        ax.axhline(0, color="#777", ls=":")
        ax.set(
            title=title,
            xlabel="World Y (m), positive toward robot left",
            ylabel="World Z (m)",
            xlim=(-0.23, 0.25),
            ylim=(-0.02, 0.66),
            aspect="equal",
        )
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
    fig.suptitle(
        f'Geometry preview only · {100*(1-args.fraction):.0f}/{100*args.fraction:.0f} modeled load split\nBoth soles fixed; peak planned static motor use {100*program.setup["static_motor_utilization"]:.1f}% of cap'
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(args.output / "geometry.png", dpi=150)
    plt.close(fig)
    print(
        json.dumps(
            {
                k: program.setup[k]
                for k in [
                    "displacement_m",
                    "target_unloaded_fraction",
                    "unloaded_foot",
                    "static_motor_utilization",
                ]
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
