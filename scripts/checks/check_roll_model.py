#!/usr/bin/env python3
"""Expand/check the ten-joint prototype and draw a front-view geometry preview."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import xacro
from scripts.planning.roll_model_geometry import (
    validate_roll_model,
    link_transforms,
    model_com,
)

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    report = validate_roll_model(xml)
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "biped_roll.urdf").write_text(xml)
    poses = [
        {},
        dict(
            left_hip_roll=0.1,
            left_ankle_roll=-0.1,
            right_hip_roll=-0.1,
            right_ankle_roll=0.1,
        ),
    ]
    report.update(
        geometry_passed=True,
        standing_verified=False,
        single_leg_balance_verified=False,
        preview="Pelvis fixed in FK only. Poses are not executed, planted-foot targets or balanced stances.",
        roll_parameters=dict(
            carrier_mass_each_kg=0.08,
            carrier_box_m=[0.05, 0.05, 0.04],
            angle_limit_rad=0.35,
            urdf_effort_limit_nm=6,
            urdf_velocity_limit_rad_s=2,
            passive_damping=0.2,
            validated_motor_rating=False,
        ),
    )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    fig, axes = plt.subplots(1, 2, figsize=(10, 6))
    for ax, q, title in zip(
        axes,
        poses,
        [
            "Neutral: original leg geometry",
            "Roll geometry: opposite 0.10 rad hip rolls",
        ],
    ):
        t = link_transforms(xml, q)
        mass, com = model_com(xml, q)
        ax.add_patch(Rectangle((-0.11, -0.08), 0.22, 0.16, color="#cc6a56", alpha=0.4))
        for side, color in [("left", "#167d9a"), ("right", "#c77a22")]:
            chain = [
                f"{side}_hip_roll_carrier",
                f"{side}_thigh",
                f"{side}_shin",
                f"{side}_ankle_pitch_carrier",
                f"{side}_foot",
            ]
            ax.plot(
                [t[n][1][3] for n in chain],
                [t[n][2][3] for n in chain],
                "-o",
                color=color,
                label=side,
            )
            foot = t[f"{side}_foot"]
            fy, fz = foot[1][3], foot[2][3]
            # Both displayed poses have zero resulting foot rotation.
            ax.add_patch(
                Rectangle(
                    (fy - 0.06, fz - 0.045), 0.12, 0.045, facecolor=color, alpha=0.4
                )
            )
        ax.scatter(
            [com[1]],
            [com[2]],
            color="#159ca2",
            marker="x",
            s=80,
            label="Calculated COM",
        )
        ax.set(
            title=title,
            xlabel="Sideways Y (m)",
            ylabel="Vertical Z relative to pelvis (m)",
            xlim=(-0.24, 0.24),
            ylim=(-0.61, 0.13),
            aspect="equal",
        )
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
    fig.suptitle(
        "Ten-joint prototype · geometry only\nHip roll allows lateral motion; opposite ankle roll keeps soles level in these poses."
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(folder / "front-view.png", dpi=150)
    plt.close(fig)
    (folder / "model-check.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
