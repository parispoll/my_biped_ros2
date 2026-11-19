#!/usr/bin/env python3
"""Visualize saved static lean-search evidence; no commands or simulation."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))
import numpy as np
import xacro
from my_robot_description.studies.support_feasibility import SupportSearch
from my_robot_description.model.roll_stance import JOINT_NAMES, MOTOR_CAPS


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("folder", type=Path)
    a.add_argument(
        "--upright", type=Path, default=ROOT / "evidence/support-posture-search-02"
    )
    args = a.parse_args()
    report = json.loads((args.folder / "search.json").read_text())
    old = json.loads((args.upright / "search.json").read_text())
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    s = SupportSearch(xml)
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon, Rectangle

    fig, axes = plt.subplots(2, 2, figsize=(13, 10))
    ax = axes[0, 0]
    f = sorted(float(k) for k in report["best"])
    for source, label, color in [
        (old, "Upright + pressure", "#777"),
        (report, "Lean + independent legs", "#167d9a"),
    ]:
        ax.plot(
            [100 * v for v in f],
            [100 * source["best"][str(v)]["peak_utilization"] for v in f],
            "-o",
            label=label,
            color=color,
        )
    ax.axhline(80, color="#777", ls="--", label="20% reserve")
    ax.axhline(100, color="#bd4338", ls=":", label="Motor cap")
    ax.set(
        title="Best sampled static poses",
        xlabel="Lighter-foot share (%)",
        ylabel="Peak torque / cap (%)",
    )
    ax.invert_xaxis()
    ax.legend(fontsize=8)
    ax.grid(alpha=0.2)
    path = report["partial_transfer"]
    r = path["selected_endpoint"]
    p = s.lean_pose(r["pelvis_shift_m"], r["pelvis_lowering_m"], r["pelvis_roll_rad"])
    ax = axes[0, 1]
    height = r["pelvis_height_m"]
    d = r["pelvis_shift_m"]
    phi = r["pelvis_roll_rad"]
    for side, color in [("left", "#167d9a"), ("right", "#c77a22")]:
        names = [
            side + "_" + n
            for n in [
                "hip_roll_carrier",
                "thigh",
                "shin",
                "ankle_pitch_carrier",
                "foot",
            ]
        ]
        ax.plot(
            [p["tf"][n][1][3] + d for n in names],
            [p["tf"][n][2][3] + height for n in names],
            "-o",
            color=color,
            label=side,
        )
        y = p["tf"][side + "_foot"][1][3] + d
        ax.add_patch(Rectangle((y - 0.06, 0), 0.12, 0.045, color=color, alpha=0.3))
        cop = y + r["pressure_offsets"][side]["y_offset_m"]
        force = r["forces_n"][side]
        ax.arrow(
            cop,
            0,
            0,
            0.12 * force / sum(r["forces_n"].values()),
            head_width=0.008,
            width=0.0015,
            color=color,
            length_includes_head=True,
        )
    size = [
        float(v)
        for v in s.links["base_link"].find("visual/geometry/box").get("size").split()
    ]
    corners = [
        (
            d + math.cos(phi) * y - math.sin(phi) * z,
            height + math.sin(phi) * y + math.cos(phi) * z,
        )
        for y, z in [
            (-size[1] / 2, -size[2] / 2),
            (size[1] / 2, -size[2] / 2),
            (size[1] / 2, size[2] / 2),
            (-size[1] / 2, size[2] / 2),
        ]
    ]
    ax.add_patch(Polygon(corners, color="#bd4338", alpha=0.4))
    com = r["com_world_m"]
    ax.scatter([com[1]], [com[2]], marker="x", s=70, c="black", label="COM")
    ax.plot([com[1], com[1]], [0, com[2]], "--", color="#555")
    ax.axhline(0, color="#777")
    ax.set(
        xlim=(-0.2, 0.27),
        ylim=(-0.02, 0.65),
        aspect="equal",
        xlabel="World Y (m)",
        ylabel="World Z (m)",
        title=f"75/25 transfer candidate · lean {math.degrees(abs(phi)):.1f}°\nFront projection of actual link frames",
    )
    ax.legend(fontsize=8)
    ax = axes[1, 0]
    names = [n.replace("left_", "L ").replace("right_", "R ") for n in JOINT_NAMES]
    index = np.arange(10)
    for offset, row, label, color in [
        (-0.2, r, "Selected 75/25", "#167d9a"),
        (0.2, report["best"]["0.0"], "Best sampled zero load", "#bd4338"),
    ]:
        ax.bar(
            index + offset,
            [
                100 * abs(row["torques_nm"][n]) / c
                for n, c in zip(JOINT_NAMES, MOTOR_CAPS)
            ],
            width=0.4,
            label=label,
            color=color,
        )
    ax.axhline(100, color="black", ls=":")
    ax.axhline(80, color="#777", ls="--")
    ax.set_xticks(index, names, rotation=60, ha="right", fontsize=8)
    ax.set(ylabel="Static torque / cap (%)", title="Which motors carry the demand?")
    ax.legend(fontsize=8)
    ax.grid(axis="y", alpha=0.2)
    ax = axes[1, 1]
    samples = path["samples"]
    u = [100 * p["progress"] for p in samples]
    for side, color in [("left", "#167d9a"), ("right", "#c77a22")]:
        ax.plot(
            u,
            [1000 * p["pressure_offsets"][side]["y_offset_m"] for p in samples],
            label=side + " lateral CoP",
            color=color,
        )
    ax.axhline(50, color="#777", ls=":")
    ax.axhline(-50, color="#777", ls=":")
    ax.set(
        xlabel="Progress along static transfer (%)",
        ylabel="Pressure offset within sole (mm)",
        title="Pressure starts centered · ≤2 mm change per sample",
    )
    ax.legend(fontsize=8)
    ax.grid(alpha=0.2)
    fig.suptitle(
        "Offline lean/posture search · static predictions only · no new Gazebo trial",
        fontsize=14,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(args.folder / "comparison.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
