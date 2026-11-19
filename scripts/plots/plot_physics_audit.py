#!/usr/bin/env python3
"""Explain the measured DART split-step ankle torque balance visually."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    report = json.loads((args.folder / "physics-audit.json").read_text())
    j = report["push"]["joints"]["left_ankle"]
    gravity = report["push"]["direct_ecm_gravity_per_ankle_nm"]
    fig = plt.figure(figsize=(12, 7.5))
    fig.patch.set_facecolor("#f5f8fa")
    fig.suptitle(
        "The missing ankle moment is accounted for inside the physics step",
        fontsize=17,
        fontweight="bold",
        color="#17324d",
        y=0.97,
    )
    boxes = [
        ("Start of step", "Measured speed ≈ 0"),
        (
            "Unconstrained prediction",
            f'Internal velocity ≈ +{j["mean_predicted_free_velocity_rad_s"]:.3f} rad/s',
        ),
        ("After constraint correction", "Measured speed ≈ 0"),
    ]
    for x, (title, value) in zip((0.18, 0.5, 0.82), boxes):
        fig.text(
            x,
            0.85,
            title + "\n" + value,
            ha="center",
            va="center",
            fontsize=10,
            bbox=dict(boxstyle="round,pad=.9", fc="white", ec="#a8bdc8"),
        )
    for x in (0.34, 0.66):
        fig.text(x, 0.85, "→", ha="center", fontsize=24, color="#167d9a")
    fig.text(
        0.5,
        0.735,
        "Passive damping uses the intermediate prediction; the final recorded speed alone cannot reconstruct it.",
        ha="center",
        fontsize=10,
        color="#647586",
    )
    ax = fig.add_axes([0.09, 0.15, 0.86, 0.48])
    motor = j["dart_force_before_constraints_nm"]
    damping = j["implicit_damping_nm"]
    constraint = j["joint_constraint_torque_nm"]
    values = [motor, damping, constraint, j["transmitted_joint_moment_nm"], gravity]
    bottoms = [0, motor, motor + damping, 0, 0]
    colors = ["#167d9a", "#c65442", "#aa7100", "#17324d", "#597e46"]
    ax.bar(range(5), values, bottom=bottoms, color=colors, width=0.62)
    for i, (value, bottom) in enumerate(zip(values, bottoms)):
        y = max(bottom, bottom + value) + 0.018
        ax.text(i, y, f"{value:+.3f} N m", ha="center", fontsize=11)
    for i in (0, 1):
        end = bottoms[i] + values[i]
        ax.plot([i + 0.31, i + 1 - 0.31], [end, end], ls=":", color="#647586")
    ax.axhline(gravity, color="#597e46", ls="--", lw=1)
    ax.set_xticks(
        range(5),
        [
            "Motor input",
            "Implicit passive\ndamping",
            "Joint constraint\nimpulse / timestep",
            "Transmitted\njoint moment",
            "Gravity estimate\nfrom Gazebo COMs",
        ],
    )
    ax.set_ylim(0, max(motor, 0.7) + 0.1)
    ax.set_ylabel("Per-ankle torque (N m)")
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="y", alpha=0.15)
    ax.set_axisbelow(True)
    equation = f'{motor:+.6f} {damping:+.6f} {constraint:+.6f} = {j["transmitted_joint_moment_nm"]:+.6f} N m'
    fig.text(
        0.5,
        0.065,
        equation,
        ha="center",
        fontsize=13,
        fontweight="bold",
        color="#17324d",
    )
    step_ms = report["push"].get("physics_step_measured_s", 0.001) * 1000
    fig.text(
        0.5,
        0.025,
        f"DART 6.13.2 · {step_ms:g} ms simulation timestep · intermediate solver values are not measured physical joint speeds.",
        ha="center",
        fontsize=9,
        color="#647586",
    )
    path = args.folder / "torque-balance.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    print(path.resolve())


if __name__ == "__main__":
    main()
