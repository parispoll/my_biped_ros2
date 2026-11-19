#!/usr/bin/env python3
"""Show measured torque terms for the settled left leg; right leg is symmetric."""

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
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "recording", type=Path, help="Experiment folder containing joints.jsonl"
    )
    args = parser.parse_args()
    rows = [
        json.loads(s)
        for s in (args.recording / "joints.jsonl").read_text().splitlines()
    ]
    active = [r for r in rows if r["active"]]
    if not active:
        raise ValueError("No active effort samples")
    settled = [r for r in active if r["sim_time"] - active[0]["sim_time"] >= 2]
    if not settled:
        raise ValueError("No settled interval")
    last = settled[-1]
    fig, axes = plt.subplots(
        1, 2, figsize=(12, 5.4), gridspec_kw={"width_ratios": [1.15, 1]}
    )
    fig.patch.set_facecolor("#f5f8fa")
    names = ["Hip", "Knee", "Ankle"]
    x = np.arange(3)
    width = 0.22
    colors = ["#167d9a", "#db8852", "#597e46"]
    for offset, key, label, color in zip(
        (-1, 0, 1),
        ("feedforward", "proportional", "command"),
        ("Static holding estimate", "Angle correction", "Total torque command"),
        colors,
    ):
        axes[0].bar(x + offset * width, last[key][:3], width, label=label, color=color)
    axes[0].axhline(0, color="#748490", lw=0.8)
    axes[0].set_xticks(x, names)
    axes[0].set_ylabel("Torque (N m)")
    axes[0].set_title("Where the settled torque comes from")
    axes[0].legend(loc="lower left", fontsize=8)
    times = [r["sim_time"] - active[0]["sim_time"] for r in active]
    for i, (name, color) in enumerate(zip(names, colors)):
        axes[1].plot(
            times,
            [r["error"][i] * 180 / np.pi for r in active],
            label=name,
            color=color,
            lw=2,
        )
    axes[1].axhline(0, color="#748490", lw=0.8)
    axes[1].set_xlabel("Seconds since activation was confirmed")
    axes[1].set_ylabel("Target minus measured angle (degrees)")
    axes[1].set_title("Small errors remain with finite stiffness")
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=0.15)
        ax.set_axisbelow(True)
    fig.suptitle(
        "Effort-controlled standing | measured left-leg commands",
        fontsize=16,
        fontweight="bold",
        color="#17324d",
        y=0.98,
    )
    fig.text(
        0.5,
        0.02,
        "Speed correction is effectively zero at rest. Both legs agree. Static standing does not establish push recovery.",
        ha="center",
        fontsize=9,
        color="#647586",
    )
    fig.tight_layout(rect=(0, 0.06, 1, 0.93))
    path = args.recording / "response.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    print(path.resolve())


if __name__ == "__main__":
    main()
