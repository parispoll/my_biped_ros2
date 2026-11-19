#!/usr/bin/env python3
"""Compare a failed and successful run using recorded controller measurements."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/biped-matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(path):
    rows = [json.loads(line) for line in path.read_text().splitlines()]
    rows = [row for row in rows if row["phase"] == "active"]
    # Stop at the first violation; later hold-position samples are a different phase.
    for index, row in enumerate(rows):
        if any(abs(error) >= 0.08 for error in row["error"]):
            return rows[: index + 1]
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("failed", type=Path)
    parser.add_argument("successful", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 2, figsize=(11, 6), sharex="col", layout="constrained")
    for column, (path, title) in enumerate(
        (
            (args.failed, "Started at knee lower stop"),
            (args.successful, "Started with 0.02 rad knee clearance"),
        )
    ):
        rows = load(path)
        if not rows:
            raise ValueError(f"No active trajectory samples: {path}")
        origin = rows[0]["controller_time"]
        times = [row["controller_time"] - origin for row in rows]
        left = rows[0]["joint_names"].index("left_knee")
        right = rows[0]["joint_names"].index("right_knee")
        axes[0, column].plot(
            times,
            [row["desired"][left] for row in rows],
            label="Desired left knee",
            color="#185abc",
        )
        axes[0, column].plot(
            times,
            [row["actual"][left] for row in rows],
            label="Measured left knee",
            color="#e37400",
            linestyle="--",
        )
        axes[0, column].set_title(title)
        axes[0, column].set_ylabel("Knee angle (rad)")
        for index, label in ((left, "Left knee"), (right, "Right knee")):
            axes[1, column].plot(
                times, [row["error"][index] for row in rows], label=label
            )
        axes[1, column].axhline(
            0.08, color="#c5221f", linestyle=":", label="Unchanged path tolerance"
        )
        axes[1, column].set_ylabel("Tracking error (rad)")
        axes[1, column].set_xlabel("Simulated seconds since first active sample")
        for axis in axes[:, column]:
            axis.grid(alpha=0.25)
            axis.legend(fontsize=8)
    fig.suptitle(
        "Crouch diagnosis: the failed knees remained at zero despite advancing commands"
    )
    fig.savefig(args.output, dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
