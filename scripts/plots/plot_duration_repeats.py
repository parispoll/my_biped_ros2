#!/usr/bin/env python3
"""Show individual duration repeats and observed peak ranges."""

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
from scripts.analysis.duration_repeat_analysis import describe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    folder = args.folder.resolve()
    study = json.loads((folder / "repeat-study.json").read_text())
    if not study.get("completed"):
        raise ValueError("Require a completed repeat study")
    results = describe(study["entries"])
    fig, axes = plt.subplots(1, 2, figsize=(9, 4), layout="constrained")
    ceiling = (
        max(
            row["max"]
            for result in results.values()
            for row in result["peak_pitch_deg"].values()
        )
        * 1.15
    )
    for ax, direction in zip(axes, ("forward", "backward")):
        data = results[direction]["peak_pitch_deg"]
        for x, key, color in ((0, "0.2", "#267985"), (1, "0.3", "#c26725")):
            row = data[key]
            center = row["mean"]
            ax.bar(x, center, color=color, alpha=0.65, width=0.55)
            ax.errorbar(
                x,
                center,
                yerr=[[center - row["min"]], [row["max"] - center]],
                fmt="none",
                color="black",
                capsize=5,
            )
            ax.scatter(
                [x - 0.05, x + 0.05], row["values"], color="black", s=25, zorder=3
            )
        ax.set_xticks([0, 1], ["0.20 s", "0.30 s"])
        ax.set_xlabel("Requested duration")
        ax.set_ylabel("Peak pitch change (degrees)")
        ax.set_title(direction.capitalize() + " · 2 N")
        ax.set_ylim(0, ceiling)
        ax.grid(axis="y", alpha=0.2)
        ax.set_axisbelow(True)
    fig.suptitle("Pulse-duration effect: two repeats per direction")
    fig.supxlabel(
        "Dots: individual trials. Whiskers: observed min–max, not confidence intervals.",
        fontsize=9,
    )
    path = folder / "repeat-summary.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    print(path)


if __name__ == "__main__":
    main()
