#!/usr/bin/env python3
"""Plot the recorded responses of a completed duration comparison."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    folder = args.folder.resolve()
    study = json.loads((folder / "comparison.json").read_text())
    if not study.get("completed"):
        raise ValueError(
            "A complete comparison is required; inspect comparison.json for errors"
        )
    figure, axes = plt.subplots(
        2, 1, figsize=(9, 6.5), sharex=True, layout="constrained"
    )
    for case, color in zip(study["cases"], ("#267985", "#c26725")):
        trial = Path(case["path"]) / "push/recording"
        summary = json.loads((trial / "summary.json").read_text())
        samples = [
            json.loads(line)
            for line in (trial / "samples.jsonl").read_text().splitlines()
        ]
        start = next(
            e["sim_time"] for e in summary["events"] if e["event"] == "push_published"
        )
        clear = (
            next(
                e["sim_time"]
                for e in summary["events"]
                if e["event"] == "clear_published"
            )
            - start
        )
        x = [r["sim_time"] - start for r in samples]
        axes[0].plot(
            x,
            [math.degrees(r["pitch"] - summary["baseline_pitch_rad"]) for r in samples],
            color=color,
            label=f"{case['duration_s']:.2f} s requested",
        )
        axes[1].plot(x, [r["pitch_rate"] for r in samples], color=color)
        for ax in axes:
            ax.axvline(clear, color=color, linestyle=":", alpha=0.8)
    for ax, band in zip(axes, (0.1, 0.01)):
        ax.axhspan(
            -band, band, color="#d6e7d6", alpha=0.6, label="Return-band component"
        )
        ax.axvline(0, color="black", linestyle="--", linewidth=0.8)
        ax.grid(alpha=0.2)
    axes[0].set_ylabel("Pitch change (degrees)")
    axes[1].set_ylabel("Pitch rate (rad/s)")
    axes[1].set_xlabel("Simulated seconds after push publication")
    axes[0].legend(loc="best")
    direction = study.get("direction", "backward").capitalize()
    figure.suptitle(
        f"{direction} 2 N: increase pulse duration, keep controller unchanged"
    )
    figure.supxlabel(
        "Dotted lines: command clear. Force actuation timing is not measured.",
        fontsize=9,
    )
    path = folder / "response-comparison.png"
    figure.savefig(path, dpi=160)
    plt.close(figure)
    print(path)


if __name__ == "__main__":
    main()
