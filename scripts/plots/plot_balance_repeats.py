#!/usr/bin/env python3
"""Show individual off/on repeats and observed ranges, not confidence intervals."""

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
    report = json.loads((args.folder / "repeat-study.json").read_text())
    if not report["completed"]:
        raise ValueError("Repeat study is not complete")
    assessment = report["assessment"]
    groups = assessment["groups"]
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.3))
    fig.patch.set_facecolor("#f5f8fa")
    colors = {"off": "#b36b42", "on": "#147e96"}
    for ax, key, scale, title, unit in (
        (axes[0, 0], "peak_pitch_deg", 1.0, "Peak lean: off versus on", "Degrees"),
        (
            axes[1, 0],
            "last_outside_band_s",
            1.0,
            "Last sample outside recovery bands",
            "Seconds after clear",
        ),
        (
            axes[1, 1],
            "final_joint_offset_rad",
            1000.0,
            "Largest final joint change from baseline",
            "Milliradians",
        ),
    ):
        for d, direction in enumerate(("forward", "backward")):
            group = groups[direction]
            for m, mode in enumerate(("off", "on")):
                x = d + (m - 0.5) * 0.3
                metric = group["metrics"][mode][key]
                label = "Joint PID only" if mode == "off" else "Joint PID + IMU"
                ax.bar(
                    x,
                    metric["mean"] * scale,
                    width=0.27,
                    color=colors[mode],
                    alpha=0.7,
                    label=label if d == 0 else "_nolegend_",
                )
                values = [
                    (
                        (p[mode]["last_outside_band_after_clear_s"] or 0.0)
                        if key == "last_outside_band_s"
                        else p[mode][key]
                    )
                    * scale
                    for p in group["pairs"]
                ]
                ax.vlines(x, min(values), max(values), color="#17324d", lw=1.3)
                for j, value in enumerate(values):
                    ax.scatter(
                        x + (j - (len(values) - 1) / 2) * 0.045,
                        value,
                        s=25,
                        color="#17324d",
                        zorder=3,
                    )
        ax.set_xticks([0, 1], ["Forward +2 N", "Backward -2 N"])
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.legend(fontsize=8, loc="upper right")
    axes[1, 1].axhline(2.0, ls=":", color="#638b50", label="Joint return band: 2 mrad")
    axes[1, 1].legend(fontsize=8, loc="upper right")
    ax = axes[0, 1]
    labels = []
    values = []
    for direction in ("forward", "backward"):
        for pair in groups[direction]["pairs"]:
            labels.append(f'{direction.title()}\nrepeat {pair["repeat"]}')
            values.append(pair["peak_reduction_percent"])
    ax.bar(range(len(values)), values, color=["#147e96", "#78aebe"] * 2, width=0.65)
    for i, value in enumerate(values):
        ax.annotate(
            f"{value:.1f}%",
            (i, value),
            xytext=(0, 6 if value >= 0 else -14),
            textcoords="offset points",
            ha="center",
            fontsize=9,
        )
    ax.axhline(
        assessment["minimum_paired_peak_reduction_percent"],
        ls="--",
        color="#638b50",
        label="Consistency threshold: 5%",
    )
    ax.set_xticks(range(len(labels)), labels, fontsize=8)
    ax.set_ylabel("Peak reduction relative to paired off run (%)")
    ax.set_title("Does every pair retain the peak-lean benefit?")
    ax.legend(fontsize=8)
    ax.margins(y=0.2)
    for ax in axes.flat:
        ax.grid(axis="y", alpha=0.18)
        ax.set_axisbelow(True)
        ax.spines[["top", "right"]].set_visible(False)
    result = "met" if assessment["peak_reduction_consistent"] else "not met"
    fig.suptitle(
        f"Repeat consistency screen {result} | fixed gains, robot and nominal pushes",
        fontsize=15,
        fontweight="bold",
    )
    fig.text(
        0.5,
        0.015,
        "Dots are individual trials; lines show observed ranges. n=2 pairs per direction: no confidence intervals or general stability claim. Force impulses are not independently measured.",
        ha="center",
        fontsize=8,
    )
    fig.tight_layout(rect=(0, 0.045, 1, 0.94))
    path = args.folder / "repeat-summary.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    print(path.resolve())


if __name__ == "__main__":
    main()
