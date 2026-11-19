#!/usr/bin/env python3
"""Visualize measured COM/foot motion separately from static force estimates."""

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


def read(path):
    return [json.loads(s) for s in path.read_text().splitlines()]


def series(folder):
    control = [r for r in read(folder / "joints.jsonl") if r.get("shift") is not None]
    origin = control[0]["sim_time"] - control[0]["shift"]["elapsed_s"]
    control = [r for r in control if r["shift"]["elapsed_s"] <= 25]
    com = [
        r for r in read(folder / "com.jsonl") if origin <= r["sim_time"] <= origin + 25
    ]
    foot = lambda r, s, i: sum(c[i] for c in r["feet"][s]) / 4
    rel = lambda r: r["com"][1] - (foot(r, "left", 1) + foot(r, "right", 1)) / 2
    baseline = [r for r in com if r["sim_time"] < origin + 2.5]
    y0 = sum(rel(r) for r in baseline) / len(baseline)
    foot0 = {
        s: [sum(foot(r, s, i) for r in baseline) / len(baseline) for i in range(3)]
        for s in ["left", "right"]
    }
    return control, com, origin, foot, rel, y0, foot0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--compare", type=Path)
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    for folder, color, label in [
        (args.compare, "#c65442", "Initial roll integral 20 · checks not met"),
        (args.folder, "#167d9a", "Shift roll integral 5"),
    ]:
        if folder is None:
            continue
        rows, com, origin, foot, rel, y0, foot0 = series(folder)
        axes[0, 0].plot(
            [r["sim_time"] - origin for r in com],
            [(rel(r) - y0) * 1000 for r in com],
            label=label,
            color=color,
        )
        axes[0, 1].plot(
            [r["shift"]["elapsed_s"] for r in rows],
            [math.degrees(r["error"][0]) for r in rows],
            color=color,
            label=label,
        )
        if folder != args.folder:
            continue
        t = [r["shift"]["elapsed_s"] for r in rows]
        axes[0, 0].plot(
            t,
            [r["shift"]["predicted_com_y_m"] * 1000 for r in rows],
            color="#333",
            ls="--",
            label="Planned COM (model)",
        )
        for side, c in [("left", "#167d9a"), ("right", "#c77a22")]:
            axes[1, 0].plot(
                [r["sim_time"] - origin for r in com],
                [
                    1000
                    * math.sqrt(
                        sum((foot(r, side, i) - foot0[side][i]) ** 2 for i in range(3))
                    )
                    for r in com
                ],
                label=side,
                color=c,
            )
            axes[1, 1].plot(
                t,
                [r["shift"]["predicted_force_n"][side] for r in rows],
                label=side + " · estimate",
                color=c,
                ls="--",
            )
    axes[1, 0].axhline(1, color="#999", ls=":", label="Foot-drift limit")
    for ax, title, ylabel in zip(
        axes.flat,
        [
            "Measured COM shift relative to feet",
            "Left hip-roll tracking error",
            "Measured foot-center drift",
            "Static normal-force estimates · NOT measured",
        ],
        ["Change from baseline (mm)", "Degrees", "Distance (mm)", "Estimated N"],
    ):
        ax.set(
            title=title,
            ylabel=ylabel,
            xlabel="Simulation seconds after readiness",
            xlim=(0, 25),
        )
        ax.grid(alpha=0.2)
        for time in [3, 7, 13, 17]:
            ax.axvline(time, color="#aaa", alpha=0.3, ls=":")
        ax.axvspan(7, 13, color="#167d9a", alpha=0.06)
    for ax in [axes[0, 0], axes[1, 0], axes[1, 1]]:
        ax.legend(fontsize=8)
    fig.suptitle(
        "Small bilateral lateral shift and return\n0–3 s baseline · 3–7 move · 7–13 hold · 13–17 return · 17–25 recover"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(args.folder / "response.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
