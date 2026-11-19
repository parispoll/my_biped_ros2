#!/usr/bin/env python3
"""Plot a roll-standing recording, optionally alongside an unsuccessful startup."""

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
    parser.add_argument("--compare", type=Path)
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
    for folder, color, label in [
        (args.compare, "#c65442", "Transferred integral gains · readiness not met"),
        (args.folder, "#167d9a", "Reduced pitch integral gains"),
    ]:
        if folder is None:
            continue
        data = [
            json.loads(s) for s in (folder / "joints.jsonl").read_text().splitlines()
        ]
        data = [r for r in data if r["active"]]
        start = data[0]["sim_time"]
        t = [r["sim_time"] - start for r in data]
        axes[0, 0].plot(
            t,
            [math.degrees(r["pelvis_tilt_rad"]) for r in data],
            color=color,
            label=label,
        )
        axes[0, 1].plot(t, [math.degrees(r["error"][3]) for r in data], color=color)
        axes[1, 0].plot(t, [math.degrees(r["error"][0]) for r in data], color=color)
        axes[1, 1].plot(t, [r["command"][2] for r in data], color=color)
        ready = next((r["sim_time"] - start for r in data if r["ready"]), None)
        if ready is not None:
            for ax in axes.flat:
                ax.axvline(ready, color=color, ls=":", alpha=0.7)
    for ax, title, ylabel in zip(
        axes.flat,
        [
            "Pelvis tilt",
            "Left ankle pitch tracking error",
            "Left hip roll tracking error",
            "Left knee motor command",
        ],
        ["Degrees", "Degrees", "Degrees", "N·m"],
    ):
        ax.set(
            title=title,
            ylabel=ylabel,
            xlabel="Simulation seconds after effort activation",
        )
        ax.grid(alpha=0.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle(
        "Ten-joint two-foot standing · same geometry and motor caps\nDotted line: readiness of the reduced-integral candidate; independent checks follow."
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(args.folder / "response.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
