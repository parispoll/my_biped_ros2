#!/usr/bin/env python3
"""Plot command timing, pitch, joint errors and contact freshness from push trials."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/biped-matplotlib")
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trials", type=Path)
    args = parser.parse_args()
    reports = json.loads((args.trials / "summary.json").read_text())
    fig, axes = plt.subplots(4, 1, figsize=(11, 9), sharex=True, layout="constrained")
    for index, (name, report) in enumerate(reports.items()):
        rows = [
            json.loads(line)
            for line in (args.trials / name / "recording/samples.jsonl")
            .read_text()
            .splitlines()
        ]
        onset = next(
            e["sim_time"] for e in report["events"] if e["event"] == "push_published"
        )
        times = [row["sim_time"] - onset for row in rows]
        color = ["#5f6368", "#185abc", "#e37400"][index % 3]
        style = "--" if name == "control" else "-"
        force = report["requested"]["force_x"]
        axes[0].plot(
            times,
            [force if row["phase"] == "push" else 0 for row in rows],
            label=name,
            color=color,
            linestyle=style,
        )
        axes[1].plot(
            times,
            [(row["pitch"] - report["baseline_pitch_rad"]) * 1e6 for row in rows],
            label=name,
            color=color,
            linestyle=style,
        )
        axes[2].plot(
            times,
            [max(abs(e) for e in row["joint_error"]) for row in rows],
            label=name,
            color=color,
            linestyle=style,
        )
        axes[3].plot(
            times,
            [
                max(row["contacts"][s]["age"] for s in ("left", "right")) * 1000
                for row in rows
            ],
            label=name,
            color=color,
            linestyle=style,
        )
    axes[0].set_ylabel("Requested force X (N)")
    axes[1].set_ylabel("Pitch change (µrad)")
    axes[2].set_ylabel("Max joint error (rad)")
    axes[3].set_ylabel("Oldest contact age (ms)")
    axes[3].axhline(50, color="#c5221f", linestyle=":", label="Unknown above 50 ms")
    axes[3].set_xlabel("Simulated seconds relative to push command")
    for axis in axes:
        axis.axvline(0, color="#999999", linewidth=0.7)
        axis.grid(alpha=0.25)
        axis.legend(fontsize=8, loc="upper right")
    fig.suptitle(
        "Small-push baseline: 5 N for ~0.20 s; existing position controller unchanged\nPitch response is ~0.1 µrad, so this is not a substantial tipping/recovery test",
        fontsize=12,
    )
    fig.savefig(args.trials / "response.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
