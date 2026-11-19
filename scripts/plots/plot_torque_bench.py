#!/usr/bin/env python3
"""Show measured torque saturation, limited holding, and mechanical-stop tests."""

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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trials", type=Path)
    args = parser.parse_args()

    def rows(case):
        return [
            json.loads(line)
            for line in (args.trials / case / "recording/samples.jsonl")
            .read_text()
            .splitlines()
        ]

    fig, axes = plt.subplots(2, 2, figsize=(11, 7), layout="constrained")
    for case, label, color in [
        ("hold", "2 N·m allowance", "#185abc"),
        ("hold-limited", "0.25 N·m soft cap", "#e37400"),
    ]:
        data = rows(case)
        times = [r["time"] for r in data]
        axes[0, 0].plot(times, [r["angle"] for r in data], label=label, color=color)
        axes[1, 0].plot(times, [r["command"] for r in data], label=label, color=color)
        if case == "hold":
            axes[0, 0].plot(
                times,
                [r["target"] for r in data],
                color="#5f6368",
                linestyle="--",
                label="Target",
            )
    axes[0, 0].set_title("Same angle target; different available torque")
    axes[0, 0].set_ylabel("Joint angle (rad)")
    axes[1, 0].set_ylabel("Published torque (N·m)")
    axes[1, 0].set_xlabel("Simulated time (s)")
    axes[1, 0].axhline(0.25, color="#e37400", linestyle=":", linewidth=0.8)
    for case, color in [("stop-positive", "#185abc"), ("stop-negative", "#e37400")]:
        data = rows(case)
        axes[0, 1].plot(
            [r["time"] for r in data],
            [r["angle"] for r in data],
            label=case,
            color=color,
        )
    axes[0, 1].axhline(0.8, color="#c5221f", linestyle=":")
    axes[0, 1].axhline(-0.8, color="#c5221f", linestyle=":")
    axes[0, 1].set_title("Both travel limits respected")
    axes[0, 1].set_ylabel("Joint angle (rad)")
    axes[0, 1].set_xlabel("Simulated time (s)")
    summary = json.loads((args.trials / "summary.json").read_text())
    cases = ["pulse-negative", "pulse-positive"]
    axes[1, 1].bar(
        [-0.18, 0.82],
        [-4, 4],
        width=0.34,
        label="Raw requested torque",
        color="#a8c7fa",
    )
    axes[1, 1].bar(
        [0.18, 1.18],
        [summary[c]["dynamics_inferred_torque_nm"] for c in cases],
        width=0.34,
        label="Inferred from motion",
        color="#185abc",
    )
    axes[1, 1].set_xticks([0, 1], ["Negative pulse", "Positive pulse"])
    axes[1, 1].axhline(2, color="#c5221f", linestyle=":")
    axes[1, 1].axhline(-2, color="#c5221f", linestyle=":")
    axes[1, 1].set_ylabel("Torque (N·m)")
    axes[1, 1].set_title("2 N·m hard command cap verified by dynamics")
    for axis in axes.flat:
        axis.grid(alpha=0.2)
        axis.legend(fontsize=8)
    fig.suptitle(
        "Independent single-joint effort bench — the biped position controller is unchanged"
    )
    fig.savefig(args.trials / "response.png", dpi=160)
    plt.close(fig)


if __name__ == "__main__":
    main()
