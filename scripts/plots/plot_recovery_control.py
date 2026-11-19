#!/usr/bin/env python3
"""Compare explicit PD/PID evidence and show what the added integral does."""

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


def load(folder):
    path = folder / "push"
    summary = json.loads((path / "recording/summary.json").read_text())
    samples = [
        json.loads(s)
        for s in (path / "recording/samples.jsonl").read_text().splitlines()
    ]
    terms = [
        json.loads(s) for s in (path / "controller.jsonl").read_text().splitlines()
    ]
    start = next(
        e["sim_time"] for e in summary["events"] if e["event"] == "push_published"
    )
    terms = [
        r
        for r in terms
        if samples[0]["sim_time"] <= r["sim_time"] <= samples[-1]["sim_time"]
    ]
    return summary, samples, terms, start


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pd", type=Path, required=True)
    parser.add_argument("--pid", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    pd = load(args.pd)
    pid = load(args.pid)
    if pd[0]["requested"] != pid[0]["requested"]:
        raise ValueError("Comparison requires matching push protocol")
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5), sharex=True)
    fig.patch.set_facecolor("#f5f8fa")
    for data, label, color in (
        (pd, "PD: offset remains", "#b65b48"),
        (pid, "PID: returns within bands", "#147d92"),
    ):
        summary, rows, terms, start = data
        axes[0, 0].plot(
            [r["sim_time"] - start for r in rows],
            [math.degrees(r["pitch"] - summary["baseline_pitch_rad"]) for r in rows],
            label=label,
            color=color,
            lw=2,
        )
    s, rows, terms, start = pid
    t = [r["sim_time"] - start for r in rows]
    ct = [r["sim_time"] - start for r in terms]
    axes[0, 0].axhspan(
        -0.1, 0.1, color="#66834b", alpha=0.14, label="Pitch return band"
    )
    axes[0, 0].set(
        title="1. Same 2 N request, different recovery",
        ylabel="Pitch change from own baseline (degrees)",
    )
    colors = ["#618244", "#ae7a24", "#147d92"]
    for i, (name, color) in enumerate(zip(("Hip", "Knee", "Ankle"), colors)):
        axes[0, 1].plot(
            t,
            [
                r["actual_joints"][i] - s["baseline_joint_positions_rad"][i]
                for r in rows
            ],
            label=name,
            color=color,
        )
        axes[1, 1].plot(ct, [r["integral"][i] for r in terms], label=name, color=color)
    axes[0, 1].axhspan(-0.002, 0.002, color="#66834b", alpha=0.14)
    axes[0, 1].set(
        title="2. All joint offsets must return", ylabel="Left-leg joint change (rad)"
    )
    for key, label, color in (
        ("proportional", "P: angle error", "#ae7a24"),
        ("damping", "D: speed feedback", "#b65b48"),
        ("integral", "I: accumulated error", "#618244"),
        ("command", "Total motor command", "#147d92"),
    ):
        axes[1, 0].plot(
            ct, [r[key][2] for r in terms], label=label, color=color, lw=1.8
        )
    axes[1, 0].set(title="3. Left ankle torque contributions", ylabel="Torque (N m)")
    axes[1, 0].text(
        0.98,
        0.97,
        "Ankle feedforward = 0 N m\nTotal motor cap = ±6 N m",
        ha="right",
        va="top",
        transform=axes[1, 0].transAxes,
        fontsize=8,
    )
    axes[1, 1].set(
        title="4. Integral supplies torque after the error shrinks",
        ylabel="Left-leg integral contribution (N m)",
    )
    axes[1, 1].text(
        0.98,
        0.97,
        "Each integral contribution is bounded at ±1 N m",
        ha="right",
        va="top",
        transform=axes[1, 1].transAxes,
        fontsize=8,
    )
    for ax in axes.flat:
        ax.axvspan(0, s["command_window_s"], color="#ae7a24", alpha=0.13)
        ax.grid(alpha=0.18)
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(fontsize=8, loc="lower right")
    for ax in axes[1]:
        ax.set_xlabel("Simulation seconds since push publication")
    last = s["last_outside_band_after_clear_s"]
    subtitle = f"PID recovery passed: {s['passed']} | last outside any band: {last:.3f} s after clear | both feet: fresh ground-contact evidence"
    fig.suptitle(
        "Small-push recovery: why we added bounded integration",
        fontsize=17,
        fontweight="bold",
        color="#17324d",
    )
    fig.text(0.5, 0.915, subtitle, ha="center", fontsize=10)
    fig.text(
        0.5,
        0.018,
        "Separate fresh simulations. Pitch and joint traces use each run’s own baseline. Torque traces are requested commands; this is joint posture control, not IMU balance.",
        ha="center",
        fontsize=8,
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.91))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=160)
    plt.close(fig)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
