#!/usr/bin/env python3
"""Plot timestep sensitivity without conflating recovery and physics-audit success."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math
import statistics
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def plot_resolution_screen(screen, folder):
    """Put repeat ranges and refinement changes on the same numerical scale."""
    groups = screen["groups"]
    changes = screen["refinement_changes"]
    labels = [f'Repeat range\n{g["physics_step_s"]*1000:g} ms' for g in groups]
    labels += [
        f'Mean change\n{c["from_step_s"]*1000:g} → {c["to_step_s"]*1000:g} ms'
        for c in changes
    ]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.4))
    fig.patch.set_facecolor("#f5f8fa")
    for ax, group_key, change_key, tolerance, scale, title, unit in (
        (
            axes[0],
            "final_pitch_change_deg",
            "pitch_mean_change_deg",
            screen["screen_tolerances"]["pitch_deg"],
            1.0,
            "Remaining pitch change",
            "Degrees",
        ),
        (
            axes[1],
            "final_joint_change_rad",
            "joint_mean_change_rad",
            screen["screen_tolerances"]["joint_rad"],
            1000.0,
            "Largest-joint-offset metric",
            "Milliradians",
        ),
    ):
        values = [g[group_key]["range"] * scale for g in groups] + [
            c[change_key] * scale for c in changes
        ]
        ax.bar(
            range(len(values)),
            values,
            color=["#167d9a"] * len(groups) + ["#c65442"] * len(changes),
            width=0.65,
        )
        height = max(max(values), tolerance * scale) * 1.2
        ax.set_ylim(0, height)
        for i, value in enumerate(values):
            label = f"{value:.4f}" if value >= 0.0001 or value == 0 else "<0.0001"
            y = max(value + height * 0.025, tolerance * scale + height * 0.012)
            ax.text(
                i,
                y,
                label,
                ha="center",
                fontsize=9,
                bbox=dict(facecolor="white", edgecolor="none", pad=1),
            )
        ax.axhline(
            tolerance * scale,
            color="#597e46",
            ls="--",
            label=f"Screen threshold: {tolerance*scale:g}",
        )
        ax.set_xticks(range(len(labels)), labels, fontsize=8)
        ax.set_title(title)
        ax.set_ylabel(unit)
        ax.legend(fontsize=8, loc="upper left")
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=0.15)
        ax.set_axisbelow(True)
    result = "met" if screen["resolution_screen_passed"] else "not met"
    fig.suptitle(
        f"Resolution screen {result} | compare refinement changes with repeat variation",
        fontsize=15,
        fontweight="bold",
        color="#17324d",
    )
    fig.text(
        0.5,
        0.025,
        "n=2 per step: ranges are not confidence intervals. The decision uses the finest pair and its repeat ranges.",
        ha="center",
        fontsize=9,
        color="#647586",
    )
    fig.tight_layout(rect=(0, 0.06, 1, 0.91))
    path = folder / "resolution-screen.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    print(path.resolve())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    report = json.loads((args.folder / "comparison.json").read_text())
    trials = report["trials"]
    if not trials:
        raise ValueError("No completed timestep cases")
    steps = sorted({r["physics_step_s"] for r in trials}, reverse=True)
    colors = {s: c for s, c in zip(steps, ("#167d9a", "#c65442", "#597e46"))}
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.patch.set_facecolor("#f5f8fa")
    seen = set()
    for row in trials:
        path = (
            Path(row.get("recording_path", str(args.folder / row["case"])))
            / "push/recording"
        )
        summary = json.loads((path / "summary.json").read_text())
        samples = [
            json.loads(line)
            for line in (path / "samples.jsonl").read_text().splitlines()
        ]
        start = next(
            e["sim_time"] for e in summary["events"] if e["event"] == "push_published"
        )
        step = row["physics_step_s"]
        count = sum(r["physics_step_s"] == step for r in trials)
        label = f"{step*1000:g} ms (n={count})" if step not in seen else "_nolegend_"
        seen.add(step)
        axes[0, 0].plot(
            [s["sim_time"] - start for s in samples],
            [math.degrees(s["pitch"] - summary["baseline_pitch_rad"]) for s in samples],
            color=colors[row["physics_step_s"]],
            label=label,
            lw=2,
            alpha=0.8,
        )
    axes[0, 0].axhspan(
        -0.1, 0.1, color="#597e46", alpha=0.12, label="Pitch return band"
    )
    axes[0, 0].axvspan(0, 0.2, color="#aa7100", alpha=0.12)
    axes[0, 0].set_title("Pelvis pitch change from its own baseline")
    axes[0, 0].set_ylabel("Pitch change (degrees)")
    axes[0, 0].set_xlabel("Simulation seconds since push publication")
    axes[0, 0].legend(fontsize=8)
    for x, step in enumerate(steps):
        values = [
            r["final_pitch_change_deg"] for r in trials if r["physics_step_s"] == step
        ]
        axes[0, 1].bar(
            x, statistics.mean(values), color=colors[step], width=0.5, alpha=0.8
        )
        axes[0, 1].errorbar(
            x,
            statistics.mean(values),
            yerr=[
                [statistics.mean(values) - min(values)],
                [max(values) - statistics.mean(values)],
            ],
            fmt="none",
            capsize=5,
            color="#17324d",
            zorder=3,
        )
        axes[0, 1].scatter(
            [x + (i - (len(values) - 1) / 2) * 0.06 for i in range(len(values))],
            values,
            color="#17324d",
            s=24,
            zorder=3,
        )
        axes[0, 1].text(
            x,
            max(values) + 0.012,
            f"{statistics.mean(values):.3f}°",
            ha="center",
            fontsize=10,
        )
    axes[0, 1].axhspan(-0.1, 0.1, color="#597e46", alpha=0.12)
    axes[0, 1].set_xticks(range(len(steps)), [f"{s*1000:g} ms" for s in steps])
    axes[0, 1].set_ylabel("Final pitch change (degrees)")
    axes[0, 1].set_title("Remaining lean after the same observation time")
    labels = [
        "Total input",
        "Backend\ndamping",
        "Joint\nconstraint",
        "Transmitted\nmoment",
    ]
    x = np.arange(4)
    width = 0.8 / len(steps)
    for i, step in enumerate(steps):
        group = [r for r in trials if r["physics_step_s"] == step]
        keys = [
            "ankle_total_physics_input_nm",
            "ankle_implicit_damping_nm",
            "ankle_constraint_torque_nm",
            "ankle_transmitted_moment_nm",
        ]
        axes[1, 0].bar(
            x + (i - (len(steps) - 1) / 2) * width,
            [
                statistics.mean(r.get(k, r["ankle_motor_input_nm"]) for r in group)
                for k in keys
            ],
            width,
            color=colors[step],
            label=f"{step*1000:g} ms",
        )
    axes[1, 0].set_xticks(x, labels)
    axes[1, 0].axhline(0, color="#647586", lw=0.8)
    axes[1, 0].set_ylabel("Final per-ankle torque (N m)")
    axes[1, 0].set_title("Recorded physics terms in the final window")
    axes[1, 0].legend(fontsize=8)
    for x, step in enumerate(steps):
        values = [
            r["physics_input_impulse_n_s"]
            for r in trials
            if r["physics_step_s"] == step
        ]
        axes[1, 1].bar(
            x, statistics.mean(values), width=0.5, color=colors[step], alpha=0.8
        )
        axes[1, 1].scatter([x] * len(values), values, color="#17324d", s=24, zorder=3)
        axes[1, 1].text(
            x, max(values) + 0.003, f"{statistics.mean(values):.4f}", ha="center"
        )
    nominal = report["protocol"]["force_x_n"] * report["protocol"]["duration_s"]
    axes[1, 1].axhline(
        nominal, color="#aa7100", ls="--", label=f"Nominal command: {nominal:g} N s"
    )
    axes[1, 1].set_xticks(range(len(steps)), [f"{s*1000:g} ms" for s in steps])
    axes[1, 1].set_ylabel("Observed physics-input impulse (N s)")
    axes[1, 1].set_title("Check that the applied input was comparable")
    axes[1, 1].legend(fontsize=8, loc="lower right")
    for ax in axes.flat:
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=0.15)
        ax.set_axisbelow(True)
    fig.suptitle(
        f"Timestep comparison | fixed controller, robot and {report['protocol']['force_x_n']:g} N push request",
        fontsize=16,
        fontweight="bold",
        color="#17324d",
    )
    fig.text(
        0.5,
        0.015,
        "Dots represent individual trials. Compare resolution changes with repeat variation and the observed input impulses.",
        ha="center",
        fontsize=9,
        color="#647586",
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    output = args.folder / "comparison.png"
    fig.savefig(output, dpi=160)
    plt.close(fig)
    print(output.resolve())
    if "resolution_screen" in report:
        plot_resolution_screen(report["resolution_screen"], args.folder)


if __name__ == "__main__":
    main()
