#!/usr/bin/env python3
"""Panel artifact for a P54 transfer: measured COM, forces and lateral IMU."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))
from my_robot_description.sensing.foot_loads import paired_loads


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("folder", type=Path)
    args = a.parse_args()
    p = args.folder
    s = json.loads((p / "summary.json").read_text())
    start, end = s["shift"]["origin_sim_time"], s["shift"]["end_sim_time"]
    read = lambda name: [
        json.loads(line) for line in (p / name).read_text().splitlines()
    ]
    pairs = paired_loads(read("contact-wrenches.jsonl"), start, end)
    com = [r for r in read("com.jsonl") if start <= r["sim_time"] <= end]
    imu = [r for r in read("imu-raw.jsonl") if start <= r["sim_time"] <= end]
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(
        [r["sim_time"] - start for r in com],
        [1000 * r["com"][1] for r in com],
        color="#167d9a",
    )
    axes[0].set(ylabel="COM world Y (mm)")
    for side, color in [("left", "#167d9a"), ("right", "#c77a22")]:
        axes[1].plot(
            [r["sim_time"] - start for r in pairs],
            [100 * r[side]["force_world_n"][2] / r["total_vertical_n"] for r in pairs],
            color=color,
            label=side,
        )
    axes[1].axhline(25, color="#888", ls=":")
    axes[1].set(ylabel="Vertical load share (%)")
    axes[1].legend()
    axes[2].plot(
        [r["sim_time"] - start for r in imu],
        [r["acceleration"][1] for r in imu],
        color="#167d9a",
    )
    axes[2].set(
        ylabel="IMU lateral acceleration (m/s²)",
        xlabel="Simulated seconds after standing readiness",
    )
    pulse = s.get("physical_pulse")
    for ax in axes:
        ax.grid(alpha=0.2)
        if pulse and len(pulse.get("events", [])) >= 2:
            ax.axvspan(
                pulse["events"][0]["sim_time"] - start,
                pulse["events"][1]["sim_time"] - start,
                color="#d2663b",
                alpha=0.2,
            )
    fig.suptitle(
        "P54 bilateral transfer · "
        + ("checks passed" if s["passed"] else "checks not met")
        + "\nOrange band: commanded pulse window; force timing/magnitude not independently calibrated"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(p / "response.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
