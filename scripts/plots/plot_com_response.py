#!/usr/bin/env python3
"""Plot recorded COM relative to the feet; no dynamic-stability verdict."""

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
    folder = parser.parse_args().folder
    rows = []
    for line in (folder / "com.jsonl").read_text().splitlines():
        try:
            rows.append(json.loads(line))
        except ValueError:
            pass
    events = json.loads((folder / "visual-events.json").read_text())
    push = next(e["sim_time"] for e in events if e["event"] == "push_published")
    clear = next(e["sim_time"] for e in events if e["event"] == "clear_published")
    rows = [r for r in rows if push - 1 <= r["sim_time"] <= clear + 8]
    baseline = [r for r in rows if r["sim_time"] < push]

    def foot_center(r):
        return sum(c[0] for corners in r["feet"].values() for c in corners) / 8

    x0 = sum(r["com"][0] - foot_center(r) for r in baseline) / len(baseline)
    z0 = sum(r["com"][2] for r in baseline) / len(baseline)
    times = [r["sim_time"] - push for r in rows]
    x = [100 * (r["com"][0] - foot_center(r) - x0) for r in rows]
    z = [1000 * (r["com"][2] - z0) for r in rows]
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(times, x, color="#087f9c", lw=2)
    axes[0].set(
        ylabel="COM fore/aft change (cm)",
        title="Whole-body COM motion during visual push and recovery",
    )
    axes[1].plot(times, z, color="#a07100", lw=2)
    axes[1].set(
        ylabel="COM height change (mm)",
        xlabel="Simulation seconds after push publication",
    )
    for ax in axes:
        ax.axvspan(
            0, clear - push, color="#e0ad4b", alpha=0.25, label="Published pulse window"
        )
        ax.axhline(0, color="#888", lw=0.6)
        ax.grid(alpha=0.2)
        ax.legend()
    fig.text(
        0.5,
        0.015,
        "Calculated from all Gazebo link inertias and world poses. Fore/aft motion is relative to current foot centers and the pre-push baseline. No stability verdict.",
        ha="center",
        fontsize=8,
    )
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    fig.savefig(folder / "com-response.png", dpi=150)
    plt.close(fig)
    report = dict(
        total_mass_kg=rows[0]["mass_kg"],
        samples=len(rows),
        peak_fore_aft_change_cm=max(abs(v) for v in x),
        nominal_projection_inside_foot_x_bounds=all(
            min(c[0] for corners in r["feet"].values() for c in corners)
            <= r["com"][0]
            <= max(c[0] for corners in r["feet"].values() for c in corners)
            for r in rows
        ),
        note="Geometric x bounds only; contact and dynamics are not certified by this plot.",
    )
    (folder / "com-response.json").write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
