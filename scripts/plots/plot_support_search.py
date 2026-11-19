#!/usr/bin/env python3
"""Render saved offline posture-search results without recomputing the search."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import json
import numpy as np


def plot(report, folder):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    records = report["samples"]
    best = report["best"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5), sharey=True)
    for ax, f in zip(axes, [0.05, 0.0]):
        grid = np.full((7, 25), np.nan)
        for r in records:
            if r["right_fraction"] == f:
                grid[
                    round(r["pelvis_lowering_m"] / 0.01),
                    round(r["pelvis_shift_m"] / 0.005),
                ] = (
                    100 * r["peak_utilization"]
                )
        m = ax.imshow(
            grid,
            origin="lower",
            extent=(-2.5, 122.5, -5, 65),
            aspect="auto",
            vmin=175,
            vmax=250,
            cmap="magma_r",
        )
        b = best[str(f)]
        if b:
            ax.plot(
                1000 * b["pelvis_shift_m"],
                1000 * b["pelvis_lowering_m"],
                "*",
                color="cyan",
                markersize=13,
            )
        ax.set(
            title=f"Right-foot target: {100*f:g}% of weight",
            xlabel="Pelvis shift left (mm)",
            ylabel="Pelvis lowering (mm)",
        )
    fig.subplots_adjust(top=0.78)
    fig.colorbar(m, ax=list(axes), label="Peak static torque / motor cap (%)")
    fig.suptitle(
        "Near-zero-load search · all solved points exceed motor caps\nCyan: best sampled pose · blank: infeasible geometry or pressure"
    )
    fig.savefig(folder / "feasibility.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    entries = sorted((float(k), v) for k, v in best.items() if v)
    axes[0].plot(
        [100 * f for f, r in entries],
        [100 * r["peak_utilization"] for f, r in entries],
        "-o",
        color="#167d9a",
    )
    axes[0].axhline(80, color="#777", ls="--", label="20% reserve")
    axes[0].axhline(100, color="#bd4338", ls=":", label="Motor cap")
    axes[0].set(
        xlabel="Right-foot load (% of total)",
        ylabel="Best sampled peak static torque / cap (%)",
        title="Smaller load does not mean smaller motor demand",
    )
    axes[0].invert_xaxis()
    axes[0].legend()
    axes[0].grid(alpha=0.2)
    r = best["0.25"]
    w = sum(r["forces_n"].values())
    com = r["com_world_m"]
    h = r["pelvis_height_m"]
    d = r["pelvis_shift_m"]
    ax = axes[1]
    for side, y, c in [("left", 0.085, "#167d9a"), ("right", -0.085, "#c77a22")]:
        ax.add_patch(Rectangle((y - 0.06, 0), 0.12, 0.02, color=c, alpha=0.35))
        ax.plot([y, d + y], [0.02, h], "-o", color=c)
        offset = r["pressure_offsets"][side]
        cop = y + offset["y_offset_m"]
        load = r["forces_n"][side]
        ax.arrow(
            cop,
            0,
            0,
            0.15 * load / w,
            width=0.0015,
            head_width=0.008,
            color=c,
            length_includes_head=True,
        )
        ax.text(y, 0.18, f"{side}: {load:.2f} N", ha="center", fontsize=8)
    ax.plot([com[1], com[1]], [0, com[2]], "--", color="#555")
    ax.scatter([com[1]], [com[2]], marker="x", s=70, c="black", label="COM")
    ax.plot([d - 0.085, d + 0.085], [h, h], color="#bd4338", lw=6, label="Pelvis")
    ax.axhline(0, color="#777")
    ax.set(
        xlim=(-0.19, 0.24),
        ylim=(-0.02, 0.65),
        aspect="equal",
        xlabel="World Y (m)",
        ylabel="World Z (m)",
        title="75/25 candidate: schematic, not a commanded pose",
    )
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(folder / "comparison.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    import argparse

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("folder", type=Path)
    a = p.parse_args()
    plot(json.loads((a.folder / "search.json").read_text()), a.folder)
