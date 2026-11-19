#!/usr/bin/env python3
"""Visual audit of recorded ankle commands against a conditional static model."""

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
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    reports = json.loads(args.report.read_text())
    r = reports[-1]["windows"]["final_second"]
    t = r["ankle_terms"]
    fig, axes = plt.subplots(1, 2, figsize=(12, 5.6))
    fig.patch.set_facecolor("#f5f8fa")
    labels = [
        "Holding\nestimate",
        "Angle\ncorrection",
        "Speed\ndamping",
        "Total\ncommand",
    ]
    values = [t[k] for k in ("feedforward", "proportional", "damping", "command")]
    axes[0].bar(
        labels, values, color=["#a8bdc8", "#167d9a", "#aa7100", "#17324d"], width=0.6
    )
    for i, v in enumerate(values):
        axes[0].text(i, v + 0.018, f"{v:.3f}", ha="center", fontsize=11)
    axes[0].set_ylim(0, 0.8)
    axes[0].set_ylabel("Requested ankle torque (N m)")
    axes[0].set_title("The controller is still requesting correction")
    gain = t["proportional"] / t["error"] if t["error"] else 0.0
    axes[0].text(
        0.04,
        0.9,
        f'Final angle error: {t["error"]:.6f} rad\nKp × error = {gain:g} × {t["error"]:.6f} = {t["proportional"]:.3f} N m',
        transform=axes[0].transAxes,
        fontsize=10,
    )
    for i, report in enumerate(reports):
        end = report["windows"]["final_second"]
        g = end["gravity_torque_per_ankle_nm"]
        f = end["configured_ankle_friction_nm"]
        axes[1].plot(
            [i - 0.28, i + 0.28],
            [g, g],
            color="#597e46",
            lw=3,
            label="Estimated gravity torque" if i == 0 else None,
        )
        axes[1].fill_between(
            [i - 0.28, i + 0.28],
            g - f,
            g + f,
            color="#597e46",
            alpha=0.15,
            label="Gravity ± configured friction" if i == 0 else None,
        )
        axes[1].scatter(
            [i],
            [end["ankle_terms"]["command"]],
            color="#c65442",
            s=75,
            label="Logged torque command" if i == 0 else None,
            zorder=4,
        )
        axes[1].annotate(
            f'Unaccounted: {end["unaccounted_static_moment_per_ankle_nm"]:.3f} N m',
            (i, end["ankle_terms"]["command"]),
            xytext=(0, 16),
            textcoords="offset points",
            ha="center",
            fontsize=9,
        )
    axes[1].set_xticks(
        range(len(reports)), [f"Your run {i+1}" for i in range(len(reports))]
    )
    axes[1].set_ylabel("Per-ankle torque (N m)")
    axes[1].set_ylim(-0.05, 0.85)
    axes[1].set_title("Gravity + a simple friction bound do not close")
    axes[1].legend(loc="lower center", fontsize=8)
    for ax in axes:
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(axis="y", alpha=0.2)
        ax.set_axisbelow(True)
    fig.suptitle(
        "Ankle diagnosis | logged correction is present; physical torque balance remains unresolved",
        fontsize=14,
        fontweight="bold",
        color="#17324d",
    )
    fig.text(
        0.5,
        0.025,
        "Static estimate assumes symmetric planar rest and no external wrench. The residual is not a measured friction torque.",
        ha="center",
        fontsize=9,
        color="#647586",
    )
    fig.tight_layout(rect=(0, 0.06, 1, 0.91))
    output = args.report.with_name("ankle-audit.png")
    fig.savefig(output, dpi=160)
    plt.close(fig)
    print(output.resolve())


if __name__ == "__main__":
    main()
