#!/usr/bin/env python3
"""Compare actuator budgets with mass-aware static and standing evidence."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("standing", type=Path)
    p.add_argument("static_search", type=Path)
    a = p.parse_args()
    s = json.loads((a.standing / "summary.json").read_text())
    r = json.loads((a.static_search / "search.json").read_text())["best"]["0.0"]
    if not s["passed"] or s["actuator_profile"]["name"] != "robotis_p54":
        raise ValueError("Passing P54 standing evidence required")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    x = np.arange(5)
    names = ["Hip roll", "Hip pitch", "Knee", "Ankle pitch", "Ankle roll"]
    caps = s["plan"]["motor_caps_nm"][:5]
    axes[0].bar(x - 0.18, [6, 6, 10, 6, 6], 0.36, label="Legacy budget", color="#888")
    axes[0].bar(x + 0.18, caps, 0.36, label="P54 continuous cap", color="#167d9a")
    axes[0].set(title="Output torque budgets per leg", ylabel="Nm")
    axes[0].set_xticks(x, names, rotation=40, ha="right")
    axes[0].legend(fontsize=8)
    joints = s["plan"]["joint_names"][:5]
    axes[1].bar(x, caps, color="#167d9a", alpha=0.3, label="New continuous cap")
    axes[1].bar(
        x,
        [abs(r["torques_nm"][n]) for n in joints],
        color="#c77a22",
        width=0.5,
        label="Predicted zero-load endpoint",
    )
    axes[1].set(title="Static prediction includes 7.86 kg added mass", ylabel="Nm")
    axes[1].set_xticks(x, names, rotation=40, ha="right")
    axes[1].legend(fontsize=8)
    start = s["pose"]["first"]["sim_time"]
    end = s["pose"]["last"]["sim_time"]
    rows = [
        json.loads(l) for l in (a.standing / "joints.jsonl").read_text().splitlines()
    ]
    rows = [r for r in rows if start <= r["sim_time"] <= end]
    axes[2].plot(
        [r["sim_time"] - start for r in rows],
        [1000 * max(abs(e) for e in r["error"]) for r in rows],
        color="#167d9a",
    )
    axes[2].axhline(2, color="#888", ls="--", label="Standing error limit")
    axes[2].set(
        title="Measured new-profile standing",
        xlabel="Simulation seconds after audit start",
        ylabel="Largest joint error (mrad)",
        ylim=(0, 2.2),
    )
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.grid(axis="y", alpha=0.2)
    fig.suptitle(
        "Actuator upgrade · 14.42 → 22.28 kg · static predictions are separate from measured standing"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(a.standing / "actuator-comparison.png", dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    main()
