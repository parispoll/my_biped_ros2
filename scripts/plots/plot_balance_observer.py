#!/usr/bin/env python3
"""Plot body measurements and hypothetical ankle torque, without applying it."""

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
    args = parser.parse_args()
    data = {}
    for name in ("reference", "push"):
        path = args.folder / name
        s = json.loads((path / "recording/summary.json").read_text())
        if s.get("balance_enabled") or s.get("balance_observer", {}).get(
            "active_balance"
        ):
            parser.error(
                "Balance was enabled in this experiment. Use plot_effort_push.py or compare_active_balance.py; an observer-only label would be incorrect."
            )
        if not s.get("balance_observer", {}).get("passed"):
            raise ValueError("A passing balance-observer audit is required")
        start = next(
            e["sim_time"] for e in s["events"] if e["event"] == "push_published"
        )
        begin = s["events"][0]["sim_time"]
        end = s["events"][-1]["sim_time"] + s["requested"]["recovery"]
        rows = [
            json.loads(x) for x in (path / "balance.jsonl").read_text().splitlines()
        ]
        rows = [r for r in rows if begin <= r["sim_time"] <= end]
        controller = [
            json.loads(x) for x in (path / "controller.jsonl").read_text().splitlines()
        ]
        controller = [r for r in controller if begin <= r["sim_time"] <= end]
        data[name] = (s, rows, controller, start)
    s, rows, controller, start = data["push"]
    t = [r["sim_time"] - start for r in rows]
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.3), sharex=True)
    fig.patch.set_facecolor("#f5f8fa")
    for name, color, label in (
        ("reference", "#81909a", "0 N reference"),
        ("push", "#157d94", f"{s['requested']['force_x']:g} N push"),
    ):
        _, r, _, origin = data[name]
        rt = [x["sim_time"] - origin for x in r]
        axes[0, 0].plot(
            rt, [math.degrees(x["pitch_rad"]) for x in r], color=color, label=label
        )
    axes[0, 0].axhline(0, color="#597e46", ls=":", label="Upright target")
    axes[0, 0].set(
        title="1. IMU: which way is the pelvis leaning?", ylabel="World pitch (degrees)"
    )
    axes[0, 1].plot(
        t,
        [math.degrees(r["pitch_rate_rad_s"]) for r in rows],
        color="#a46e1f",
        label="Pitch rate from body gyro + roll",
    )
    axes[0, 1].set(
        title="2. IMU: is the lean growing or shrinking?",
        ylabel="Pitch rate (degrees/s)",
    )
    for key, scale, label, color in (
        ("proportional_total_nm", 0.5, "P: half of body angle correction", "#597e46"),
        ("derivative_total_nm", 0.5, "D: half of body rate correction", "#a46e1f"),
        ("proposal_per_ankle_nm", 1.0, "Bounded per-ankle proposal", "#157d94"),
    ):
        axes[1, 0].plot(
            t, [r[key] * scale for r in rows], label=label, color=color, lw=1.8
        )
    cap = rows[0]["settings"]["ankle_limit"]
    axes[1, 0].axhline(cap, color="#a8afb4", ls=":")
    axes[1, 0].axhline(
        -cap, color="#a8afb4", ls=":", label=f"Proposal limit ±{cap:g} N m"
    )
    axes[1, 0].set(
        title="3. Calculate a correction, but do not apply it",
        ylabel="Proposed extra torque per ankle (N m)",
    )
    axes[1, 1].plot(
        [r["sim_time"] - start for r in controller],
        [r["command"][2] for r in controller],
        label="Existing left ankle motor command",
        color="#157d94",
    )
    axes[1, 1].plot(
        t,
        [r["proposal_per_ankle_nm"] for r in rows],
        label="Candidate extra correction: NOT applied",
        color="#a46e1f",
        ls="--",
    )
    axes[1, 1].plot(
        t,
        [0.0] * len(t),
        label="Applied balance contribution = 0",
        color="#597e46",
        ls=":",
    )
    axes[1, 1].set(
        title="4. Existing posture controller still drives the robot",
        ylabel="Torque (N m)",
    )
    for ax in axes.flat:
        ax.axvspan(0, s["command_window_s"], color="#a46e1f", alpha=0.12)
        ax.grid(alpha=0.18)
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(fontsize=8, loc="lower right")
    for ax in axes[1]:
        ax.set_xlabel("Simulation seconds since push publication")
    config = rows[0]["settings"]
    fig.suptitle(
        "IMU balance observer | proposals only, no balance torque applied",
        fontsize=16,
        fontweight="bold",
        color="#17324d",
    )
    fig.text(
        0.5,
        0.92,
        f"Candidate body gains: Kp={config['kp']:g} N m/rad, Kd={config['kd']:g} N m s/rad | split equally between two grounded ankles",
        ha="center",
        fontsize=10,
    )
    fig.text(
        0.5,
        0.018,
        "Positive pitch = forward lean. Positive ankle motor torque reacts against it. Existing motor request verified as feedforward + joint P + joint D + joint I.",
        ha="center",
        fontsize=8,
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.91))
    path = args.folder / "balance-observer.png"
    fig.savefig(path, dpi=160)
    plt.close(fig)
    print(path.resolve())


if __name__ == "__main__":
    main()
