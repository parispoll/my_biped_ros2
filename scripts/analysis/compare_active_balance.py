#!/usr/bin/env python3
"""Compare nominally matched off/on push recordings; report measured differences."""

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
from my_robot_description.validation.balance_audit import audit_balance


def load(folder, active):
    s = json.loads((folder / "summary.json").read_text())
    if not s["completed"]:
        raise ValueError("Incomplete experiment")
    p = s["push"]
    first = p["events"][0]["sim_time"]
    last = p["events"][-1]["sim_time"] + p["requested"]["recovery"]
    c = [
        json.loads(x)
        for x in (folder / "push/controller.jsonl").read_text().splitlines()
    ]
    c = [r for r in c if r["active"] and first <= r["sim_time"] <= last]
    obs = [
        json.loads(x) for x in (folder / "push/balance.jsonl").read_text().splitlines()
    ]
    audit = audit_balance(obs, c, first, last, active=active)
    if not audit["passed"]:
        raise ValueError("Command/gate audit failed")
    r = [
        json.loads(x)
        for x in (folder / "push/recording/samples.jsonl").read_text().splitlines()
    ]
    return s, c, r, audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--off", type=Path, required=True)
    parser.add_argument("--on", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    off = load(args.off, False)
    on = load(args.on, True)
    a, b = off[0]["push"], on[0]["push"]
    if a["requested"] != b["requested"]:
        raise ValueError("Push protocols differ")
    spread = abs(a["command_window_s"] - b["command_window_s"]) / (
        (a["command_window_s"] + b["command_window_s"]) / 2
    )
    rows = []
    for label, data in (("off", off), ("on", on)):
        root, c, samples, audit = data
        p = root["push"]
        row = dict(
            mode=label,
            path=str((args.off if label == "off" else args.on).resolve()),
            recovery_passed=root["passed"],
            peak_pitch_deg=math.degrees(p["peak_pitch_change_rad"]),
            peak_pitch_rate_rad_s=p["peak_pitch_rate_rad_s"],
            last_outside_band_after_clear_s=p["last_outside_band_after_clear_s"],
            final_pitch_deg=math.degrees(p["final_pitch_change_rad"]),
            final_joint_offset_rad=p["final_joint_change_from_baseline_rad"],
            contacts_passed=p["continuous_contact_evidence"],
            saturated_samples=p["saturated_controller_samples"],
            peak_extra_balance_command_nm=audit["peak_requested_balance_nm"],
            audit=audit,
        )
        rows.append(row)
    reduction = 100 * (1 - rows[1]["peak_pitch_deg"] / rows[0]["peak_pitch_deg"])
    eligible = spread <= 0.05 and all(
        r["recovery_passed"] and r["contacts_passed"] and r["saturated_samples"] == 0
        for r in rows
    )
    report = dict(
        completed=True,
        rows=rows,
        requested=a["requested"],
        published_window_relative_difference=spread,
        peak_pitch_reduction_percent=reduction,
        valid_comparison=eligible,
        peak_lean_improvement_observed=eligible and reduction >= 5,
        limitation="One off/on pair. Matched requested force/published windows and Gazebo handling, not independently matched measured impulses; not a general stability or robustness claim.",
    )
    (args.output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    fig.patch.set_facecolor("#f5f8fa")
    for label, data, color in (
        ("Joint PID only", off, "#b36b42"),
        ("Joint PID + IMU correction", on, "#147e96"),
    ):
        root, c, r, audit = data
        p = root["push"]
        origin = next(
            e["sim_time"] for e in p["events"] if e["event"] == "push_published"
        )
        t = [x["sim_time"] - origin for x in r]
        ct = [x["sim_time"] - origin for x in c]
        axes[0, 0].plot(
            t,
            [math.degrees(x["pitch"] - p["baseline_pitch_rad"]) for x in r],
            label=label,
            color=color,
        )
        axes[0, 1].plot(t, [x["pitch_rate"] for x in r], label=label, color=color)
        axes[1, 0].plot(
            ct, [x.get("balance_nm", [0.0] * 6)[2] for x in c], label=label, color=color
        )
        axes[1, 1].plot(
            t, [x["joint_command_nm"][2] for x in r], label=label, color=color
        )
    axes[0, 0].axhspan(-0.1, 0.1, color="#638b50", alpha=0.12)
    axes[0, 0].set(title="Pelvis pitch change from own baseline", ylabel="Degrees")
    axes[0, 1].set(title="Pelvis pitch rate", ylabel="rad/s")
    axes[1, 0].set(
        title="Additional IMU command: left ankle",
        ylabel="Requested balance contribution (N m)",
    )
    axes[1, 1].set(title="Total left ankle motor command", ylabel="Command (N m)")
    for ax in axes.flat:
        ax.axvspan(0, b["command_window_s"], color="#b36b42", alpha=0.12)
        ax.grid(alpha=0.18)
        ax.spines[["top", "right"]].set_visible(False)
        ax.legend(fontsize=8, loc="lower right")
    for ax in axes[1]:
        ax.set_xlabel("Simulated seconds since push publication")
    fig.suptitle(
        f"{b['requested']['force_x']:+g} N push | peak pitch reduction {reduction:.1f}% in this pair",
        fontsize=16,
        fontweight="bold",
    )
    fig.text(
        0.5,
        0.018,
        "Separate fresh simulations, same requested pulse. Extra balance request is added before the unchanged total motor clamp. Torque is commanded, not independently measured.",
        ha="center",
        fontsize=8,
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    fig.savefig(args.output / "comparison.png", dpi=160)
    plt.close(fig)
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
