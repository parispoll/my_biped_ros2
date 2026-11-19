#!/usr/bin/env python3
"""Plot measured bilateral lowering and return, separate from lateral pulses."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import json, sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))
from my_robot_description.sensing.foot_loads import paired_loads
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    folder = Path(sys.argv[1])
    s = json.loads((folder / "summary.json").read_text())
    start = s["shift"]["origin_sim_time"]
    end = s["shift"]["end_sim_time"]
    read = lambda name: [
        json.loads(l) for l in (folder / name).read_text().splitlines()
    ]
    poses = [r["pose"] for r in read("sensors.jsonl") if start <= r["sim_time"] <= end]
    joints = [
        r
        for r in read("joints.jsonl")
        if r.get("shift") is not None and start <= r["sim_time"] <= end
    ]
    lift = "foot_lift" in s
    zero = (
        lift
        or s.get("motion_plan", {}).get("experiment_kind") == "zero_load_transition"
    )
    if zero:
        from my_robot_description.motion.zero_load import paired_contact_states

        pairs = paired_contact_states(read("contact-wrenches.jsonl"), start, end)
    else:
        pairs = paired_loads(read("contact-wrenches.jsonl"), start, end)
    combined = s.get("motion_plan", {}).get("program_kind") == "lowered_shift"
    fig, axes = plt.subplots(
        5 if lift else 4 if combined else 3,
        1,
        figsize=(10, 14 if lift else 12 if combined else 9),
        sharex=True,
    )
    axes[0].plot(
        [r["sim_time"] - start for r in poses],
        [r["position"][2] * 1000 for r in poses],
        label="Measured pelvis",
        color="#087f8c",
    )
    axes[0].plot(
        [r["sim_time"] - start for r in joints],
        [r["shift"]["pelvis_height_m"] * 1000 for r in joints],
        label="Planned pelvis",
        ls="--",
        color="#bc5b2b",
    )
    axes[0].set_ylabel("Height above floor (mm)")
    axes[0].legend()
    if combined:
        axes[1].plot(
            [r["sim_time"] - start for r in poses],
            [r["position"][1] * 1000 for r in poses],
            label="Measured pelvis Y",
        )
        axes[1].plot(
            [r["sim_time"] - start for r in joints],
            [r["shift"]["pelvis_y_m"] * 1000 for r in joints],
            ls="--",
            label="Planned pelvis Y",
        )
        axes[1].set_ylabel("Lateral position (mm)")
        axes[1].legend()
    if lift:
        side = s["foot_lift"]["swing_foot"]
        com = read("com.jsonl")
        cr = [r for r in com if start <= r["sim_time"] <= end]
        axes[3].plot(
            [r["sim_time"] - start for r in cr],
            [min(c[2] for c in r["feet"][side]) * 1000 for r in cr],
            label="Measured minimum swing sole corner",
        )
        axes[3].plot(
            [r["sim_time"] - start for r in joints],
            [r["shift"]["swing_clearance_m"] * 1000 for r in joints],
            ls="--",
            label="Requested clearance",
        )
        axes[3].set_ylabel("Sole clearance (mm)")
        axes[3].legend()
    force_axis = axes[2 if combined else 1]
    speed_axis = axes[-1]
    for side in ["left", "right"]:
        force_axis.plot(
            [r["sim_time"] - start for r in pairs],
            [100 * r[side]["force_world_n"][2] / r["total_vertical_n"] for r in pairs],
            label=side,
        )
    if combined:
        force_axis.plot(
            [r["sim_time"] - start for r in joints],
            [
                100
                * r["shift"]["predicted_force_n"]["left"]
                / (s["plan"]["mass_kg"] * 9.81)
                for r in joints
            ],
            ls="--",
            color="#666",
            label="Planned left share (static)",
        )
    force_axis.set_ylabel("Foot load share (%)")
    force_axis.ticklabel_format(axis="y", useOffset=False)
    force_axis.legend()
    speed_axis.plot(
        [r["sim_time"] - start for r in joints],
        [max(abs(v) for v in r["velocity"]) for r in joints],
    )
    speed_axis.axhline(
        0.05, color="#bc5b2b", ls="--", label="Existing slow-motion audit limit"
    )
    speed_axis.set(
        ylabel="Largest joint speed (rad/s)",
        xlabel="Simulated seconds after standing readiness",
    )
    speed_axis.legend()
    for ax in axes:
        ax.grid(alpha=0.2)
    fig.suptitle(
        (
            f"P54 measured {s['motion_plan']['clearance_m']*1000:g} mm foot lift"
            if lift
            else (
                "P54 zero-load transition"
                if zero
                else (
                    "P54 lowered lateral transfer"
                    if combined
                    else "P54 bilateral lowering and return"
                )
            )
        )
        + " - "
        + ("all checks passed" if s["passed"] else "checks not met")
        + (
            "\nMeasured clearance, contact loss, landing and return"
            if lift
            else (
                "\nFixed-sole reference; no lift commanded"
                if zero
                else "\nBoth feet planted; no foot lift"
            )
        )
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(folder / "response.png", dpi=150)
    plt.close(fig)
    print(folder / "response.png")


if __name__ == "__main__":
    main()
