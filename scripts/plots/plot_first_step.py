#!/usr/bin/env python3
"""Plot planned or independently measured single forward placement."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import json, sys
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plot(folder):
    summary = json.loads((folder / "summary.json").read_text())
    preview = summary.get("preview_kind") == "first_step"
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
    if preview:
        rows = json.loads((folder / "trajectory.json").read_text())
        t = [r["elapsed_s"] for r in rows]
        axes[0].plot(
            t, [1000 * r["swing_advance_m"] for r in rows], label="Planned foot advance"
        )
        axes[1].plot(
            t,
            [1000 * r["swing_clearance_m"] for r in rows],
            label="Planned sole clearance",
        )
        for s in ("left", "right"):
            axes[2].plot(
                t,
                [
                    100
                    * r["predicted_force_n"][s]
                    / sum(r["predicted_force_n"].values())
                    for r in rows
                ],
                label=s + " predicted",
            )
        axes[3].plot(
            t, [1000 * r["pelvis_x_m"] for r in rows], label="Pelvis X planned"
        )
        axes[3].plot(
            t, [1000 * r.get("pelvis_y_m", 0) for r in rows], label="Pelvis Y planned"
        )
    else:
        audit = summary["first_step"]
        start = audit["origin_sim_time"]
        swing = audit["swing_foot"]
        support = audit["supporting_foot"]
        com = [json.loads(l) for l in (folder / "com.jsonl").read_text().splitlines()]
        com = [r for r in com if start <= r["sim_time"] <= audit["end_sim_time"]]
        t = [r["sim_time"] - start for r in com]
        x0 = sum(c[0] for c in com[0]["feet"][swing]) / 4
        axes[0].plot(
            t,
            [1000 * (sum(c[0] for c in r["feet"][swing]) / 4 - x0) for r in com],
            label=swing + " foot measured",
        )
        axes[1].plot(
            t,
            [1000 * min(c[2] for c in r["feet"][swing]) for r in com],
            label="Lowest sole corner measured",
        )
        from my_robot_description.motion.zero_load import paired_contact_states

        raw = [
            json.loads(l)
            for l in (folder / "contact-wrenches.jsonl").read_text().splitlines()
        ]
        pairs = paired_contact_states(raw, start, audit["end_sim_time"])
        for s in ("left", "right"):
            axes[2].plot(
                [r["sim_time"] - start for r in pairs],
                [100 * r[s]["force_world_n"][2] / r["total_vertical_n"] for r in pairs],
                label=s + " measured",
            )
        axes[3].plot(t, [1000 * r["com"][0] for r in com], label="COM X measured")
        axes[3].plot(t, [1000 * r["com"][1] for r in com], label="COM Y measured")
    for ax, y in zip(
        axes,
        ["Foot advance (mm)", "Sole height (mm)", "Vertical load (%)", "Position (mm)"],
    ):
        ax.set_ylabel(y)
        ax.grid(alpha=0.25)
        ax.legend(loc="best")
        ax.axvline(117, color="gray", ls=":")
        ax.axvline(169, color="gray", ls=":")
        ax.axvline(233, color="gray", ls=":")
    axes[-1].set_xlabel("Simulated seconds after readiness")
    fig.suptitle(
        ("OFFLINE prediction" if preview else "MEASURED first forward step")
        + " — 50 mm advance / 30 mm lift / staggered reload"
    )
    fig.tight_layout()
    fig.savefig(folder / "response.png", dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    plot(Path(sys.argv[1]).resolve())
