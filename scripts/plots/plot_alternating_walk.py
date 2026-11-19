#!/usr/bin/env python3
"""Plot both feet, clearance, measured load exchange and forward COM progress."""

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
    preview = summary.get("preview_kind") == "alternating_walk"
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
    if preview:
        rows = json.loads((folder / "trajectory.json").read_text())
        t = [r["elapsed_s"] for r in rows]
        for side in ("left", "right"):
            initial = rows[0][side + "_foot_xyz_m"][0]
            axes[0].plot(
                t,
                [(r[side + "_foot_xyz_m"][0] - initial) * 1000 for r in rows],
                label=side + " planned",
            )
            axes[1].plot(
                t,
                [(r[side + "_foot_xyz_m"][2] - 0.045) * 1000 for r in rows],
                label=side + " planned",
            )
            axes[2].plot(
                t,
                [
                    100
                    * r["predicted_force_n"][side]
                    / sum(r["predicted_force_n"].values())
                    for r in rows
                ],
                label=side + " predicted",
            )
        axes[3].plot(
            t, [1000 * r["pelvis_x_m"] for r in rows], label="Pelvis X planned"
        )
        axes[3].plot(
            t, [1000 * r["pelvis_y_m"] for r in rows], label="Pelvis Y planned"
        )
    else:
        a = summary["alternating_walk"]
        start = a["origin_sim_time"]
        end = a["end_sim_time"]
        rows = [json.loads(l) for l in (folder / "com.jsonl").read_text().splitlines()]
        rows = [r for r in rows if start <= r["sim_time"] <= end]
        t = [r["sim_time"] - start for r in rows]
        for side in ("left", "right"):
            initial = sum(c[0] for c in rows[0]["feet"][side]) / 4
            axes[0].plot(
                t,
                [
                    1000 * (sum(c[0] for c in r["feet"][side]) / 4 - initial)
                    for r in rows
                ],
                label=side + " measured",
            )
            axes[1].plot(
                t,
                [1000 * min(c[2] for c in r["feet"][side]) for r in rows],
                label=side + " measured",
            )
        from my_robot_description.motion.zero_load import paired_contact_states

        raw = [
            json.loads(l)
            for l in (folder / "contact-wrenches.jsonl").read_text().splitlines()
        ]
        pairs = paired_contact_states(raw, start, end)
        for side in ("left", "right"):
            axes[2].plot(
                [r["sim_time"] - start for r in pairs],
                [
                    100 * r[side]["force_world_n"][2] / r["total_vertical_n"]
                    for r in pairs
                ],
                label=side + " measured",
            )
        axes[3].plot(
            t,
            [1000 * (r["com"][0] - rows[0]["com"][0]) for r in rows],
            label="COM X progress measured",
        )
        axes[3].plot(t, [1000 * r["com"][1] for r in rows], label="COM Y measured")
    for ax, label in zip(
        axes,
        [
            "Foot advance (mm)",
            "Sole clearance (mm)",
            "Vertical load (%)",
            "Position (mm)",
        ],
    ):
        ax.set_ylabel(label)
        ax.legend()
        ax.grid(alpha=0.25)
        for step in summary["motion_plan"]["steps"]:
            ax.axvspan(
                step["lift_start"], step["reload_start"], color="#dcecf5", alpha=0.4
            )
    axes[-1].set_xlabel("Simulated seconds after readiness")
    fig.suptitle(
        ("OFFLINE prediction" if preview else "MEASURED alternating gait")
        + f" — {summary['motion_plan']['step_count']} placements / 30 mm clearance"
    )
    fig.tight_layout()
    fig.savefig(folder / "response.png", dpi=140)
    plt.close(fig)


if __name__ == "__main__":
    plot(Path(sys.argv[1]).resolve())
