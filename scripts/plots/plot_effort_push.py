#!/usr/bin/env python3
"""Plot a no-push reference beside a gentle effort-mode disturbance."""

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
        path = args.folder / name / "recording"
        summary = json.loads((path / "summary.json").read_text())
        rows = [
            json.loads(s) for s in (path / "samples.jsonl").read_text().splitlines()
        ]
        start = next(
            e["sim_time"] for e in summary["events"] if e["event"] == "push_published"
        )
        data[name] = (summary, rows, [s["sim_time"] - start for s in rows])
    summary, rows, t = data["push"]
    reference, control, ct = data["reference"]
    window = summary["command_window_s"]
    degrees = 180 / math.pi
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.8), sharex=True)
    fig.patch.set_facecolor("#f5f8fa")
    force = summary["requested"]["force_x"]
    axes[0, 0].step(
        t,
        [force if s["phase"] == "push" else 0 for s in rows],
        where="post",
        color="#aa7100",
        lw=2,
    )
    axes[0, 0].set_title("External force command")
    axes[0, 0].set_ylabel("Requested force X (N)")
    handling = summary.get("force_handling", {})
    confirmation = (
        "Gazebo queue + clear confirmed"
        if handling.get("queued_in_gazebo") and handling.get("cleared_in_gazebo")
        else "Publication timing only; no debug confirmation"
    )
    axes[0, 0].text(
        0.35,
        0.78,
        f'Request: {force:g} N for {summary["requested"]["duration"]:g} s\nPublished window: {window:.3f} s\n{confirmation}',
        transform=axes[0, 0].transAxes,
        fontsize=9,
    )
    axes[0, 1].plot(
        t,
        [(s["pitch"] - summary["baseline_pitch_rad"]) * degrees for s in rows],
        color="#167d9a",
        lw=2,
        label=f"{force:g} N push",
    )
    axes[0, 1].plot(
        ct,
        [(s["pitch"] - reference["baseline_pitch_rad"]) * degrees for s in control],
        color="#7a8791",
        ls="--",
        label="0 N reference",
    )
    band = summary["return_band"]["pitch_rad"] * degrees
    axes[0, 1].axhspan(
        -band, band, color="#597e46", alpha=0.13, label="Return band ±0.1°"
    )
    axes[0, 1].set_title("Pelvis pitch change from its own baseline")
    axes[0, 1].set_ylabel("Pitch change (degrees)")
    axes[0, 1].legend(fontsize=8, loc="lower right")
    q0 = summary["baseline_joint_positions_rad"][2]
    axes[1, 0].plot(
        t,
        [(s["actual_joints"][2] - q0) * degrees for s in rows],
        color="#c65442",
        lw=2,
        label="Left ankle (right matches)",
    )
    qband = summary["return_band"]["joint_change_rad"] * degrees
    axes[1, 0].axhspan(
        -qband, qband, color="#597e46", alpha=0.13, label="Joint return band"
    )
    axes[1, 0].set_title("Ankle angle change from baseline")
    axes[1, 0].set_ylabel("Angle change (degrees)")
    axes[1, 0].legend(fontsize=8, loc="lower right")
    for i, (name, color) in enumerate(
        zip(("Hip", "Knee", "Ankle"), ("#597e46", "#aa7100", "#167d9a"))
    ):
        axes[1, 1].plot(
            t, [s["joint_command_nm"][i] for s in rows], label=name, color=color, lw=2
        )
    axes[1, 1].set_title("Left-leg torque commands")
    axes[1, 1].set_ylabel("Requested joint torque (N m)")
    axes[1, 1].legend(fontsize=8, loc="center right")
    for ax in axes.flat:
        ax.axvspan(0, window, color="#aa7100", alpha=0.12)
        ax.axvline(window, color="#aa7100", ls=":", lw=1)
        ax.grid(alpha=0.18)
        ax.spines[["top", "right"]].set_visible(False)
    for ax in axes[1]:
        ax.set_xlabel("Simulation seconds since push publication")
    verdict = (
        "returned within bands"
        if summary["passed"]
        else "stayed upright; did not return within bands"
    )
    fig.suptitle(
        f"{force:g} N effort-mode push | {verdict}",
        fontsize=16,
        fontweight="bold",
        color="#17324d",
    )
    fig.text(
        0.5,
        0.015,
        "Shaded vertical strip: published push window. Torque traces are commands. Force magnitude and exact actuation duration are not independently measured.",
        ha="center",
        fontsize=8,
        color="#647586",
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    output = args.folder / "response.png"
    fig.savefig(output, dpi=160)
    plt.close(fig)
    print(output.resolve())


if __name__ == "__main__":
    main()
