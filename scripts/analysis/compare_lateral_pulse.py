#!/usr/bin/env python3
"""Compare a matched P54 reference with an externally applied lateral pulse."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import numpy as np


def load(folder):
    s = json.loads((folder / "summary.json").read_text())
    if not s["passed"] or not s["source_unchanged"]:
        raise ValueError("Original trial checks must pass")
    start = s["shift"]["origin_sim_time"]
    end = s["shift"]["end_sim_time"]
    rows = [json.loads(l) for l in (folder / "imu-raw.jsonl").read_text().splitlines()]
    rows = [r for r in rows if start <= r["sim_time"] <= end]
    return (
        s,
        np.array([r["sim_time"] - start for r in rows]),
        np.array([r["acceleration"][1] for r in rows]),
    )


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("reference", type=Path)
    a.add_argument("push", type=Path)
    a.add_argument("--output", type=Path, required=True)
    args = a.parse_args()
    ref, t0, y0 = load(args.reference)
    push, t1, y1 = load(args.push)
    if ref.get("physical_pulse") or not push.get("physical_pulse", {}).get("passed"):
        raise ValueError("Expected a no-push reference and a verified physical pulse")
    for key in ["actuator_profile", "unloading_plan", "real_time_factor"]:
        if ref[key] != push[key]:
            raise ValueError("Reference model/motion/controller mismatch")
    if (args.reference / "joints.gains.json").read_bytes() != (
        args.push / "joints.gains.json"
    ).read_bytes():
        raise ValueError("Inner gains differ")
    if json.loads((args.reference / "source-hashes.json").read_text()) != json.loads(
        (args.push / "source-hashes.json").read_text()
    ):
        raise ValueError("Package sources differ")
    pulse = push["physical_pulse"]
    start = push["shift"]["origin_sim_time"]
    on = pulse["events"][0]["sim_time"] - start
    off = pulse["events"][1]["sim_time"] - start
    residual = y1 - np.interp(t1, t0, y0)
    window = (t1 >= on - 0.02) & (t1 <= off + 1.0)
    peak = float(max(abs(residual[window])))
    response = peak > 0.005
    visual = pulse.get("visual")
    visual_ok = visual is None or (
        visual["service_seen"]
        and visual["updates_accepted"] > 0
        and visual["clear_accepted"]
    )
    report = dict(
        passed=bool(response and visual_ok),
        reference=str(args.reference),
        push=str(args.push),
        pulse=pulse,
        peak_incremental_lateral_acceleration_m_s2=peak,
        response_detection_floor_m_s2=0.005,
        response_detected=bool(response),
        visual_marker_verified=visual_ok,
        note="Matched raw IMU response plus Gazebo queue/clear evidence; not a calibrated force or comparison of feedback-on versus feedback-off push rejection.",
    )
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(t0, y0, label="0 N reference", color="#888")
    axes[0].plot(t1, y1, label="Lateral pulse", color="#167d9a")
    axes[0].set(ylabel="Raw IMU lateral acceleration (m/s²)")
    axes[0].legend()
    axes[1].plot(t1, residual, color="#167d9a")
    axes[1].set(
        xlabel="Simulated seconds after readiness",
        ylabel="Push minus reference (m/s²)",
        xlim=(10, 17),
    )
    for ax in axes:
        ax.axvspan(on, off, color="#d2663b", alpha=0.2)
        ax.grid(alpha=0.2)
    fig.suptitle(
        f'P54 external disturbance · {pulse["force_y_n"]:g} N requested for {pulse["requested_duration_s"]:g} s\nOriginal standing/contact/return checks passed; orange: command window'
    )
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(args.output / "comparison.png", dpi=150)
    plt.close(fig)
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
