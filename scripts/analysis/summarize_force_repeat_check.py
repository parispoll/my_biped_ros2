#!/usr/bin/env python3
"""Verify fixed-pulse repeats and plot the observed, possibly partial response."""

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
from scripts.analysis.analyze_force_response import analyze, rows
from scripts.studies.run_duration_comparison import source_snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    folder = parser.parse_args().folder
    report = json.loads((folder / "repeat-study.json").read_text())
    if not report.get("completed"):
        raise ValueError("Repeat runner has not finished all requested cases")
    assessment = dict(
        source_unchanged=source_snapshot() == report["source_hashes"],
        cases=[],
        interpretation="Two trials per direction. Guard stops are partial records; they do not measure eventual falling or recovery.",
    )
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.8), sharex=True, sharey=True)
    for case in report["cases"]:
        path = Path(case["path"])
        raw = json.loads((path / "summary.json").read_text())
        diagnosis = analyze(path)
        messages = (path / "push/launch.log").read_text()
        events = case["events"]
        start = next(e["sim_time"] for e in events if e["event"] == "push_published")
        clear = next(
            (e["sim_time"] for e in events if e["event"] == "clear_published"), None
        )
        samples = rows(path / "push/recording/samples.jsonl")
        baseline = [r for r in samples if r["phase"] == "baseline"]
        p0 = sum(r["pitch"] for r in baseline) / len(baseline)
        response = [r for r in samples if r["sim_time"] >= start - 0.1]
        controllers = rows(path / "push/controller.jsonl")
        controlled = [
            r for r in controllers if start <= r["sim_time"] <= samples[-1]["sim_time"]
        ]
        item = dict(
            name=case["name"],
            direction=case["direction"],
            repeat=case["repeat"],
            outcome=case["outcome"],
            reference_passed=raw.get("reference", {}).get("passed", False),
            command_window_s=None if clear is None else clear - start,
            gazebo_queue_confirmed="Queueing persistent wrench" in messages,
            gazebo_clear_confirmed="Clearing persistent wrench for entity" in messages,
            tilt_guard_confirmed="Pelvis tilt exceeded 0.15 rad" in messages,
            saturated_samples_by_joint=[
                sum(abs(r["requested"][i] - r["command"][i]) > 1e-9 for r in controlled)
                for i in range(6)
            ],
            **diagnosis,
        )
        assessment["cases"].append(item)
        ax = axes[0 if case["direction"] == "forward" else 1]
        ax.plot(
            [r["sim_time"] - start for r in response],
            [abs(r["pitch"]) * 180 / math.pi for r in response],
            label=f'Repeat {case["repeat"]} · {case["outcome"].replace("posture_guard_stopped_partial_trial","guard stop")}',
            linestyle="-" if case["repeat"] == 1 else "--",
        )
        ax.plot(
            response[-1]["sim_time"] - start,
            abs(response[-1]["pitch"]) * 180 / math.pi,
            "o",
            markersize=4,
        )
    for ax, direction in zip(axes, ("Forward", "Backward")):
        ax.axhline(
            math.degrees(0.15),
            color="#a33",
            linestyle=":",
            label="Controller tilt guard (8.59°)",
        )
        ax.axvspan(
            0,
            report["duration_s"],
            alpha=0.12,
            color="#d99a42",
            label="Requested pulse window",
        )
        ax.set(
            title=direction,
            xlabel="Simulated seconds after push publication",
            ylabel="Absolute recorded pelvis pitch (degrees)",
        )
        ax.set_ylim(bottom=0)
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
    assessment["all_references_passed"] = all(
        c["reference_passed"] for c in assessment["cases"]
    )
    assessment["all_pulses_queued_and_cleared"] = all(
        c["gazebo_queue_confirmed"] and c["gazebo_clear_confirmed"]
        for c in assessment["cases"]
    )
    assessment["guard_stopped_cases"] = sum(
        c["tilt_guard_confirmed"] for c in assessment["cases"]
    )
    fig.suptitle(
        f'{report["force_n"]} N × {report["duration_s"]:.2f} s · two repeats per direction · observed partial traces'
    )
    fig.text(
        0.5,
        0.02,
        "Trace ends are recording cutoffs, not measured peaks or completed recovery. IMU/transport callbacks may record beyond the guard crossing.",
        ha="center",
        fontsize=8,
    )
    fig.tight_layout(rect=(0, 0.06, 1, 0.94))
    fig.savefig(folder / "repeat-response.png", dpi=160)
    plt.close(fig)
    (folder / "assessment.json").write_text(json.dumps(assessment, indent=2) + "\n")
    print(json.dumps({k: v for k, v in assessment.items() if k != "cases"}, indent=2))


if __name__ == "__main__":
    main()
