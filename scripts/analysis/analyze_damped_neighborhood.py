#!/usr/bin/env python3
"""Summarize completed and guarded neighborhood trials without conflating them."""

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
from scripts.analysis.analyze_force_response import rows, analyze
from scripts.analysis.body_damping_metrics import response_metrics
from scripts.studies.run_duration_comparison import ROOT, source_snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    folder = parser.parse_args().folder
    study = json.loads((folder / "study.json").read_text())
    report = dict(
        completed=study["completed"],
        source_unchanged=source_snapshot() == study["source_hashes"],
        cases=[],
        note="Guarded curves are partial, not measured full peaks or failed full recovery assessments. Prior 25 N / .20 s baseline is historical.",
    )
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    for side, sign, row in [("forward", 1, 0), ("backward", -1, 1)]:
        baseline = ROOT / "evidence/body-damping-study-01" / f"candidate-{side}-1"
        s = json.loads((baseline / "summary.json").read_text())["push"]
        start = next(
            e["sim_time"] for e in s["events"] if e["event"] == "push_published"
        )
        for col in range(2):
            data = [
                r
                for r in rows(baseline / "push/recording/samples.jsonl")
                if start - 0.2 <= r["sim_time"] <= start + 4
            ]
            axes[row, col].plot(
                [r["sim_time"] - start for r in data],
                [
                    sign * math.degrees(r["pitch"] - s["baseline_pitch_rad"])
                    for r in data
                ],
                color="#666",
                ls="--",
                label="Prior 25 N / .20 s · recovered",
            )
    for case in study["cases"]:
        p = Path(case["path"])
        summary = json.loads((p / "summary.json").read_text())
        samples = rows(p / "push/recording/samples.jsonl")
        events = case["events"]
        start = next(
            (e["sim_time"] for e in events if e["event"] == "push_published"), None
        )
        item = dict(case)
        item.pop("events", None)
        item["reference_passed"] = summary.get("reference", {}).get("passed", False)
        item["diagnostic"] = analyze(p)
        if summary.get("completed"):
            item["metrics"] = response_metrics(
                samples, summary["push"], case["force_x_n"]
            )
            item["full_recovery_assessment"] = True
        else:
            item["full_recovery_assessment"] = False
            item["partial_trace_warning"] = (
                "Recorded motion may extend beyond guard crossing during transport and shutdown; not a full controlled peak."
            )
        report["cases"].append(item)
        row = 0 if case["direction"] == "forward" else 1
        col = 0 if case["variable"] == "force" else 1
        sign = 1 if case["force_x_n"] > 0 else -1
        baseline = [r for r in samples if r["phase"] == "baseline"]
        pitch0 = sum(r["pitch"] for r in baseline) / len(baseline) if baseline else None
        if start is not None and pitch0 is not None:
            data = [r for r in samples if start - 0.2 <= r["sim_time"] <= start + 4]
            label = f"Repeat {case['repeat']} · " + (
                "recovered"
                if case["outcome"] == "recovered"
                else (
                    "partial guard stop"
                    if case["outcome"] == "posture_guard_stopped_partial_trial"
                    else case["outcome"]
                )
            )
            axes[row, col].plot(
                [r["sim_time"] - start for r in data],
                [sign * math.degrees(r["pitch"] - pitch0) for r in data],
                label=label,
                color=("#167d9a" if case["repeat"] == 1 else "#c65442"),
            )
    for row, side in enumerate(["Forward", "Backward"]):
        for col, label in enumerate(
            ["27 N / .20 s: force increased", "25 N / .22 s: duration increased"]
        ):
            ax = axes[row, col]
            ax.axhline(
                math.degrees(0.15),
                color="#999",
                ls=":",
                label="Tilt guard scale (absolute tilt)",
            )
            ax.set(
                title=f"{side} · {label}",
                xlabel="Simulation seconds after push publication",
                ylabel="Pitch change along push direction (°)",
                xlim=(-0.2, 4),
            )
            ax.grid(alpha=0.2)
            ax.legend(fontsize=7)
    fig.suptitle(
        "Fixed damped controller: nearby force and duration tests\nTraces ending early at a guard are partial; baseline-relative pitch is not absolute tilt."
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(folder / "comparison.png", dpi=150)
    plt.close(fig)
    (folder / "assessment.json").write_text(json.dumps(report, indent=2) + "\n")
    print(
        json.dumps(
            [
                dict(name=r["name"], outcome=r["outcome"], metrics=r.get("metrics"))
                for r in report["cases"]
            ],
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
