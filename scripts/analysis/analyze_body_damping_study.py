#!/usr/bin/env python3
"""Compare full gain-4/gain-8 responses and direction-aware rebound metrics."""

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
from scripts.studies.run_duration_comparison import source_snapshot
from scripts.analysis.body_damping_metrics import response_metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    folder = parser.parse_args().folder
    study = json.loads((folder / "study.json").read_text())
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    assessment = dict(
        completed=study.get("completed", False),
        source_unchanged=source_snapshot() == study["source_hashes"],
        cases=[],
    )
    for case in study["cases"]:
        p = Path(case["path"])
        samples = rows(p / "push/recording/samples.jsonl")
        events = json.loads((p / "push/recording/events.json").read_text())
        start = next(e["sim_time"] for e in events if e["event"] == "push_published")
        clear = next(
            (e["sim_time"] for e in events if e["event"] == "clear_published"), None
        )
        end = samples[-1]["sim_time"]
        ctrl = [
            r
            for r in rows(p / "push/controller.jsonl")
            if start <= r["sim_time"] <= end
        ]
        com = [
            r for r in rows(p / "push/com.jsonl") if start - 1 <= r["sim_time"] <= end
        ]
        foot = (
            lambda r: sum(c[0] for corners in r["feet"].values() for c in corners) / 8
        )
        baseline = [r for r in com if r["sim_time"] < start]
        x0 = sum(r["com"][0] - foot(r) for r in baseline) / len(baseline)
        velocity = [
            ((b["com"][0] - foot(b)) - (a["com"][0] - foot(a)))
            / (b["sim_time"] - a["sim_time"])
            for a, b in zip(com, com[2:])
        ]
        diagnostic = analyze(p)
        diagnostic.pop("note", None)
        summary = json.loads((p / "push/recording/summary.json").read_text())
        metrics = response_metrics(samples, summary, case["force_x_n"])
        diagnostic.update(metrics)
        item = dict(
            name=case["name"],
            force_x_n=case["force_x_n"],
            correction_cap_nm=case["ankle_balance_cap_nm"],
            balance_kd=case["balance_kd_nm_s_per_rad"],
            outcome=case["outcome"],
            **diagnostic,
            peak_com_offset_change_cm=max(
                abs(r["com"][0] - foot(r) - x0) * 100 for r in com
            ),
            peak_estimated_com_fore_aft_speed_m_s=max(abs(v) for v in velocity),
            note="Completed measured response; COM velocity is a finite-difference estimate."
        )
        assessment["cases"].append(item)
        # Forward baseline and first forward candidate share axes for a clear A/B view.
        if case["name"] not in ("baseline-forward", "candidate-forward-1"):
            continue
        original = case["name"].startswith("baseline")
        label = (
            "Pitch-rate gain 4 · measured baseline"
            if original
            else "Pitch-rate gain 8 · measured candidate"
        )
        color = "#c65442" if original else "#167d9a"
        display = [r for r in samples if start - 0.2 <= r["sim_time"] <= start + 4]
        axes[0, 0].plot(
            [r["sim_time"] - start for r in display],
            [r["pitch"] * 180 / math.pi for r in display],
            label=label,
            color=color,
        )

        display = [r for r in com if start - 0.2 <= r["sim_time"] <= start + 4]
        axes[0, 1].plot(
            [r["sim_time"] - start for r in display],
            [(r["com"][0] - foot(r) - x0) * 100 for r in display],
            color=color,
            label=label,
        )
        display = [r for r in ctrl if r["sim_time"] <= start + 4]
        axes[1, 0].plot(
            [r["sim_time"] - start for r in display],
            [r["command"][2] for r in display],
            color=color,
            label=label,
        )
        axes[1, 1].plot(
            [r["sim_time"] - start for r in display],
            [r["balance_nm"][2] for r in display],
            color=color,
            label=label,
        )
    axes[0, 0].axhline(
        math.degrees(0.15), color="#777", ls=":", label="Existing tilt guard"
    )
    for ax, title, ylabel in zip(
        axes.flat,
        (
            "Pelvis pitch",
            "COM fore/aft motion relative to feet",
            "Left ankle motor command",
            "Left ankle IMU correction",
        ),
        (
            "Degrees",
            "Change from baseline (cm)",
            "N·m · existing limit ±6",
            "N·m · same correction cap ±1.5",
        ),
    ):
        ax.set(
            title=title,
            ylabel=ylabel,
            xlabel="Simulation seconds after push publication",
            xlim=(-0.2, 4),
        )
        ax.axvspan(0, 0.2, color="#d99a42", alpha=0.12)
        ax.grid(alpha=0.2)
    axes[0, 0].legend(fontsize=7, loc="upper right")
    fig.suptitle(
        "25 N × 0.20 s · pitch-rate gain 4 → 8 · same correction cap, model and motor limits"
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(folder / "comparison.png", dpi=150)
    plt.close(fig)
    (folder / "assessment.json").write_text(json.dumps(assessment, indent=2) + "\n")


if __name__ == "__main__":
    main()
