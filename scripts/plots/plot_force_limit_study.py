#!/usr/bin/env python3
"""Plot completed force-study cases; omitted/incomplete cases are not passes."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scripts.analysis.analyze_force_response import analyze
from scripts.studies.run_duration_comparison import source_snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    folder = parser.parse_args().folder
    study = json.loads((folder / "study.json").read_text())
    fig, axes = plt.subplots(1, 3, figsize=(13, 4.6))
    for direction, color in (("forward", "#167d9a"), ("backward", "#c65442")):
        records = [
            r
            for r in study["cases"]
            if r["name"].startswith(direction) and "response" in r
        ]
        x = [abs(r["force_x_n"]) for r in records]
        diagnostic = [analyze(Path(r["path"])) for r in records]
        values = (
            [r["response"]["peak_pitch_deg"] for r in records],
            [r["response"]["last_outside_band_after_clear_s"] for r in records],
            [100 * d["balance_clipped_fraction"] for d in diagnostic],
        )
        for ax, y in zip(axes, values):
            ax.plot(x, y, "o-", label=direction.capitalize(), color=color)
    for ax, title, ylabel in zip(
        axes,
        (
            "Peak pelvis pitch change",
            "Last outside return bands",
            "Ankle balance correction clipped",
        ),
        (
            "Degrees",
            "Seconds after force clear",
            "% of recorded response controller samples",
        ),
    ):
        ax.set(title=title, xlabel="Requested force magnitude (N)", ylabel=ylabel)
        ax.set_ylim(bottom=0)
        ax.grid(alpha=0.2)
        ax.legend()
    fig.suptitle(
        "0.20 s sagittal pushes · joint PID + IMU · unchanged model and motor limits"
    )
    fig.text(
        0.5,
        0.015,
        f'One fresh reference/push pair per point. {sum(r["outcome"]=="recovered" for r in study["cases"])} / {len(study["cases"])} cases recovered. No repeats or stability-boundary claim.',
        ha="center",
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0.04, 1, 0.94))
    fig.savefig(folder / "comparison.png", dpi=160)
    plt.close(fig)
    (folder / "verification.json").write_text(
        json.dumps(
            dict(
                source_unchanged=source_snapshot() == study["source_hashes"],
                cases=len(study["cases"]),
                all_cases_passed=all(
                    r["outcome"] == "recovered" for r in study["cases"]
                ),
            ),
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
