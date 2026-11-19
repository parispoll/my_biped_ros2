#!/usr/bin/env python3
"""Repeat two nearby pulses with fixed damped feedback and preserve guard stops."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import signal
import subprocess
from scripts.studies.run_duration_comparison import ROOT, source_snapshot, compare
from scripts.studies.run_force_repeat_check import assess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int, choices=range(101), default=75)
    args = parser.parse_args()
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    snapshot = source_snapshot()
    report = dict(
        completed=False,
        source_hashes=snapshot,
        cases=[],
        controller=dict(ankle_correction_cap_nm=1.5, balance_kp=20, balance_kd=8),
        protocol="Fresh paired reference/push; full measured settling; 0.25x; 1 ms physics; 10 s recovery.",
        note="Vary only force (27 N / .20 s) or duration (25 N / .22 s) from the prior 25 N / .20 s result. Two repeats in each direction. Continue physical guard outcomes; stop infrastructure failures. No falling or stability boundary claim.",
    )

    def save():
        (folder / "study.json").write_text(json.dumps(report, indent=2) + "\n")

    save()
    try:
        for label, magnitude, duration in [("force", 27, 0.2), ("duration", 25, 0.22)]:
            for repeat, directions in [(1, [1, -1]), (2, [-1, 1])]:
                for sign in directions:
                    if source_snapshot() != snapshot:
                        raise RuntimeError("Robot package changed during study")
                    direction = "forward" if sign > 0 else "backward"
                    name = f"{label}-{direction}-{repeat}"
                    case = folder / name
                    command = [
                        sys.executable,
                        str(ROOT / "scripts/experiments/run_effort_push.py"),
                        "--force-x",
                        str(sign * magnitude),
                        "--duration",
                        str(duration),
                        "--balance-enabled",
                        "--ankle-balance-limit",
                        "1.5",
                        "--balance-kd",
                        "8.0",
                        "--observe-com",
                        "--damping-model",
                        "explicit",
                        "--controller-mode",
                        "pid",
                        "--real-time-factor",
                        ".25",
                        "--ros-domain-id",
                        str(args.ros_domain_id),
                        "--output",
                        str(case),
                    ]
                    print("[neighborhood] Starting " + name, flush=True)
                    with (folder / f"{name}.log").open("w") as log:
                        process = subprocess.Popen(
                            command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT
                        )
                        try:
                            code = process.wait()
                        except BaseException:
                            process.send_signal(signal.SIGINT)
                            try:
                                process.wait(timeout=25)
                            except subprocess.TimeoutExpired:
                                process.kill()
                                process.wait()
                            raise
                    summary = (
                        json.loads((case / "summary.json").read_text())
                        if (case / "summary.json").exists()
                        else {}
                    )
                    launch = (
                        (case / "push/launch.log").read_text()
                        if (case / "push/launch.log").exists()
                        else ""
                    )
                    events = (
                        json.loads((case / "push/recording/events.json").read_text())
                        if (case / "push/recording/events.json").exists()
                        else []
                    )
                    outcome = assess(summary, code, launch, events)
                    row = dict(
                        name=name,
                        variable=label,
                        force_x_n=sign * magnitude,
                        duration_s=duration,
                        requested_impulse_n_s=magnitude * duration,
                        repeat=repeat,
                        direction=direction,
                        path=str(case),
                        returncode=code,
                        outcome=outcome,
                        events=events,
                    )
                    if summary.get("completed"):
                        row["response"] = compare([(duration, summary)])[0]
                    else:
                        row["error"] = summary.get("error", "No summary")
                    report["cases"].append(row)
                    save()
                    print("[neighborhood] " + json.dumps(row), flush=True)
                    if outcome == "incomplete_requires_diagnosis":
                        report["outcome"] = "stopped_for_diagnosis"
                        save()
                        return
        if source_snapshot() != snapshot:
            raise RuntimeError("Robot package changed during final case")
        report.update(completed=True, outcome="all_planned_cases_classified")
        save()
    except BaseException as error:
        report["error"] = str(error)
        save()
        raise


if __name__ == "__main__":
    main()
