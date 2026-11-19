#!/usr/bin/env python3
"""Fresh baseline plus repeated 25 N candidate checks, with read-only COM."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import signal
import subprocess
from scripts.studies.run_duration_comparison import ROOT, source_snapshot
from scripts.studies.run_force_repeat_check import assess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    snapshot = source_snapshot()
    report = dict(
        completed=False,
        source_hashes=snapshot,
        cases=[],
        force_n=25,
        duration_s=0.2,
        change="Only per-ankle IMU correction cap: baseline 0.5, candidate 1.5 N m. Motor caps unchanged. COM observation enabled in all fresh cases.",
    )

    def save():
        (folder / "study.json").write_text(json.dumps(report, indent=2) + "\n")

    save()
    plan = [
        ("baseline-forward", 25, 0.5),
        ("candidate-forward-1", 25, 1.5),
        ("candidate-backward-1", -25, 1.5),
        ("candidate-backward-2", -25, 1.5),
        ("candidate-forward-2", 25, 1.5),
    ]
    try:
        for name, force, cap in plan:
            if source_snapshot() != snapshot:
                raise RuntimeError("Robot package changed during comparison")
            case = folder / name
            command = [
                sys.executable,
                str(ROOT / "scripts/experiments/run_effort_push.py"),
                "--force-x",
                str(force),
                "--duration",
                ".2",
                "--balance-enabled",
                "--ankle-balance-limit",
                str(cap),
                "--observe-com",
                "--damping-model",
                "explicit",
                "--controller-mode",
                "pid",
                "--real-time-factor",
                ".25",
                "--ros-domain-id",
                "73",
                "--output",
                str(case),
            ]
            print("[cap-study] Starting " + name, flush=True)
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
            result = (
                json.loads((case / "summary.json").read_text())
                if (case / "summary.json").exists()
                else {}
            )
            messages = (
                (case / "push/launch.log").read_text()
                if (case / "push/launch.log").exists()
                else ""
            )
            events = (
                json.loads((case / "push/recording/events.json").read_text())
                if (case / "push/recording/events.json").exists()
                else []
            )
            outcome = assess(result, code, messages, events)
            row = dict(
                name=name,
                path=str(case),
                force_x_n=force,
                ankle_balance_cap_nm=cap,
                outcome=outcome,
                error=result.get("error"),
            )
            if result.get("completed"):
                row["response"] = {
                    key: result["push"][key]
                    for key in (
                        "passed",
                        "peak_pitch_change_rad",
                        "saturated_controller_samples",
                        "continuous_contact_evidence",
                        "com_observation",
                    )
                }
            report["cases"].append(row)
            save()
            print("[cap-study] " + json.dumps(row), flush=True)
            if name.startswith("baseline"):
                if outcome != "posture_guard_stopped_partial_trial":
                    report["outcome"] = "baseline_not_reproduced"
                    save()
                    return
            elif outcome != "recovered":
                report["outcome"] = "candidate_not_verified"
                save()
                return
        if source_snapshot() != snapshot:
            raise RuntimeError("Robot package changed during final case")
        report.update(
            completed=True, outcome="candidate_recovered_both_directions_twice"
        )
        save()
    except BaseException as error:
        report["error"] = str(error)
        save()
        raise


if __name__ == "__main__":
    main()
