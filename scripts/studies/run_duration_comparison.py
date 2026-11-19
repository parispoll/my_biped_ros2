#!/usr/bin/env python3
"""Compare 2 N pulses of .20/.30 s using fresh paired experiments."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import hashlib
import json
import math
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def source_snapshot():
    package = ROOT / "src/my_robot_description"
    return {
        str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(package.rglob("*"))
        if p.is_file()
        and p.suffix in (".py", ".yaml", ".xacro", ".sdf", ".cpp", ".config")
    }


def compare(reports):
    """Describe valid completed cases; do not infer a falling/stability limit."""
    rows = []
    for duration, result in reports:
        if not result.get("completed"):
            raise ValueError(
                "Incomplete experiment cannot be compared as a physical response"
            )
        push = result["push"]
        rows.append(
            dict(
                duration_s=duration,
                force_x_n=push["requested"]["force_x"],
                nominal_requested_impulse_n_s=abs(push["requested"]["force_x"])
                * duration,
                command_window_s=push["command_window_s"],
                peak_pitch_deg=math.degrees(push["peak_pitch_change_rad"]),
                peak_pitch_rate_rad_s=push["peak_pitch_rate_rad_s"],
                returned_within_bands=push["final_second_within_return_band"],
                continuous_contact_evidence=push["continuous_contact_evidence"],
                saturated_controller_samples=push["saturated_controller_samples"],
                last_outside_band_after_clear_s=push["last_outside_band_after_clear_s"],
                last_outside_band_after_push_s=(
                    None
                    if push["last_outside_band_after_clear_s"] is None
                    else push["command_window_s"]
                    + push["last_outside_band_after_clear_s"]
                ),
                recovery_passed=result["passed"],
            )
        )
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument(
        "--direction", choices=("backward", "forward"), default="backward"
    )
    parser.add_argument(
        "--reverse-order",
        action="store_true",
        help="Run .30 s before .20 s for a repeat",
    )
    args = parser.parse_args()
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    snapshot = source_snapshot()
    force = -2 if args.direction == "backward" else 2
    durations = (0.3, 0.2) if args.reverse_order else (0.2, 0.3)
    report = dict(
        completed=False,
        source_hashes=snapshot,
        cases=[],
        direction=args.direction,
        duration_execution_order_s=list(durations),
        requested_impulse_increase_percent=50,
        interpretation="One fresh pair per duration. Commanded impulses, not independently measured forces; no stability-limit claim.",
    )
    save = lambda: (folder / "comparison.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    save()
    results = []
    try:
        for duration in durations:
            case = folder / f"pulse-{round(duration*1000)}ms"
            command = [
                sys.executable,
                str(ROOT / "scripts/experiments/run_effort_push.py"),
                "--force-x",
                str(force),
                "--duration",
                str(duration),
                "--balance-enabled",
                "--damping-model",
                "explicit",
                "--controller-mode",
                "pid",
                "--real-time-factor",
                ".25",
                "--ros-domain-id",
                "63",
                *(["--gui"] if args.gui else []),
                "--output",
                str(case),
            ]
            print(f"Running {force:+d} N for {duration:.2f} s", flush=True)
            with (folder / f"{case.name}.log").open("w") as log:
                process = subprocess.Popen(
                    command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT
                )
                try:
                    code = process.wait()
                except BaseException:
                    import signal

                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                    raise
            summary = json.loads((case / "summary.json").read_text())
            record = dict(
                duration_s=duration,
                path=str(case),
                runner_returncode=code,
                completed=summary.get("completed", False),
            )
            report["cases"].append(record)
            save()
            if code or not summary.get("completed"):
                report["outcome"] = "experiment_error"
                raise RuntimeError(
                    f'Experiment incomplete; not classified as failed recovery: {summary.get("error")}. Inspect {case}'
                )
            if source_snapshot() != snapshot:
                raise RuntimeError("Source changed during comparison")
            results.append((duration, summary))
            report["responses"] = compare(results)
            save()
            if not summary["passed"]:
                report["outcome"] = "recovery_criteria_not_met"
                save()
                print(
                    "Stopped escalation: completed case did not meet recovery criteria.",
                    flush=True,
                )
                return
        report.update(completed=True, outcome="both_tested_durations_passed")
        save()
        print(json.dumps(report["responses"], indent=2), flush=True)
    except BaseException as error:
        report["error"] = str(error)
        save()
        raise


if __name__ == "__main__":
    main()
