#!/usr/bin/env python3
"""Escalate sagittal force, preserving fresh references and the first failure."""

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


def classify(summary, returncode):
    if summary.get("completed") and returncode == 0:
        return "recovered" if summary.get("passed") else "recovery_criteria_not_met"
    # A partial run is never a completed recovery measurement. Inspect logs to
    # distinguish a posture guard from transport/recording failures.
    return "incomplete_requires_diagnosis"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ros-domain-id", type=int, choices=range(101), default=68)
    parser.add_argument(
        "--forces",
        type=int,
        nargs="+",
        choices=range(1, 101),
        default=[3, 5, 10],
        metavar="N",
        help="Strictly increasing force magnitudes; each tested forward then backward",
    )
    args = parser.parse_args()
    if args.forces != sorted(set(args.forces)):
        parser.error("--forces must be strictly increasing, without duplicates")
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    snapshot = source_snapshot()
    report = dict(
        completed=False,
        source_hashes=snapshot,
        duration_s=0.2,
        requested_forces_n=args.forces,
        cases=[],
        note="One fresh reference/push pair per case; stops at first nonpass. No repeatability or stability-boundary claim.",
    )

    def save():
        (folder / "study.json").write_text(json.dumps(report, indent=2) + "\n")

    save()
    try:
        for magnitude in args.forces:
            for direction, sign in (("forward", 1), ("backward", -1)):
                if source_snapshot() != snapshot:
                    raise RuntimeError("Robot package changed during study")
                name = f"{direction}-{magnitude}N"
                case = folder / name
                command = [
                    sys.executable,
                    str(ROOT / "scripts/experiments/run_effort_push.py"),
                    "--force-x",
                    str(sign * magnitude),
                    "--duration",
                    ".2",
                    "--balance-enabled",
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
                print(f"[study] Starting {name}", flush=True)
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
                outcome = classify(summary, code)
                row = dict(
                    name=name,
                    force_x_n=sign * magnitude,
                    path=str(case),
                    outcome=outcome,
                    returncode=code,
                )
                if summary.get("completed"):
                    row["response"] = compare([(0.2, summary)])[0]
                else:
                    row["error"] = summary.get("error", "No summary")
                report["cases"].append(row)
                save()
                print("[study] " + json.dumps(row), flush=True)
                if outcome != "recovered":
                    report["outcome"] = outcome
                    save()
                    return
        report.update(completed=True, outcome="all_requested_cases_recovered")
        save()
    except BaseException as error:
        report["error"] = str(error)
        save()
        raise


if __name__ == "__main__":
    main()
