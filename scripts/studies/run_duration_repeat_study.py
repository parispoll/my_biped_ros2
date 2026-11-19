#!/usr/bin/env python3
"""Reuse verified direction studies and run reversed-duration repeats sequentially."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import signal
import subprocess
from scripts.studies.run_duration_comparison import source_snapshot
from scripts.analysis.duration_repeat_analysis import verify, describe

ROOT = Path(__file__).resolve().parents[2]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backward-seed", type=Path, required=True)
    parser.add_argument("--forward-seed", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    sources = source_snapshot()
    seeds = [
        dict(
            repeat=1,
            source="previous verified comparison",
            **verify(path, sources, direction),
        )
        for direction, path in (
            ("backward", args.backward_seed),
            ("forward", args.forward_seed),
        )
    ]
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    report = dict(
        completed=False,
        source_hashes=sources,
        entries=seeds,
        execution_order=["backward-02", "forward-02"],
        interpretation="Two observed repeats per direction; descriptive ranges, not confidence intervals or a stability boundary.",
    )
    save = lambda: (folder / "repeat-study.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    save()
    try:
        for direction in ("backward", "forward"):
            name = direction + "-02"
            case = folder / name
            print("Running reversed-duration " + name, flush=True)
            command = [
                sys.executable,
                str(ROOT / "scripts/studies/run_duration_comparison.py"),
                "--direction",
                direction,
                "--reverse-order",
                "--output",
                str(case),
            ]
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
            raw = json.loads((case / "comparison.json").read_text())
            if raw.get("outcome") == "recovery_criteria_not_met":
                report.update(
                    outcome="recovery_criteria_not_met", stopped_case=str(case)
                )
                save()
                print("Stopped repeats: a completed case failed recovery.", flush=True)
                return
            if code or not raw.get("completed"):
                raise RuntimeError(f"Invalid repeat; inspect {case}")
            report["entries"].append(
                dict(
                    repeat=2,
                    source="fresh reversed-duration comparison",
                    **verify(case, sources, direction),
                )
            )
            if source_snapshot() != sources:
                raise RuntimeError("Source snapshot changed during repeats")
            save()
        report.update(
            completed=True,
            outcome="all_recovery_checks_passed",
            assessment=describe(report["entries"]),
        )
        save()
        print(json.dumps(report["assessment"], indent=2), flush=True)
    except BaseException as error:
        report["error"] = str(error)
        report.setdefault("outcome", "experiment_error")
        save()
        raise


if __name__ == "__main__":
    main()
