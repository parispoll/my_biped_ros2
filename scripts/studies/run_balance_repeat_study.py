#!/usr/bin/env python3
"""Reuse one verified matched study, run a reversed-order repeat, summarize both."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import subprocess
from scripts.studies.run_active_balance_comparison import hashes
from my_robot_description.studies.balance_repeats import summarize_repeats


def verify(folder, current):
    report = json.loads((folder / "validation.json").read_text())
    if not report["completed"] or report["source_hashes"] != current:
        raise ValueError("Incomplete study or changed control/model sources")
    for case in report["cases"].values():
        outcome = json.loads((Path(case) / "summary.json").read_text())
        if not outcome["completed"]:
            raise ValueError("Incomplete raw trial")
        for phase in ("reference", "push"):
            trial = outcome[phase]
            if (
                trial["control_gap_frozen_samples"]
                or not trial["balance_observer"]["passed"]
                or not trial["viscous_damping_audit"]["passed"]
            ):
                raise ValueError("Invalid measurement audit")
            if (
                trial["damping_model"] != "explicit"
                or trial["controller_mode"] != "pid"
                or trial["physics_step_requested_s"] != 0.001
                or trial["real_time_factor_requested"] != 0.25
            ):
                raise ValueError("Changed simulation/control protocol")
    return {d: report[d] for d in ("forward", "backward")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    current = hashes()
    seed = args.seed.resolve()
    first = verify(seed, current)
    folder = args.output.resolve()
    if folder.exists():
        parser.error("Choose a new output folder; previous evidence is preserved")
    folder.mkdir(parents=True)
    report = dict(
        completed=False,
        source_hashes=current,
        repeats=[
            dict(
                repeat=1,
                path=str(seed),
                source="previous verified study",
                comparisons=first,
            )
        ],
    )
    save = lambda: (folder / "repeat-study.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    save()
    try:
        with (folder / "repeat-02.log").open("w") as stream:
            subprocess.run(
                [
                    sys.executable,
                    str(
                        (
                            Path(__file__).resolve().parents[2]
                            / "scripts/studies/run_active_balance_comparison.py"
                        )
                    ),
                    "--reverse-order",
                    "--output",
                    str(folder / "repeat-02"),
                ],
                stdout=stream,
                stderr=subprocess.STDOUT,
                check=True,
            )
        second = verify(folder / "repeat-02", current)
        if hashes() != current:
            raise ValueError("Sources changed during repeats")
        report["repeats"].append(
            dict(
                repeat=2,
                path=str(folder / "repeat-02"),
                source="fresh reversed-order study",
                comparisons=second,
            )
        )
        report["assessment"] = summarize_repeats(report["repeats"])
        report["completed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        save()
    print(json.dumps(report["assessment"], indent=2))


if __name__ == "__main__":
    main()
