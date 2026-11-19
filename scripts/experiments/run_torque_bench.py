#!/usr/bin/env python3
"""Run independent effort-joint tests, each with a fresh owned simulator."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import os
import signal
import subprocess
import time
import uuid
from scripts.experiments.torque_bench_experiment import CASES


def trial(case, folder, gui):
    folder.mkdir()
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "55"
    env["GZ_PARTITION"] = "torque_bench_" + uuid.uuid4().hex
    child = None
    with (folder / "launch.log").open("w") as log:
        simulation = subprocess.Popen(
            [
                "ros2",
                "launch",
                "my_robot_description",
                "torque_bench.launch.py",
                f"gui:={str(gui).lower()}",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 30
            while "[bench:ready]" not in (folder / "launch.log").read_text():
                if simulation.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f"Bench did not become ready: {folder}")
                time.sleep(0.2)
            with (folder / "experiment.log").open("w") as output:
                child = subprocess.Popen(
                    [
                        sys.executable,
                        str(
                            (
                                Path(__file__).resolve().parents[2]
                                / "scripts/experiments/torque_bench_experiment.py"
                            )
                        ),
                        "--case",
                        case,
                        "--output",
                        str(folder / "recording"),
                    ],
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    env=env,
                )
                if child.wait(timeout=35) != 0:
                    raise RuntimeError(
                        f'Bench test failed: {folder / "experiment.log"}'
                    )
        finally:
            if child is not None and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    child.kill()
                    child.wait()
            if simulation.poll() is None:
                simulation.send_signal(signal.SIGINT)
            try:
                simulation.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(simulation.pid, signal.SIGKILL)
                simulation.wait()
    report = json.loads((folder / "recording/summary.json").read_text())
    print(json.dumps(report), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--cases", nargs="+", choices=CASES, default=list(CASES))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    reports = {}
    for case in args.cases:
        reports[case] = trial(case, args.output / case, args.gui)
        (args.output / "summary.json").write_text(json.dumps(reports, indent=2) + "\n")


if __name__ == "__main__":
    main()
