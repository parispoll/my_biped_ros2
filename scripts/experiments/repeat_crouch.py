#!/usr/bin/env python3
"""Repeat fresh isolated crouch launches, preserving logs and tracking data."""

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


def run_once(index, folder, extra_args):
    folder.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(160 + index)
    env["GZ_PARTITION"] = "crouch_check_" + uuid.uuid4().hex
    path = folder / "launch.log"
    with path.open("w") as log:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "my_robot_description",
                "standing.launch.py",
                "stance:=crouch",
                f'tracking_log:={folder.resolve() / "tracking.jsonl"}',
                *extra_args,
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        outcome = "timeout"
        started = time.monotonic()
        try:
            while time.monotonic() - started < 45:
                content = path.read_text()
                if "[biped:standing]" in content:
                    outcome = "succeeded"
                    break
                if process.poll() is not None:
                    outcome = "failed"
                    break
                time.sleep(0.2)
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    summary = folder / "tracking.summary.json"
    report = {
        "run": index,
        "outcome": outcome,
        "wall_duration": time.monotonic() - started,
        "tracking": json.loads(summary.read_text()) if summary.exists() else None,
    }
    (folder / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("launch_args", nargs="*")
    args = parser.parse_args()
    if not 1 <= args.runs <= 20:
        parser.error("Use 1 to 20 runs")
    args.output.mkdir(parents=True, exist_ok=False)
    reports = []
    for index in range(args.runs):
        reports.append(
            run_once(index, args.output / f"run-{index+1:02d}", args.launch_args)
        )
        (args.output / "summary.json").write_text(json.dumps(reports, indent=2) + "\n")


if __name__ == "__main__":
    main()
