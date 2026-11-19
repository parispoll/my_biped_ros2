#!/usr/bin/env python3
"""Own fresh isolated simulations for a no-push control and repeated small pushes."""

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


def trial(folder, force, gui=False):
    folder.mkdir()
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = "190"
    env["GZ_PARTITION"] = "push_" + uuid.uuid4().hex
    log = folder / "launch.log"
    experiment = None
    with log.open("w") as stream:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "my_robot_description",
                "standing.launch.py",
                "stance:=crouch",
                f"gui:={str(gui).lower()}",
            ],
            stdout=stream,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 30
            while "[biped:standing]" not in log.read_text():
                if process.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(f"Simulation did not reach crouch: {log}")
                time.sleep(0.2)
            with (folder / "experiment.log").open("w") as output:
                experiment = subprocess.Popen(
                    [
                        sys.executable,
                        str(
                            (
                                Path(__file__).resolve().parents[2]
                                / "scripts/experiments/push_experiment.py"
                            )
                        ),
                        "--force-x",
                        str(force),
                        "--output",
                        str(folder / "recording"),
                    ],
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    env=env,
                )
                if experiment.wait(timeout=40) != 0:
                    raise RuntimeError(
                        f'Push recording failed: {folder / "experiment.log"}'
                    )
        finally:
            if experiment is not None and experiment.poll() is None:
                experiment.send_signal(signal.SIGINT)
                try:
                    experiment.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    experiment.kill()
                    experiment.wait()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    report = json.loads((folder / "recording/summary.json").read_text())
    print(
        json.dumps(
            {
                "trial": folder.name,
                "completed": report["completed"],
                "peak_pitch_change_rad": report.get("peak_pitch_change_rad"),
            }
        ),
        flush=True,
    )
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--gui", action="store_true", help="Show each isolated trial in Gazebo."
    )
    parser.add_argument("--push-repeats", type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.push_repeats <= 5:
        parser.error("Use 1 to 5 push repeats")
    args.output.mkdir(parents=True, exist_ok=False)
    reports = {}
    for name, force in [
        ("control", 0),
        *[(f"push-{i+1:02d}", 5) for i in range(args.push_repeats)],
    ]:
        reports[name] = trial(args.output / name, force, args.gui)
        (args.output / "summary.json").write_text(json.dumps(reports, indent=2) + "\n")


if __name__ == "__main__":
    main()
