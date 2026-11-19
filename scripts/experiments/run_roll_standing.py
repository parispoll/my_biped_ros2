#!/usr/bin/env python3
"""Verify isolated ten-joint two-foot standing, never the old six-joint controller."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math
import os
import signal
import subprocess
import time
import uuid


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=30)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--actuator-profile", choices=["legacy", "robotis_p54"])
    args = parser.parse_args()
    if args.actuator_profile:
        os.environ["BIPED_ACTUATOR_PROFILE"] = args.actuator_profile
    from my_robot_description.model.roll_actuators import ACTIVE

    if not math.isfinite(args.duration) or not 5 <= args.duration <= 120:
        parser.error("duration must be 5–120 simulation seconds")
    folder = args.output.resolve()
    if folder.exists():
        parser.error(
            f"Output folder already exists: {folder}. Choose a new --output name to preserve earlier evidence."
        )
    folder.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(
        ROS_DOMAIN_ID="76",
        GZ_PARTITION="roll_standing_" + uuid.uuid4().hex,
        ROS_LOG_DIR=str(Path.cwd() / "log/runtime"),
    )
    from scripts.studies.run_duration_comparison import source_snapshot

    snapshot = source_snapshot()
    (folder / "source-hashes.json").write_text(json.dumps(snapshot, indent=2) + "\n")
    checks = []
    report = {
        "passed": False,
        "model": "biped_roll",
        "real_time_factor": 0.25,
        "active_body_balance": False,
        "actuator_profile": ACTIVE,
    }
    with (folder / "launch.log").open("w") as log:
        simulation = subprocess.Popen(
            [
                "ros2",
                "launch",
                "my_robot_description",
                "roll_standing.launch.py",
                f"gui:={str(args.gui).lower()}",
                f'recording:={folder / "joints.jsonl"}',
                f'com_recording:={folder / "com.jsonl"}',
                f'damping_recording:={folder / "viscous.csv"}',
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + 160
            marker = "[roll:ready]"
            while marker not in (folder / "launch.log").read_text():
                if simulation.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(
                        f'Roll posture failed startup; inspect {folder / "launch.log"}'
                    )
                time.sleep(0.2)
            plan = json.loads((folder / "joints.setup.json").read_text())
            report["plan"] = plan
            scripts = Path(__file__).resolve().parent
            interfaces = subprocess.run(
                ["ros2", "control", "list_hardware_interfaces"],
                capture_output=True,
                text=True,
                env=env,
                timeout=10,
                check=True,
            )
            (folder / "hardware-interfaces.txt").write_text(interfaces.stdout)
            for name, arguments in [
                (
                    "pose",
                    [
                        "check_standing.py",
                        "--duration",
                        str(args.duration),
                        "--max-tilt",
                        ".05",
                        "--expected-height",
                        str(plan["expected_pelvis_height_m"]),
                        "--height-tolerance",
                        ".005",
                        "--wall-timeout",
                        str(args.duration / 0.25 + 20),
                    ],
                ),
                (
                    "contacts",
                    [
                        "check_contacts.py",
                        "--duration",
                        str(args.duration),
                        "--wall-timeout",
                        str(args.duration / 0.25 + 20),
                    ],
                ),
                (
                    "imu",
                    [
                        "check_imu.py",
                        "--duration",
                        str(args.duration),
                        "--wall-timeout",
                        str(args.duration / 0.25 + 20),
                    ],
                ),
            ]:
                stream = (folder / f"{name}.json").open("w")
                child = subprocess.Popen(
                    [sys.executable, str(scripts / arguments[0]), *arguments[1:]],
                    stdout=stream,
                    stderr=subprocess.STDOUT,
                    env=env,
                )
                checks.append((name, child, stream))
                time.sleep(
                    0.5
                )  # Stagger verifier imports/discovery instead of one startup burst.
            for name, child, stream in checks:
                code = child.wait(timeout=args.duration / 0.25 + 35)
                stream.close()
                if code != 0:
                    raise RuntimeError(
                        f'{name} verification failed: {folder / (name+".json")}'
                    )
                report[name] = json.loads((folder / f"{name}.json").read_text())
            if simulation.poll() is not None:
                raise RuntimeError("Simulation stopped during verification")
            # Freeze/close all writers before reading CSV and JSONL evidence.
            simulation.send_signal(signal.SIGINT)
            try:
                simulation.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(simulation.pid, signal.SIGKILL)
                simulation.wait()
            report["simulation_stopped_before_file_audit"] = True
            rows = [
                json.loads(line)
                for line in (folder / "joints.jsonl").read_text().splitlines()
            ]
            from my_robot_description.validation.roll_audit import (
                audit_posture,
                audit_damping,
            )
            from my_robot_description.validation.com_audit import audit_com
            import csv

            start, end = (
                report["pose"]["first"]["sim_time"],
                report["pose"]["last"]["sim_time"],
            )
            report["posture"] = audit_posture(rows, plan, start, end)
            com = [
                json.loads(line)
                for line in (folder / "com.jsonl").read_text().splitlines()
            ]
            report["com"] = audit_com(com, start, end)
            report["com"]["expected_mass_kg"] = plan["mass_kg"]
            report["com"]["passed"] &= (
                abs(report["com"].get("mass_kg", 0) - plan["mass_kg"]) < 1e-9
            )
            with (folder / "viscous.csv").open() as stream:
                report["physical_damping"] = audit_damping(list(csv.DictReader(stream)))
            if not all(
                report[k]["passed"]
                for k in [
                    "pose",
                    "contacts",
                    "imu",
                    "posture",
                    "com",
                    "physical_damping",
                ]
            ):
                raise RuntimeError("One or more roll-standing evidence checks failed")
            if source_snapshot() != snapshot:
                raise RuntimeError("Robot sources changed during measurement")
            report["source_unchanged"] = True
            report["passed"] = True
        except Exception as error:
            report["error"] = str(error)
            raise
        finally:
            for _, child, stream in checks:
                if child.poll() is None:
                    child.terminate()
                    try:
                        child.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        child.kill()
                        child.wait()
                stream.close()
            if simulation.poll() is None:
                simulation.send_signal(signal.SIGINT)
            try:
                simulation.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(simulation.pid, signal.SIGKILL)
                simulation.wait()
            (folder / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
