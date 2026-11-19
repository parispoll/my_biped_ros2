#!/usr/bin/env python3
"""Launch, verify and stop one isolated effort-standing experiment."""

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
    parser.add_argument("--balance-enabled", action="store_true")
    parser.add_argument(
        "--damping-model", choices=["implicit", "explicit"], default="implicit"
    )
    parser.add_argument("--controller-mode", choices=["pd", "pid"], default="pd")
    parser.add_argument(
        "--real-time-factor", type=float, choices=[0.25, 0.5, 1.0], default=1.0
    )
    args = parser.parse_args()
    if args.balance_enabled and (
        args.controller_mode != "pid" or args.damping_model != "explicit"
    ):
        parser.error(
            "--balance-enabled requires --controller-mode pid --damping-model explicit"
        )
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
        ROS_DOMAIN_ID="56",
        GZ_PARTITION="effort_standing_" + uuid.uuid4().hex,
        ROS_LOG_DIR=str(Path.cwd() / "log/runtime"),
    )
    checks = []
    report = {
        "passed": False,
        "damping_model": args.damping_model,
        "controller_mode": args.controller_mode,
        "real_time_factor": args.real_time_factor,
        "balance_enabled": args.balance_enabled,
    }
    with (folder / "launch.log").open("w") as log:
        simulation = subprocess.Popen(
            [
                "ros2",
                "launch",
                "my_robot_description",
                "effort_standing.launch.py",
                f"gui:={str(args.gui).lower()}",
                f'recording:={folder / "joints.jsonl"}',
                f"damping_model:={args.damping_model}",
                f"controller_mode:={args.controller_mode}",
                f"balance_enabled:={str(args.balance_enabled).lower()}",
                f"real_time_factor:={args.real_time_factor}",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        try:
            deadline = time.monotonic() + (100 if args.controller_mode == "pid" else 45)
            marker = "[balance:ready]" if args.balance_enabled else "[effort:ready]"
            while marker not in (folder / "launch.log").read_text():
                if simulation.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(
                        f'Effort posture failed startup; inspect {folder / "launch.log"}'
                    )
                time.sleep(0.2)
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
                        ".55448687633",
                        "--height-tolerance",
                        ".005",
                        "--wall-timeout",
                        str(args.duration / args.real_time_factor + 20),
                    ],
                ),
                (
                    "contacts",
                    [
                        "check_contacts.py",
                        "--duration",
                        str(args.duration),
                        "--wall-timeout",
                        str(args.duration / args.real_time_factor + 20),
                    ],
                ),
                (
                    "imu",
                    [
                        "check_imu.py",
                        "--duration",
                        str(args.duration),
                        "--wall-timeout",
                        str(args.duration / args.real_time_factor + 20),
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
            for name, child, stream in checks:
                code = child.wait(timeout=args.duration / args.real_time_factor + 35)
                stream.close()
                if code != 0:
                    raise RuntimeError(
                        f'{name} verification failed: {folder / (name+".json")}'
                    )
                report[name] = json.loads((folder / f"{name}.json").read_text())
            if simulation.poll() is not None:
                raise RuntimeError("Simulation stopped during verification")
            rows = [
                json.loads(line)
                for line in (folder / "joints.jsonl").read_text().splitlines()
            ]
            active = [row for row in rows if row["active"]]
            settled = [
                row for row in active if row["sim_time"] - active[0]["sim_time"] >= 2
            ]
            if args.balance_enabled:
                start, end = (
                    report["pose"]["first"]["sim_time"],
                    report["pose"]["last"]["sim_time"],
                )
                settled = [row for row in active if start <= row["sim_time"] <= end]
            if not settled:
                raise RuntimeError("No settled effort-control samples")
            if any(row.get("integral_frozen_gap", False) for row in settled):
                raise RuntimeError(
                    "Feedback gap occurred in the standing verification window"
                )
            max_error = max(abs(e) for row in settled for e in row["error"])
            peak_speed = max(abs(v) for row in settled for v in row["velocity"])
            report["posture"] = dict(
                samples=len(settled),
                max_error_rad=max_error,
                peak_settled_speed_rad_s=peak_speed,
                peak_commands_nm=[
                    max(abs(row["command"][i]) for row in active) for i in range(6)
                ],
                final_command_nm=settled[-1]["command"],
                final_position_rad=settled[-1]["position"],
                saturated_samples=sum(
                    any(
                        abs(raw - command) > 1e-9
                        for raw, command in zip(row["requested"], row["command"])
                    )
                    for row in active
                ),
                note="Commands are requested torques; joint_states effort is not a calibrated motor-torque measurement.",
            )
            if max_error > 0.03:
                raise RuntimeError("Joint tracking exceeded 0.03 rad")
            if peak_speed > 0.05:
                raise RuntimeError("Settled joint speed exceeded 0.05 rad/s")
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
