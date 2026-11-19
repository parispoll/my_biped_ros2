#!/usr/bin/env python3
"""Run a no-push reference and a gentle push in separate fresh effort simulations."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import csv
import json
import os
import signal
import subprocess
import time
import uuid
from my_robot_description.experiments.effort_push import EffortPushConfig
from my_robot_description.studies.damping import validate_damping_trace
from my_robot_description.validation.experiment_failure import launch_failure


def launch_command(
    folder,
    gui,
    joint_friction,
    physics_probe,
    dart_probe,
    real_time_factor,
    physics_step,
    damping_model,
    controller_mode="pd",
    balance_observer=False,
    balance_enabled=False,
    ankle_balance_limit=0.5,
    observe_com=False,
    balance_kd=4.0,
):
    return [
        arg
        for arg in [
            "ros2",
            "launch",
            "my_robot_description",
            "effort_standing.launch.py",
            f"gui:={str(gui).lower()}",
            "verbosity:=4",
            f"joint_friction:={joint_friction}",
            f'physics_recording:={folder / "physics.csv" if physics_probe else ""}',
            f'dart_recording:={folder / "dart.csv" if dart_probe else ""}',
            f"real_time_factor:={real_time_factor}",
            f"physics_step:={physics_step}",
            f"damping_model:={damping_model}",
            f"controller_mode:={controller_mode}",
            f"balance_enabled:={str(balance_enabled).lower()}",
            f"ankle_balance_limit:={ankle_balance_limit}",
            f"balance_kd:={balance_kd}",
            f"com_visual:={str(observe_com).lower()}",
            f'com_recording:={folder / "com.jsonl" if observe_com else ""}',
            f'balance_recording:={folder / "balance.jsonl" if balance_observer else ""}',
            f'damping_recording:={folder / "viscous.csv" if damping_model=="explicit" else ""}',
            f'recording:={folder / "controller.jsonl"}',
        ]
        if not arg.endswith(":=")
    ]


def trial(
    folder,
    force,
    duration,
    recovery,
    gui,
    joint_friction=0.2,
    physics_probe=False,
    dart_probe=False,
    real_time_factor=1.0,
    physics_step=0.001,
    damping_model="implicit",
    controller_mode="pd",
    balance_observer=False,
    balance_enabled=False,
    posture_delay=0.0,
    ros_domain_id=57,
    ankle_balance_limit=0.5,
    observe_com=False,
    balance_kd=4.0,
):
    folder.mkdir()
    env = os.environ.copy()
    # Separate from position mode (42), bench (55), and manual effort mode (56).
    env.update(
        ROS_DOMAIN_ID=str(ros_domain_id),
        GZ_PARTITION="effort_push_" + uuid.uuid4().hex,
        ROS_LOG_DIR=str(Path.cwd() / "log/runtime"),
    )
    experiment = None
    observed = {}
    with (folder / "launch.log").open("w") as log:
        simulation = subprocess.Popen(
            launch_command(
                folder,
                gui,
                joint_friction,
                physics_probe,
                dart_probe,
                real_time_factor,
                physics_step,
                damping_model,
                controller_mode,
                balance_observer,
                balance_enabled,
                ankle_balance_limit,
                observe_com,
                balance_kd,
            ),
            stdout=log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        try:
            startup_budget = 150 if controller_mode == "pid" else 45
            deadline = time.monotonic() + startup_budget
            while "[effort:activated]" not in (folder / "launch.log").read_text():
                if simulation.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError(
                        f'Effort controller did not activate: {launch_failure(folder / "launch.log")}; inspect {folder / "launch.log"}'
                    )
                time.sleep(0.2)
            # Create ROS graph participants during warmup, before declaring the
            # measured experiment ready. The recorder has an explicit arm gate.
            interfaces = subprocess.run(
                ["ros2", "control", "list_hardware_interfaces"],
                capture_output=True,
                text=True,
                check=True,
                timeout=10,
                env=env,
            )
            (folder / "hardware-interfaces.txt").write_text(interfaces.stdout)
            with (folder / "experiment.log").open("w") as output:
                wall_timeout = max(35.0, (recovery + 6) / real_time_factor + 10)
                experiment = subprocess.Popen(
                    [
                        sys.executable,
                        str(
                            (
                                Path(__file__).resolve().parents[2]
                                / "scripts/experiments/effort_push_experiment.py"
                            )
                        ),
                        "--force-x",
                        str(force),
                        "--duration",
                        str(duration),
                        "--recovery",
                        str(recovery),
                        "--controller-setup",
                        str(folder / "controller.setup.json"),
                        "--ready-file",
                        str(folder / "recording.ready"),
                        "--wall-timeout",
                        str(wall_timeout + startup_budget),
                        "--output",
                        str(folder / "recording"),
                    ],
                    stdout=output,
                    stderr=subprocess.STDOUT,
                    env=env,
                )
                marker = "[balance:ready]" if balance_enabled else "[effort:ready]"
                while marker not in (folder / "launch.log").read_text():
                    if (
                        simulation.poll() is not None
                        or experiment.poll() is not None
                        or time.monotonic() > deadline
                    ):
                        raise RuntimeError(
                            f'Effort standing did not become ready: {launch_failure(folder / "launch.log")}; inspect {folder / "launch.log"} and {folder / "experiment.log"}'
                        )
                    time.sleep(0.2)
                if posture_delay:
                    records = [
                        json.loads(x)
                        for x in (folder / "controller.jsonl").read_text().splitlines()
                    ]
                    target = records[-1]["sim_time"] + posture_delay
                    while records[-1]["sim_time"] < target:
                        if (
                            simulation.poll() is not None
                            or experiment.poll() is not None
                            or time.monotonic() > deadline
                        ):
                            raise RuntimeError("Posture delay failed")
                        time.sleep(0.1)
                        records = [
                            json.loads(x)
                            for x in (folder / "controller.jsonl")
                            .read_text()
                            .splitlines()
                        ]
                arm_time = time.monotonic()
                (folder / "recording.ready").write_text(
                    json.dumps(dict(armed_wall_time=arm_time, readiness_marker=marker))
                    + "\n"
                )
                deadline = time.monotonic() + max(60, wall_timeout + 10)
                while experiment.poll() is None:
                    messages = (folder / "launch.log").read_text()
                    for name, marker in [
                        ("queued", "Queueing persistent wrench"),
                        ("cleared", "Clearing persistent wrench for entity"),
                    ]:
                        if name not in observed and marker in messages:
                            observed[name] = time.monotonic()
                    if simulation.poll() is not None:
                        raise RuntimeError(
                            f'Effort simulation stopped during experiment: {launch_failure(folder / "launch.log")}; inspect {folder / "launch.log"}'
                        )
                    if time.monotonic() > deadline:
                        raise TimeoutError("Push recorder exceeded wall-clock deadline")
                    time.sleep(0.2)
                if experiment.returncode != 0:
                    raise RuntimeError(
                        f'Push recording failed; inspect {folder / "experiment.log"}'
                    )
            if simulation.poll() is not None:
                raise RuntimeError("Simulator stopped before verification finished")
            report = json.loads((folder / "recording/summary.json").read_text())
            report["recorder_arming"] = dict(
                connected_wall_time=report["recorder_connected_wall_time"],
                armed_wall_time=arm_time,
                recording_started_wall_time=report["events"][0]["wall_time"],
            )
            if damping_model == "explicit":
                text = (folder / "viscous.csv").read_text()
                lines = text.splitlines()
                if text and not text.endswith("\n"):
                    lines = lines[:-1]
                report["viscous_damping_audit"] = validate_damping_trace(
                    list(csv.DictReader(lines))
                )
                if not report["viscous_damping_audit"]["passed"]:
                    raise RuntimeError(
                        "Viscous damping trace failed its constitutive/passivity check"
                    )
            report["joint_friction_setting"] = joint_friction
            report.update(
                physics_step_requested_s=physics_step,
                real_time_factor_requested=real_time_factor,
                damping_model=damping_model,
                controller_mode=controller_mode,
            )
            messages = (folder / "launch.log").read_text()
            handling = {
                "queued_in_gazebo": "Queueing persistent wrench" in messages,
                "cleared_in_gazebo": "Clearing persistent wrench for entity"
                in messages,
                "clear_observed_while_recording": "cleared" in observed,
                "note": "Gazebo debug messages confirm handling, not calibrated force or exact actuation timing.",
            }
            report["force_handling"] = handling
            clear_time = next(
                e["wall_time"]
                for e in report["events"]
                if e["event"] == "clear_published"
            )
            handling["clear_log_observation_lag_wall_s"] = (
                observed["cleared"] - clear_time if "cleared" in observed else None
            )
            (folder / "force-handling.json").write_text(
                json.dumps(handling, indent=2) + "\n"
            )
            if (
                not handling["queued_in_gazebo"]
                or not handling["cleared_in_gazebo"]
                or not handling["clear_observed_while_recording"]
                or not -1 <= handling["clear_log_observation_lag_wall_s"] <= 2
            ):
                raise RuntimeError(
                    "Could not confirm Gazebo queued and cleared the persistent wrench"
                )
            controller = [
                json.loads(line)
                for line in (folder / "controller.jsonl").read_text().splitlines()
            ]
            first, last = (
                report["events"][0]["sim_time"],
                report["events"][-1]["sim_time"] + recovery,
            )
            controlled = [
                row
                for row in controller
                if row["active"] and first <= row["sim_time"] <= last
            ]
            if not controlled:
                raise RuntimeError("No controller-term recording for this push")
            report["control_gap_frozen_samples"] = sum(
                r.get("integral_frozen_gap", False) for r in controlled
            )
            if report["control_gap_frozen_samples"]:
                raise RuntimeError(
                    "Feedback gap occurred inside the measurement window; trial invalid"
                )
            if balance_observer:
                from my_robot_description.validation.balance_audit import audit_balance

                rows = [
                    json.loads(line)
                    for line in (folder / "balance.jsonl").read_text().splitlines()
                ]
                report["balance_observer"] = audit_balance(
                    rows, controlled, first, last, active=balance_enabled
                )
                if not report["balance_observer"]["passed"]:
                    raise RuntimeError("Balance observation audit failed")
            if observe_com:
                from my_robot_description.validation.com_audit import audit_com

                com_rows = [
                    json.loads(line)
                    for line in (folder / "com.jsonl").read_text().splitlines()
                ]
                report["com_observation"] = audit_com(com_rows, first, last)
                if not report["com_observation"]["passed"]:
                    raise RuntimeError("COM observation coverage invalid")
            report["balance_enabled"] = balance_enabled
            report["ankle_balance_limit_nm"] = ankle_balance_limit
            report["balance_kd_nm_s_per_rad"] = balance_kd
            report["ros_domain_id"] = ros_domain_id
            report["controller_samples"] = len(controlled)
            report["saturated_controller_samples"] = sum(
                any(abs(a - b) > 1e-9 for a, b in zip(row["requested"], row["command"]))
                for row in controlled
            )
            (folder / "recording/summary.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
            print(
                f"{folder.name}: "
                + json.dumps(
                    {
                        key: report[key]
                        for key in (
                            "completed",
                            "passed",
                            "peak_pitch_change_rad",
                            "final_second_within_return_band",
                            "continuous_contact_evidence",
                            "saturated_controller_samples",
                            "force_handling",
                        )
                    }
                ),
                flush=True,
            )
            return report
        finally:
            if experiment is not None and experiment.poll() is None:
                experiment.send_signal(signal.SIGINT)
                try:
                    experiment.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    experiment.kill()
                    experiment.wait()
            if simulation.poll() is None:
                simulation.send_signal(signal.SIGINT)
            try:
                simulation.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(simulation.pid, signal.SIGKILL)
                simulation.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument(
        "--ros-domain-id",
        type=int,
        choices=range(101),
        default=57,
        metavar="0..100",
        help="Experiment ROS domain; default preserves the existing benchmark domain",
    )
    parser.add_argument(
        "--posture-delay",
        type=float,
        choices=[0.0, 3.0],
        default=0.0,
        help="Extra simulated settling after readiness, for matched comparisons",
    )
    parser.add_argument(
        "--observe-com",
        action="store_true",
        help="Read-only COM recording; optional markers if Gazebo GUI is present",
    )
    parser.add_argument(
        "--balance-kd",
        type=float,
        choices=[4.0, 8.0],
        default=4.0,
        help="Body pitch-rate feedback gain, total N m s/rad",
    )
    parser.add_argument(
        "--ankle-balance-limit",
        type=float,
        choices=[0.5, 1.5],
        default=0.5,
        help="Per-ankle IMU correction cap; total motor cap remains 6 N m",
    )
    parser.add_argument(
        "--balance-enabled",
        action="store_true",
        help="Apply bounded IMU ankle feedback; implies observer recording",
    )
    parser.add_argument(
        "--balance-observer",
        action="store_true",
        help="Record IMU-based ankle proposals without applying them",
    )
    parser.add_argument(
        "--damping-model", choices=["implicit", "explicit"], default="implicit"
    )
    parser.add_argument("--controller-mode", choices=["pd", "pid"], default="pd")
    parser.add_argument(
        "--physics-probe",
        action="store_true",
        help="Enable the read-only physics input/wrench observer",
    )
    parser.add_argument(
        "--dart-probe",
        action="store_true",
        help="Enable the DART 6.13 read-only joint getter audit",
    )
    parser.add_argument(
        "--real-time-factor",
        type=float,
        choices=[0.25, 0.5, 1.0],
        default=1.0,
        help="Wall-clock pacing only",
    )
    parser.add_argument(
        "--physics-step",
        type=float,
        choices=[0.001, 0.0005, 0.00025],
        default=0.001,
        help="Diagnostic physics timestep; default preserves 1 ms",
    )
    parser.add_argument("--force-x", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=0.2)
    parser.add_argument("--recovery", type=float, default=10.0)
    parser.add_argument(
        "--joint-friction",
        type=float,
        choices=[0.0, 0.2],
        default=0.2,
        help="Diagnostic joint friction override; default 0.2 preserves the model",
    )
    args = parser.parse_args()
    if args.balance_kd != 4.0 and (
        not args.balance_enabled or args.ankle_balance_limit != 1.5
    ):
        parser.error(
            "Higher body damping requires active balance and the 1.5 N m correction profile"
        )
    if args.ankle_balance_limit != 0.5 and not args.balance_enabled:
        parser.error("A candidate ankle balance cap requires --balance-enabled")
    if args.balance_enabled and (
        args.controller_mode != "pid" or args.damping_model != "explicit"
    ):
        parser.error(
            "--balance-enabled requires --controller-mode pid --damping-model explicit"
        )
    config = EffortPushConfig(
        force_x=args.force_x, duration=args.duration, recovery=args.recovery
    ).validate()
    if config.force_x == 0:
        parser.error(
            "The push must be nonzero; a zero-force reference is run automatically"
        )
    folder = args.output.resolve()
    if folder.exists():
        parser.error(
            f"Output folder already exists: {folder}. Choose a new --output name to preserve earlier evidence."
        )
    folder.mkdir(parents=True, exist_ok=False)
    reports = {"completed": False, "passed": False}
    try:
        for name, force in [("reference", 0.0), ("push", config.force_x)]:
            reports[name] = trial(
                folder / name,
                force,
                config.duration,
                config.recovery,
                args.gui,
                args.joint_friction,
                args.physics_probe,
                args.dart_probe,
                args.real_time_factor,
                args.physics_step,
                args.damping_model,
                args.controller_mode,
                args.balance_observer or args.balance_enabled,
                args.balance_enabled,
                args.posture_delay,
                args.ros_domain_id,
                args.ankle_balance_limit,
                args.observe_com,
                args.balance_kd,
            )
            if name == "reference" and not reports[name]["passed"]:
                raise RuntimeError("No-push reference did not meet baseline criteria")
        reference = reports["reference"]["peak_pitch_change_rad"]
        response = reports["push"]["peak_pitch_change_rad"]
        reports["pitch_response_above_reference"] = response > max(1e-5, reference * 10)
        reports["completed"] = True
        reports["passed"] = (
            reports["push"]["passed"] and reports["pitch_response_above_reference"]
        )
        reports["outcome"] = (
            "returned_within_bands"
            if reports["passed"]
            else "did_not_meet_recovery_criteria"
        )
    except Exception as error:
        reports["error"] = str(error)
        raise
    finally:
        (folder / "summary.json").write_text(json.dumps(reports, indent=2) + "\n")
    print(
        f'Experiment completed={reports["completed"]}; recovery criteria passed={reports["passed"]}',
        flush=True,
    )


if __name__ == "__main__":
    main()
