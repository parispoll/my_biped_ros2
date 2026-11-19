#!/usr/bin/env python3
"""Capture before warm-up, run a small bilateral COM shift, audit closed files."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import time
import uuid
from scripts.studies.run_duration_comparison import source_snapshot

ROOT = Path(__file__).resolve().parents[2]


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=12)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait()


def read(path):
    return [json.loads(s) for s in path.read_text().splitlines()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    motion = parser.add_mutually_exclusive_group()
    motion.add_argument("--displacement", type=float)
    motion.add_argument(
        "--lowering",
        type=float,
        help="Separate symmetric P54 lowering experiment, maximum 0.04 m",
    )
    motion.add_argument(
        "--lowered-shift",
        type=float,
        help="Separate P54 combined experiment: 40 mm lowering, then exactly ±0.005, ±0.01, ±0.02, ±0.03 or ±0.04 m lateral shift",
    )
    motion.add_argument(
        "--alternating-walk",
        action="store_true",
        help="Three alternating foot placements with stance-aware support exchanges",
    )
    motion.add_argument(
        "--first-step",
        action="store_true",
        help="One 50 mm forward step, 30 mm lift, then reload and hold new staggered stance",
    )
    motion.add_argument(
        "--foot-lift",
        action="store_true",
        help="Guarded 2 mm live foot lift after zero-load transfer",
    )
    motion.add_argument(
        "--zero-load",
        action="store_true",
        help="Separate physics-stamped zero-load transition, no lift commanded",
    )
    motion.add_argument(
        "--lowered-unload",
        type=float,
        choices=[0.2, 0.1, 0.05],
        help="Separate lowered positive-load target, requires --direction left/right",
    )
    motion.add_argument(
        "--unload-fraction",
        type=float,
        choices=[0.4, 0.35, 0.3],
        help="Desired share remaining on the lighter foot; both feet stay grounded",
    )
    parser.add_argument(
        "--pulse-force-y",
        type=float,
        default=0.0,
        help="Physical lateral test pulse; |force|≤1 N, scheduled 13 simulated seconds into transfer",
    )
    parser.add_argument("--pulse-duration", type=float, default=0.2)
    parser.add_argument(
        "--load-feedback",
        action="store_true",
        help="Enable bounded measured-load posture correction (P54 unloading only)",
    )
    parser.add_argument(
        "--load-reference-bias",
        type=float,
        default=0.0,
        help="Validation-only pelvis bias toward loaded side, metres; maximum ±0.003",
    )
    parser.add_argument(
        "--walk-transfer-scale",
        type=float,
        choices=[0.0, 0.25, 0.375, 0.5, 1.0],
        default=0.0,
    )
    parser.add_argument("--walk-velocity-feedforward", action="store_true")
    parser.add_argument("--walk-steps", type=int, default=3)
    parser.add_argument("--walk-step-mm", type=int, choices=[50, 75, 100], default=50)
    parser.add_argument(
        "--walk-time-scale", type=float, choices=[1.0, 0.5, 0.4, 0.25], default=1.0
    )
    parser.add_argument("--direction", choices=["left", "right"])
    parser.add_argument(
        "--real-time-factor", type=float, choices=[0.25, 0.5, 1.0], default=0.25
    )
    parser.add_argument("--lift-mm", type=int, choices=[2, 10, 30], default=2)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--capture-contacts", action="store_true")
    parser.add_argument(
        "--foot-loads",
        action="store_true",
        help="Capture and validate measured ground reaction forces",
    )
    parser.add_argument("--actuator-profile", choices=["legacy", "robotis_p54"])
    args = parser.parse_args()
    if not args.foot_lift and args.lift_mm != 2:
        parser.error("--lift-mm requires --foot-lift")
    if args.alternating_walk:
        args.zero_load = True
    if args.first_step:
        args.zero_load = True
    if args.foot_lift:
        args.zero_load = True
    if args.actuator_profile:
        os.environ["BIPED_ACTUATOR_PROFILE"] = args.actuator_profile
    from my_robot_description.model.roll_actuators import ACTIVE

    if (
        args.lowered_shift is not None
        or args.lowered_unload is not None
        or args.zero_load
    ):
        args.lowering = 0.04
    if (
        not math.isfinite(args.pulse_force_y)
        or abs(args.pulse_force_y) > 1
        or not math.isfinite(args.pulse_duration)
        or not 0.02 <= args.pulse_duration <= 0.2
    ):
        parser.error("Pulse requires |force_y|≤1 N and duration 0.02–0.2 s")
    controlled = (
        args.load_feedback
        or args.load_reference_bias != 0.0
        or args.pulse_force_y != 0.0
    )
    if controlled and (args.unload_fraction is None or ACTIVE["name"] != "robotis_p54"):
        parser.error("Load feedback/bias requires P54 partial unloading")
    if (
        not math.isfinite(args.load_reference_bias)
        or abs(args.load_reference_bias) > 0.003
    ):
        parser.error("Finite reference bias within ±0.003 m required")
    if args.lowering is not None:
        if (
            ACTIVE["name"] != "robotis_p54"
            or args.direction is not None
            and args.lowered_unload is None
            and not args.zero_load
        ):
            parser.error("Lowering requires P54, with no direction option")
        args.displacement = (
            args.lowered_shift if args.lowered_shift is not None else 0.0
        )
        args.foot_loads = True
    elif args.unload_fraction is None:
        if args.direction is not None:
            parser.error("--direction applies only to --unload-fraction")
        args.displacement = 0.01 if args.displacement is None else args.displacement
        if (
            not math.isfinite(args.displacement)
            or not 0 < abs(args.displacement) <= 0.01
        ):
            parser.error("Nonzero signed pelvis shift up to 0.01 m required")
    # Import installed modules before creating artifacts, catching stale builds early.
    import xacro
    from my_robot_description.validation.lateral_audit import audit_shift, audit_sensors
    from my_robot_description.validation.roll_audit import audit_damping

    if args.foot_loads or args.unload_fraction is not None:
        from my_robot_description.sensing.foot_loads import audit_foot_loads
    program = None
    windows = None
    if args.lowering is not None:
        from my_robot_description.motion.bilateral_lowering import LoweringProgram

        geometry = xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
        ).toxml()
        if args.zero_load:
            from my_robot_description.motion.zero_load import ZeroLoadProgram

            args.direction = args.direction or "left"
            if args.alternating_walk:
                from my_robot_description.motion.alternating_walk import (
                    AlternatingWalkProgram,
                )

                from my_robot_description.motion.profiled_walk import (
                    ProfiledWalkProgram,
                )
                from my_robot_description.motion.walking_profile import WalkingProfile

                program = ProfiledWalkProgram(
                    geometry,
                    args.direction,
                    WalkingProfile(
                        args.walk_steps,
                        args.walk_step_mm / 1000,
                        args.walk_time_scale,
                        args.walk_transfer_scale,
                        args.walk_velocity_feedforward,
                    ),
                )
            elif args.first_step:
                from my_robot_description.motion.first_step import FirstStepProgram

                program = FirstStepProgram(geometry, args.direction)
            elif args.foot_lift:
                from my_robot_description.motion.live_foot_lift import (
                    LiveFootLiftProgram,
                )

                program = LiveFootLiftProgram(
                    geometry, args.direction, args.lift_mm / 1000
                )
            else:
                program = ZeroLoadProgram(geometry, args.direction)
            args.lowered_shift = program.displacement
            args.displacement = program.displacement
        elif args.lowered_unload is not None:
            from my_robot_description.motion.lowered_unload import LoweredUnloadProgram

            args.direction = args.direction or "left"
            program = LoweredUnloadProgram(
                geometry, args.lowered_unload, args.direction
            )
            args.lowered_shift = program.displacement
            args.displacement = program.displacement
        elif args.lowered_shift is not None:
            from my_robot_description.motion.lowered_shift import LoweredShiftProgram

            program = LoweredShiftProgram(geometry, args.lowered_shift)
        else:
            program = LoweringProgram(geometry, args.lowering)
        windows = program.setup["analysis_windows"]
    if args.unload_fraction is not None:
        from my_robot_description.motion.partial_unload import (
            PartialUnloadProgram,
            audit_load_guards,
        )

        args.direction = args.direction or "left"
        args.foot_loads = True
        geometry = xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
        ).toxml()
        if controlled:
            from my_robot_description.motion.load_feedback import (
                LoadTransferProgram,
                audit_load_feedback,
            )

            program = LoadTransferProgram(
                geometry,
                args.unload_fraction,
                args.direction,
                args.load_feedback,
                args.load_reference_bias,
            )
        else:
            program = PartialUnloadProgram(
                geometry, args.unload_fraction, args.direction
            )
        args.displacement = getattr(program, "audit_displacement", program.displacement)
        windows = program.setup["analysis_windows"]
        print("[unload:plan] " + json.dumps(program.setup), flush=True)
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    env = os.environ.copy()
    env.update(
        ROS_DOMAIN_ID="77",
        GZ_PARTITION="lateral_shift_" + uuid.uuid4().hex,
        ROS_LOG_DIR=str(ROOT / "log/runtime"),
    )
    snapshot = source_snapshot()
    (folder / "source-hashes.json").write_text(json.dumps(snapshot, indent=2) + "\n")
    report = dict(
        actuator_profile=ACTIVE,
        passed=False,
        requested_pelvis_shift_m=args.displacement,
        foot_forces_measured=False,
        real_time_factor=args.real_time_factor,
        recorders_started_before_warmup=True,
        contact_wrenches_captured=args.capture_contacts or args.foot_loads,
    )
    if program:
        report["motion_plan" if args.lowering is not None else "unloading_plan"] = (
            program.setup
        )
    pulse = None
    if args.pulse_force_y:
        os.environ["GZ_PARTITION"] = env["GZ_PARTITION"]
        from my_robot_description.experiments.lateral_pulse import LateralPulse

        pulse = LateralPulse(folder, args.pulse_force_y, args.pulse_duration, args.gui)
    with (folder / "launch.log").open("w") as log, (
        folder / "sensor-recorder.log"
    ).open("w") as sensor_log:
        recorder = subprocess.Popen(
            [
                sys.executable,
                str(ROOT / "scripts/experiments/record_lateral_sensors.py"),
                "--output",
                str(folder),
                *(
                    ["--capture-contacts"]
                    if args.capture_contacts or args.foot_loads
                    else []
                ),
                *(["--state-contacts"] if args.zero_load else []),
            ],
            stdout=sensor_log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        simulation = None
        try:
            simulation = subprocess.Popen(
                [
                    "ros2",
                    "launch",
                    "my_robot_description",
                    "roll_standing.launch.py",
                    f"gui:={str(args.gui).lower()}",
                    f"wrench_debug:={str(pulse is not None).lower()}",
                    *(
                        [
                            "shift_enabled:=false",
                            "lowering_enabled:=true",
                            f"lowering_distance:={args.lowering}",
                            f"lowering_lateral_shift:={args.lowered_shift or 0.}",
                            f"lowered_unload_fraction:={args.lowered_unload or 0.}",
                            f"zero_load_enabled:={str(args.zero_load).lower()}",
                            f"alternating_walk_enabled:={str(args.alternating_walk).lower()}",
                            f"walk_steps:={args.walk_steps}",
                            f"walk_step_length:={args.walk_step_mm/1000}",
                            f"walk_time_scale:={args.walk_time_scale}",
                            f"walk_transfer_time_scale:={args.walk_transfer_scale}",
                            f"walk_velocity_feedforward:={str(args.walk_velocity_feedforward).lower()}",
                            f"first_step_enabled:={str(args.first_step).lower()}",
                            f"foot_lift_enabled:={str(args.foot_lift).lower()}",
                            f"foot_lift_clearance:={args.lift_mm/1000}",
                            f'unload_direction:={args.direction or "left"}',
                        ]
                        if args.lowering is not None
                        else (
                            [
                                "shift_enabled:=false",
                                "unload_enabled:=true",
                                f"unload_fraction:={args.unload_fraction}",
                                f"unload_direction:={args.direction}",
                                f"load_feedback_enabled:={str(args.load_feedback).lower()}",
                                f"load_reference_bias:={args.load_reference_bias}",
                            ]
                            if program
                            else [
                                "shift_enabled:=true",
                                f"shift_displacement:={args.displacement}",
                            ]
                        )
                    ),
                    f'recording:={folder/"joints.jsonl"}',
                    f"real_time_factor:={args.real_time_factor}",
                    f'com_recording:={folder/"com.jsonl"}',
                    f'damping_recording:={folder/"viscous.csv"}',
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                env=env,
                start_new_session=True,
            )
            deadline = time.monotonic() + max(
                260,
                (
                    (program.setup["total_time_s"] + 15) / args.real_time_factor
                    + (600 if args.alternating_walk else 120 if args.zero_load else 60)
                    if program
                    else 260
                ),
            )
            while "[shift:done]" not in (folder / "launch.log").read_text():
                if simulation.poll() is not None:
                    raise RuntimeError(
                        "Simulation exited before shift completion; inspect launch.log"
                    )
                if recorder.poll() is not None:
                    raise RuntimeError(
                        "Independent sensor recorder exited; inspect sensor-recorder.log"
                    )
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        "Shift startup/program exceeded its configured wall-time budget"
                    )
                if pulse:
                    pulse.tick((folder / "launch.log").read_text())
                time.sleep(0.02 if pulse else 0.2)
            if pulse:
                pulse.tick((folder / "launch.log").read_text())
            time.sleep(
                0.5
            )  # Let read-only observers receive the final physics samples.
            stop(simulation)
            stop(recorder)
            report["writers_closed_before_audit"] = True
            geometry = xacro.process_file(
                str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
            ).toxml()
            rows = read(folder / "joints.jsonl")
            com = read(folder / "com.jsonl")
            plan = json.loads((folder / "joints.setup.json").read_text())
            report["plan"] = plan
            if args.alternating_walk:
                from my_robot_description.validation.alternating_walk_audit import (
                    audit_alternating_walk,
                )

                report.update(audit_alternating_walk(folder, rows, com, program))
                if source_snapshot() != snapshot:
                    raise RuntimeError(
                        "Robot sources changed during walking measurement"
                    )
                report["source_unchanged"] = True
                report["completed"] = True
                print(json.dumps(report, indent=2))
                if not report["passed"]:
                    raise SystemExit(2)
                return
            if args.first_step:
                from my_robot_description.validation.first_step_audit import (
                    audit_first_step,
                )

                report.update(audit_first_step(folder, rows, com, program))
                if source_snapshot() != snapshot:
                    raise RuntimeError(
                        "Robot sources changed during first-step measurement"
                    )
                report["source_unchanged"] = True
                report["completed"] = True
                print(json.dumps(report, indent=2))
                if not report["passed"]:
                    raise SystemExit(2)
                return
            report["shift"] = audit_shift(
                rows,
                com,
                geometry,
                args.displacement,
                program=program,
                contact_policy=(
                    program.controller_contacts_ok if args.zero_load else None
                ),
                foot_reference=program.foot_offset if args.foot_lift else None,
            )
            start, end = (
                report["shift"]["origin_sim_time"],
                report["shift"]["end_sim_time"],
            )
            report["sensors"] = audit_sensors(
                read(folder / "sensors.jsonl"),
                read(folder / "imu-raw.jsonl"),
                start,
                end,
                plan["expected_pelvis_height_m"],
                args.displacement,
                windows=windows,
                height_program=program if args.lowering is not None else None,
                contact_policy=(
                    (lambda r: program.sensor_contacts_ok(r, start))
                    if args.zero_load
                    else None
                ),
            )
            with (folder / "viscous.csv").open() as stream:
                report["physical_damping"] = audit_damping(list(csv.DictReader(stream)))
            if source_snapshot() != snapshot:
                raise RuntimeError("Robot sources changed during measurement")
            report["source_unchanged"] = True
            if pulse:
                report["physical_pulse"] = pulse.report()
            report["passed"] = all(
                report[k]["passed"] for k in ["shift", "sensors", "physical_damping"]
            )
            if args.lowering is not None:
                from my_robot_description.validation.lowering_audit import (
                    audit_lowering,
                )
                from my_robot_description.motion.partial_unload import audit_load_guards

                if args.zero_load:
                    from my_robot_description.motion.zero_load import (
                        paired_contact_states,
                        audit_zero_load,
                    )

                    report["lowering"] = audit_lowering(
                        read(folder / "sensors.jsonl"),
                        com,
                        read(folder / "contact-wrenches.jsonl"),
                        start,
                        end,
                        program,
                        force_reference=program,
                        pairer=paired_contact_states,
                    )
                    if args.foot_lift:
                        from my_robot_description.motion.live_foot_lift import (
                            audit_lift,
                        )

                        report["foot_lift"] = audit_lift(
                            read(folder / "contact-wrenches.jsonl"),
                            com,
                            rows,
                            start,
                            end,
                            program,
                        )
                        report["passed"] &= report["foot_lift"]["passed"]
                    else:
                        report["zero_load_transition"] = audit_zero_load(
                            read(folder / "contact-wrenches.jsonl"),
                            com,
                            rows,
                            start,
                            end,
                            program,
                        )
                    report["load_guard"] = dict(
                        passed=report[
                            "foot_lift" if args.foot_lift else "zero_load_transition"
                        ]["valid_live_and_independent_guards"],
                        mode="zero_load",
                        note="Supporting-foot contact required; fresh other-foot absence allowed only in the planned near-zero window.",
                    )
                    report["passed"] &= report[
                        "foot_lift" if args.foot_lift else "zero_load_transition"
                    ]["passed"]
                else:
                    report["lowering"] = audit_lowering(
                        read(folder / "sensors.jsonl"),
                        com,
                        read(folder / "contact-wrenches.jsonl"),
                        start,
                        end,
                        program,
                        force_reference=(
                            program if args.lowered_shift is not None else None
                        ),
                    )
                    report["load_guard"] = audit_load_guards(
                        rows,
                        start,
                        end,
                        plan["mass_kg"],
                        minimum_share=getattr(program, "minimum_measured_share", 0.25),
                    )
                if args.lowered_unload is not None:
                    from my_robot_description.motion.lowered_unload import (
                        audit_unloading_goal,
                    )

                    report["unloading_goal"] = audit_unloading_goal(
                        read(folder / "contact-wrenches.jsonl"), com, start, program
                    )
                    report["passed"] &= report["unloading_goal"]["passed"]
                report["foot_forces_measured"] = report["lowering"][
                    "force_coverage_passed"
                ]
                report["passed"] &= (
                    report["lowering"]["passed"] and report["load_guard"]["passed"]
                )
            elif args.foot_loads:
                report["foot_loads"] = audit_foot_loads(
                    read(folder / "contact-wrenches.jsonl"),
                    com,
                    start,
                    end,
                    plan["mass_kg"],
                    args.displacement,
                    phase_windows=windows,
                )
                report["foot_forces_measured"] = report["foot_loads"][
                    "measurement_validated"
                ]
                report["foot_forces_validated"] = report["foot_loads"][
                    "measurement_validated"
                ]
                report["shift"]["foot_forces_measured"] = report["foot_forces_measured"]
                report["shift"][
                    "note"
                ] = "COM/geometry sub-audit; predicted_foot_forces_n remains a model estimate. See foot_loads for measured forces and their separate validation."
                report["passed"] &= report["foot_loads"]["passed"]
            if args.unload_fraction is not None:
                report["load_guard"] = audit_load_guards(
                    rows, start, end, plan["mass_kg"]
                )
                foot = program.setup["unloaded_foot"]
                hold = report["foot_loads"]["phase_means"]["hold"]
                measured = hold[foot + "_n"] / hold["total_n"]
                report["unloading_goal"] = dict(
                    target_fraction=args.unload_fraction,
                    measured_fraction=measured,
                    foot=foot,
                    passed=abs(measured - args.unload_fraction) <= 0.02,
                    foot_lifted=False,
                )
                report["foot_loads"]["used_for_control"] = "safety guard only"
                if controlled:
                    report["load_feedback"] = audit_load_feedback(
                        rows, program, start, end
                    )
                    report["foot_loads"]["used_for_control"] = (
                        "bounded pelvis correction and safety guard"
                        if args.load_feedback
                        else "safety guard only; feedback comparator"
                    )
                    report["passed"] &= report["load_feedback"]["passed"]
                report["passed"] &= (
                    report["load_guard"]["passed"]
                    and report["unloading_goal"]["passed"]
                )
            if pulse:
                report["passed"] &= report["physical_pulse"]["passed"]
            report["completed"] = True
            if not report["passed"]:
                report["outcome"] = "completed_checks_not_met"
            else:
                report["outcome"] = (
                    "completed_lowered_lateral_transfer_and_return"
                    if args.lowered_shift is not None
                    else (
                        "completed_bilateral_lowering_and_return"
                        if args.lowering is not None
                        else (
                            "completed_force_guarded_partial_unload_and_return"
                            if program
                            else (
                                "completed_validated_foot_load_shift_and_return"
                                if args.foot_loads
                                else "completed_bilateral_com_shift_and_return"
                            )
                        )
                    )
                )
        except BaseException as error:
            report["error"] = str(error)
            raise
        finally:
            if pulse:
                pulse.close()
            if simulation is not None:
                stop(simulation)
            stop(recorder)
            (folder / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
