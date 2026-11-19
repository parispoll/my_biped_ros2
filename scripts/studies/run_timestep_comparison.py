#!/usr/bin/env python3
"""Compare physics steps and repeats with fixed effort-control settings."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import hashlib
import json
import math
import signal
import subprocess
from scripts.analysis.analyze_physics_probe import analyze
from my_robot_description.studies.resolution_study import (
    summarize_resolution_study,
    PITCH_TOLERANCE_DEG,
    JOINT_TOLERANCE_RAD,
)

ROOT = Path(__file__).resolve().parents[2]
FIXED_FILES = (
    "config/effort_standing.yaml",
    "urdf/components/control_effort.xacro",
    "nodes/legacy/hold_effort_stance.py",
    "my_robot_description/control/posture_effort.py",
    "urdf/components/leg.xacro",
    "urdf/components/properties.xacro",
    "worlds/standing.sdf",
    "plugins/viscous_damping.cpp",
    "urdf/components/explicit_damping.xacro",
    "my_robot_description/control/integral.py",
    "urdf/effort_standing.urdf.xacro",
)


def model_hashes():
    return {
        name: hashlib.sha256(
            (ROOT / "src/my_robot_description" / name).read_bytes()
        ).hexdigest()
        for name in FIXED_FILES
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--damping-model", choices=["implicit", "explicit"], default="implicit"
    )
    parser.add_argument("--controller-mode", choices=["pd", "pid"], default="pd")
    parser.add_argument("--force-x", type=float, choices=[1.0, 2.0], default=1.0)
    parser.add_argument(
        "--real-time-factor", type=float, choices=[0.25, 0.5, 1.0], default=0.25
    )
    parser.add_argument("--repeats", type=int, choices=[1, 2], default=1)
    parser.add_argument(
        "--steps",
        type=float,
        nargs="+",
        choices=[0.001, 0.0005, 0.00025],
        default=[0.001, 0.0005],
    )
    parser.add_argument(
        "--seed-comparison",
        type=Path,
        help="Reuse matching, completed prior trials without copying or rerunning them",
    )
    args = parser.parse_args()
    if len(set(args.steps)) != len(args.steps):
        parser.error("Duplicate timesteps are not allowed")
    folder = args.output.resolve()
    if folder.exists():
        parser.error(
            f"Output folder already exists: {folder}. Choose a new --output name to preserve earlier evidence."
        )
    folder.mkdir(parents=True, exist_ok=False)
    fixed = model_hashes()
    report = {
        "completed": False,
        "fixed_model_and_controller_hashes": fixed,
        "protocol": {
            "force_x_n": args.force_x,
            "duration_s": 0.2,
            "baseline_s": 2.0,
            "recovery_s": 10.0,
            "real_time_factor": args.real_time_factor,
            "damping_model": args.damping_model,
            "controller_mode": args.controller_mode,
        },
        "trials": [],
    }
    report.update(
        requested_steps_s=args.steps,
        requested_repeats=args.repeats,
        screen_tolerances={
            "pitch_deg": PITCH_TOLERANCE_DEG,
            "joint_rad": JOINT_TOLERANCE_RAD,
        },
    )
    save = lambda: (folder / "comparison.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    save()
    try:
        if args.seed_comparison:
            seed = args.seed_comparison.resolve()
            previous = json.loads((seed / "comparison.json").read_text())
            if (
                not previous["trials"]
                or not all(r["recording_completed"] for r in previous["trials"])
                or previous["fixed_model_and_controller_hashes"] != fixed
                or previous["protocol"] != report["protocol"]
            ):
                raise ValueError(
                    "Seed data must contain completed recordings with matching model/controller and protocol"
                )
            for old in previous["trials"]:
                if (
                    old["physics_step_s"] not in args.steps
                    or old["repeat"] > args.repeats
                ):
                    continue
                source = Path(
                    old.get("recording_path", str(seed / old["case"]))
                ).resolve()
                audit = json.loads((source / "physics-audit.json").read_text())
                if not audit["passed"] or not old["recording_completed"]:
                    raise ValueError("Seed trial lacks a valid physics audit/recording")
                ankle = audit["push"]["joints"]["left_ankle"]
                report["trials"].append(
                    {
                        **old,
                        "physics_audit_passed": True,
                        "ankle_motor_input_nm": ankle.get(
                            "mean_motor_input_nm",
                            ankle["dart_force_before_constraints_nm"],
                        ),
                        "ankle_total_physics_input_nm": ankle[
                            "dart_force_before_constraints_nm"
                        ],
                        "recording_path": str(source),
                        "source": "prior matching comparison",
                    }
                )
            report["seed_comparison"] = str(seed)
            save()
        for repeat in range(1, args.repeats + 1):
            # Reverse order in the second repeat to reduce a simple run-order confound.
            steps = args.steps if repeat == 1 else list(reversed(args.steps))
            for step in steps:
                if any(
                    r["physics_step_s"] == step and r["repeat"] == repeat
                    for r in report["trials"]
                ):
                    continue
                name = f"{step*1000:g}ms-{repeat:02d}"
                case = folder / name
                print(
                    f"Starting {name}: fresh reference + {args.force_x:g} N push",
                    flush=True,
                )
                with (folder / f"{name}.log").open("w") as log:
                    child = subprocess.Popen(
                        [
                            sys.executable,
                            str(ROOT / "scripts/experiments/run_effort_push.py"),
                            "--physics-probe",
                            "--dart-probe",
                            "--real-time-factor",
                            str(args.real_time_factor),
                            "--force-x",
                            str(args.force_x),
                            "--damping-model",
                            args.damping_model,
                            "--controller-mode",
                            args.controller_mode,
                            "--physics-step",
                            str(step),
                            "--output",
                            str(case),
                        ],
                        stdout=log,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                    )
                    try:
                        code = child.wait(timeout=500)
                    finally:
                        if child.poll() is None:
                            # Let the owned runner clear the push and stop its simulator.
                            child.send_signal(signal.SIGINT)
                            try:
                                child.wait(timeout=25)
                            except subprocess.TimeoutExpired:
                                child.kill()
                                child.wait()
                if code:
                    raise RuntimeError(
                        f'{name} failed to record; inspect {folder / (name+".log")}'
                    )
                outcome = json.loads((case / "summary.json").read_text())
                audit = analyze(case)
                (case / "physics-audit.json").write_text(
                    json.dumps(audit, indent=2) + "\n"
                )
                measured = audit["push"]["physics_step_measured_s"]
                if abs(measured - step) > 1e-12:
                    raise RuntimeError(
                        f"Requested step {step} differs from DART {measured}"
                    )
                if model_hashes() != fixed:
                    raise RuntimeError("Model or controller changed during comparison")
                p = outcome["push"]
                ankle = audit["push"]["joints"]["left_ankle"]
                row = {
                    "case": name,
                    "repeat": repeat,
                    "physics_step_s": step,
                    "dart_step_s": measured,
                    "recording_completed": outcome["completed"],
                    "recovery_passed": outcome["passed"],
                    "physics_audit_passed": audit["passed"],
                    "final_pitch_change_deg": math.degrees(p["final_pitch_change_rad"]),
                    "peak_pitch_change_deg": math.degrees(p["peak_pitch_change_rad"]),
                    "final_joint_change_rad": p["final_joint_change_from_baseline_rad"],
                    "reference_peak_pitch_change_rad": outcome["reference"][
                        "peak_pitch_change_rad"
                    ],
                    "physics_input_impulse_n_s": audit["push"][
                        "observed_external_input_impulse_n_s"
                    ],
                    "published_window_s": p["command_window_s"],
                    "ankle_motor_input_nm": ankle.get(
                        "mean_motor_input_nm", ankle["dart_force_before_constraints_nm"]
                    ),
                    "ankle_total_physics_input_nm": ankle[
                        "dart_force_before_constraints_nm"
                    ],
                    "ankle_implicit_damping_nm": ankle["implicit_damping_nm"],
                    "ankle_constraint_torque_nm": ankle["joint_constraint_torque_nm"],
                    "ankle_transmitted_moment_nm": ankle["transmitted_joint_moment_nm"],
                    "gravity_per_ankle_nm": audit["push"][
                        "direct_ecm_gravity_per_ankle_nm"
                    ],
                    "ground_contact_evidence": p["continuous_contact_evidence"],
                    "saturated_controller_samples": p["saturated_controller_samples"],
                }
                report["trials"].append(row)
                save()
                print(json.dumps(row), flush=True)
                if not audit["passed"]:
                    raise RuntimeError(f"{name}: physics audit did not pass")
        impulses = [r["physics_input_impulse_n_s"] for r in report["trials"]]
        report["impulse_relative_spread"] = (max(impulses) - min(impulses)) / (
            sum(impulses) / len(impulses)
        )
        report["input_impulses_within_five_percent"] = (
            report["impulse_relative_spread"] <= 0.05
        )
        if set(args.steps) == {0.001, 0.0005, 0.00025} and args.repeats == 2:
            report["resolution_screen"] = summarize_resolution_study(report["trials"])
        report["completed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        save()
    print(
        "Timestep comparison complete. See " + str(folder / "comparison.json"),
        flush=True,
    )


if __name__ == "__main__":
    main()
