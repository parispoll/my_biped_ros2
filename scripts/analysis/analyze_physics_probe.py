#!/usr/bin/env python3
"""Check the final stationary second of a physics + DART probe experiment."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import csv
import json
from my_robot_description.model.stance import JOINT_NAMES
from my_robot_description.validation.physics_audit import (
    audit_joint,
    audit_explicit_joint,
    final_getter_window,
    stationary_physics_window,
)
from my_robot_description.studies.damping import validate_damping_trace


def analyze(folder):
    report = {}
    for name in ("reference", "push"):
        path = folder / name
        physics = list(csv.DictReader((path / "physics.csv").open()))
        dart = list(csv.DictReader((path / "dart.csv").open()))
        controller = [
            json.loads(s) for s in (path / "controller.jsonl").read_text().splitlines()
        ]
        damping = (
            list(csv.DictReader((path / "viscous.csv").open()))
            if (path / "viscous.csv").exists()
            else []
        )
        if damping and not validate_damping_trace(damping)["passed"]:
            raise ValueError("Invalid explicit damping trace")
        active = [s for s in controller if s["active"]]
        end = min(max(float(r["sim_time"]) for r in physics), active[-1]["sim_time"])
        c = [r for r in active if end - 1 <= r["sim_time"] <= end]
        joints = {}
        selected = {}
        for index, joint in enumerate(JOINT_NAMES):
            p = [
                r
                for r in physics
                if r["joint"] == joint and end - 1 <= float(r["sim_time"]) <= end
            ]
            # Getter is called once per physics step in this world; every tenth call is logged.
            candidates = [r for r in dart if r["joint"].split("/")[-1] == joint]
            d = final_getter_window(candidates)
            if (
                not p
                or not c
                or float(p[-1]["sim_time"]) - float(p[0]["sim_time"]) < 0.95
                or c[-1]["sim_time"] - c[0]["sim_time"] < 0.95
                or (int(d[-1]["sequence"]) - int(d[0]["sequence"]))
                * float(d[-1]["dt_s"])
                < 0.95
            ):
                raise ValueError(
                    "Audit requires approximately one complete steady simulated second"
                )
            selected[joint] = p
            if damping:
                v = [
                    r
                    for r in damping
                    if r["joint"] == joint and end - 1 <= float(r["sim_time"]) <= end
                ]
                joints[joint] = audit_explicit_joint(p, d, v, index)
            else:
                joints[joint] = audit_joint(p, d, c, index)
        ankles = [r for joint in ("left_ankle", "right_ankle") for r in selected[joint]]
        gravity = sum(float(r["gravity_per_ankle_nm"]) for r in ankles) / len(ankles)
        transmitted = (
            joints["left_ankle"]["gazebo_transmitted_world_y_nm"]
            + joints["right_ankle"]["gazebo_transmitted_world_y_nm"]
        ) / 2
        representative = [r for r in physics if r["joint"] == "left_ankle"]
        stationary = stationary_physics_window(list(selected.values()))
        gravity_applicable = not damping or stationary
        timesteps = {float(r["dt_s"]) for r in dart}
        if len(timesteps) != 1:
            raise ValueError("Variable physics timestep is not supported by this audit")
        dt = timesteps.pop()
        impulse = sum(
            float(r["external_fx_mean_n"]) * int(r["steps"]) * dt
            for r in representative
        )
        report[name] = {
            "joints": joints,
            "physics_step_measured_s": dt,
            "direct_ecm_gravity_per_ankle_nm": gravity,
            "mean_ankle_transmitted_world_y_nm": transmitted,
            "gravity_agreement_error_nm": transmitted - gravity,
            "static_gravity_check_applicable": gravity_applicable,
            "static_gravity_agreement_passed": (
                abs(transmitted - gravity) < 0.001 if gravity_applicable else None
            ),
            "observed_external_input_impulse_n_s": impulse,
            "external_input_zero_in_final_window": all(
                float(r["external_fx_last_n"]) == 0
                and float(r["external_fx_mean_n"]) == 0
                for r in ankles
            ),
            "window_basis": (
                "Final 1 simulated second: ECM/damping matched by simulation stamp; DART getter window scales with measured timestep and checks its own force/wrench closure."
                if damping
                else "Final 1 simulated second for ROS/ECM; DART getter sample count scales with its measured timestep, checked at the same stationary pose."
            ),
            "passed": all(j["passed"] for j in joints.values())
            and (not gravity_applicable or abs(transmitted - gravity) < 0.001)
            and all(j["external_force_final_n"] == 0 for j in joints.values()),
        }
    report["passed"] = all(report[name]["passed"] for name in ("reference", "push"))
    report["meaning"] = (
        "Physics-path audit, not push-recovery success. Explicit mode checks observed motor + passive torque sums; legacy mode checks stationary ROS/DART command agreement and split-step damping."
    )
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    args = parser.parse_args()
    report = analyze(args.folder)
    output = args.folder / "physics-audit.json"
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not report["passed"]:
        raise SystemExit("Physics audit checks failed")


if __name__ == "__main__":
    main()
