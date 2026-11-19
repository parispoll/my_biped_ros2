#!/usr/bin/env python3
"""Re-audit immutable raw lateral records with the recorded actuator profile.

Preserves original summary.json; rejects changes in model/control source files.
"""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import csv
import hashlib
import json
import os

ROOT = Path(__file__).resolve().parents[2]


def read(p):
    return [json.loads(line) for line in p.read_text().splitlines()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    a = parser.parse_args()
    p = a.folder.resolve()
    original = json.loads((p / "summary.json").read_text())
    os.environ["BIPED_ACTUATOR_PROFILE"] = original.get("actuator_profile", {}).get(
        "name", "legacy"
    )
    sys.path.insert(0, str(ROOT / "src/my_robot_description"))
    import xacro
    from my_robot_description.model.roll_actuators import ACTIVE
    from my_robot_description.motion.partial_unload import (
        PartialUnloadProgram,
        audit_load_guards,
    )
    from my_robot_description.validation.lateral_audit import audit_shift, audit_sensors
    from my_robot_description.sensing.foot_loads import audit_foot_loads
    from my_robot_description.validation.roll_audit import audit_damping

    hashes = json.loads((p / "source-hashes.json").read_text())
    physical = {
        f: v
        for f, v in hashes.items()
        if "/urdf/" in f
        or f.endswith((".yaml", ".cpp"))
        or Path(f).name
        in [
            "roll_geometry.py",
            "roll_stance.py",
            "roll_posture.py",
            "roll_actuators.py",
            "lateral_shift.py",
        ]
    }
    if not all(
        hashlib.sha256((ROOT / f).read_bytes()).hexdigest() == h
        for f, h in physical.items()
    ):
        raise ValueError(
            "Recorded model/controller sources differ; cannot replay against this model"
        )
    if original.get("actuator_profile", ACTIVE) != ACTIVE:
        raise ValueError("Recorded actuator data differs from selected profile")
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    program = None
    windows = None
    if original.get("unloading_plan"):
        plan = original["unloading_plan"]
        if plan.get("program_kind") == "load_feedback":
            from my_robot_description.motion.load_feedback import (
                LoadTransferProgram,
                audit_load_feedback,
            )

            bias = plan["reference_bias_world_m"] * (
                1 if plan["direction"] == "left" else -1
            )
            program = LoadTransferProgram(
                xml,
                plan["target_unloaded_fraction"],
                plan["direction"],
                plan["feedback_enabled"],
                bias,
            )
        else:
            program = PartialUnloadProgram(
                xml, plan["target_unloaded_fraction"], plan["direction"]
            )
        windows = program.setup["analysis_windows"]
    rows = read(p / "joints.jsonl")
    com = read(p / "com.jsonl")
    shift = audit_shift(
        rows, com, xml, original["requested_pelvis_shift_m"], program=program
    )
    start, end = shift["origin_sim_time"], shift["end_sim_time"]
    height = json.loads((p / "joints.setup.json").read_text())[
        "expected_pelvis_height_m"
    ]
    sensors = audit_sensors(
        read(p / "sensors.jsonl"),
        read(p / "imu-raw.jsonl"),
        start,
        end,
        height,
        original["requested_pelvis_shift_m"],
        windows=windows,
    )
    with (p / "viscous.csv").open() as f:
        damping = audit_damping(list(csv.DictReader(f)))
    mass = shift["com_coverage"]["expected_mass_kg"]
    loads = audit_foot_loads(
        read(p / "contact-wrenches.jsonl"),
        com,
        start,
        end,
        mass,
        original["requested_pelvis_shift_m"],
        phase_windows=windows,
    )
    guard = audit_load_guards(rows, start, end, mass) if program else None
    reports = dict(
        shift=shift, sensors=sensors, physical_damping=damping, foot_loads=loads
    )
    if guard:
        reports["load_guard"] = guard
    if program and program.setup.get("program_kind") == "load_feedback":
        reports["load_feedback"] = audit_load_feedback(rows, program, start, end)
    report = dict(
        passed=original.get("source_unchanged", False)
        and all(r["passed"] for r in reports.values()),
        actuator_profile=ACTIVE,
        reason="Expected COM mass derived from the recorded model instead of fixed 14.42 kg; acceptance thresholds unchanged",
        original_summary_sha256=hashlib.sha256(
            (p / "summary.json").read_bytes()
        ).hexdigest(),
        physical_sources_verified=list(physical),
        audit_source_sha256=hashlib.sha256(
            (
                ROOT
                / "src/my_robot_description/my_robot_description/validation/lateral_audit.py"
            ).read_bytes()
        ).hexdigest(),
        **reports
    )
    with (p / "reaudit.json").open("x") as f:
        json.dump(report, f, indent=2)
        f.write("\n")
    print(
        json.dumps(
            dict(
                passed=report["passed"],
                expected_mass_kg=mass,
                measured_mass_kg=shift["com_coverage"]["mass_kg"],
                checks={k: r["passed"] for k, r in reports.items()},
            ),
            indent=2,
        )
    )
    if not report["passed"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
