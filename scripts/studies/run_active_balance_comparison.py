#!/usr/bin/env python3
"""Run fresh off/on pairs for +/-2 N, optionally reusing one completed active case."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import hashlib
import json
import subprocess

ROOT = Path(__file__).resolve().parents[2]
TRACKED = (
    "nodes/legacy/hold_effort_stance.py",
    "nodes/sensing/sample_contacts.py",
    "my_robot_description/sensing/balance.py",
    "my_robot_description/control/balance_control.py",
    "my_robot_description/control/integral.py",
    "my_robot_description/control/feedback_timing.py",
    "config/effort_standing.yaml",
    "config/balance_control.yaml",
    "urdf/components/properties.xacro",
    "urdf/components/leg.xacro",
    "plugins/viscous_damping.cpp",
    "worlds/standing.sdf",
    "urdf/effort_standing.urdf.xacro",
    "urdf/components/control_effort.xacro",
    "config/balance_observer.yaml",
    "launch/effort_standing.launch.py",
    "my_robot_description/validation/balance_audit.py",
    "my_robot_description/control/posture_effort.py",
    "my_robot_description/control/posture_setup.py",
)


def hashes():
    return {
        p: hashlib.sha256(
            (ROOT / "src/my_robot_description" / p).read_bytes()
        ).hexdigest()
        for p in TRACKED
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed-forward-active", type=Path)
    parser.add_argument(
        "--reverse-order",
        action="store_true",
        help="Reverse directions and off/on order for a repeat",
    )
    args = parser.parse_args()
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    plan = [("forward", 2.0, ("on", "off")), ("backward", -2.0, ("off", "on"))]
    if args.reverse_order:
        plan = [(d, f, tuple(reversed(m))) for d, f, m in reversed(plan)]
    report = dict(
        completed=False,
        source_hashes=hashes(),
        cases={},
        execution_order=[f"{d}-{m}" for d, _, modes in plan for m in modes],
    )
    save = lambda: (folder / "validation.json").write_text(
        json.dumps(report, indent=2) + "\n"
    )
    save()
    try:
        for direction, force, modes in plan:
            paths = {}
            for mode in modes:
                name = f"{direction}-{mode}"
                case = folder / name
                if direction == "forward" and mode == "on" and args.seed_forward_active:
                    case = args.seed_forward_active.resolve()
                    s = json.loads((case / "summary.json").read_text())
                    if (
                        json.loads((case / "source_hashes.json").read_text())
                        != report["source_hashes"]
                    ):
                        raise ValueError(
                            "Seed source snapshot does not match the comparison"
                        )
                    if (
                        not s["completed"]
                        or not s["passed"]
                        or not s["push"].get("balance_enabled")
                        or s["push"]["requested"]["force_x"] != 2.0
                    ):
                        raise ValueError(
                            "Seed must be a completed passing forward active trial"
                        )
                else:
                    print(f"Running {name}", flush=True)
                    command = [
                        sys.executable,
                        str(ROOT / "scripts/experiments/run_effort_push.py"),
                        "--damping-model",
                        "explicit",
                        "--controller-mode",
                        "pid",
                        "--real-time-factor",
                        ".25",
                        "--force-x",
                        str(force),
                        "--output",
                        str(case),
                        "--balance-enabled" if mode == "on" else "--balance-observer",
                        "--posture-delay",
                        "0" if mode == "on" else "3",
                    ]
                    with (folder / f"{name}.log").open("w") as stream:
                        subprocess.run(
                            command, stdout=stream, stderr=subprocess.STDOUT, check=True
                        )
                if hashes() != report["source_hashes"]:
                    raise RuntimeError(
                        "Control/model sources changed during comparison"
                    )
                paths[mode] = case
                report["cases"][name] = str(case)
                save()
            destination = folder / f"{direction}-comparison"
            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "scripts/analysis/compare_active_balance.py"),
                    "--off",
                    str(paths["off"]),
                    "--on",
                    str(paths["on"]),
                    "--output",
                    str(destination),
                ],
                check=True,
            )
            report[direction] = json.loads(
                (destination / "comparison.json").read_text()
            )
            save()
        report["completed"] = True
        report["all_comparisons_valid"] = all(
            report[d]["valid_comparison"] for d in ("forward", "backward")
        )
        report["peak_improvement_both_directions"] = all(
            report[d]["peak_lean_improvement_observed"] for d in ("forward", "backward")
        )
    finally:
        save()


if __name__ == "__main__":
    main()
