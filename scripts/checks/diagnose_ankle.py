#!/usr/bin/env python3
"""Compare pre-push and final-second ankle terms with a static gravity estimate."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import xml.etree.ElementTree as ET
import xacro
from my_robot_description.studies.ankle_diagnosis import ankle_gravity


def diagnose(folder, xml):
    summary = json.loads((folder / "push/recording/summary.json").read_text())
    snapshots = [
        json.loads(s)
        for s in (folder / "push/recording/samples.jsonl").read_text().splitlines()
    ]
    controller = [
        json.loads(s)
        for s in (folder / "push/controller.jsonl").read_text().splitlines()
    ]
    start = summary["events"][0]["sim_time"]
    push = next(
        e["sim_time"] for e in summary["events"] if e["event"] == "push_published"
    )
    end = snapshots[-1]["sim_time"]
    windows = {"baseline": (start, push - 0.001), "final_second": (end - 1, end)}
    root = ET.fromstring(xml)
    dynamics = root.find("joint[@name='left_ankle']/dynamics")
    report = {
        "recording": str(folder.resolve()),
        "windows": {},
        "model_basis": "Current default expanded effort URDF; geometry and dynamics must match the recording.",
    }
    for name, (lo, hi) in windows.items():
        s = [r for r in snapshots if lo <= r["sim_time"] <= hi]
        c = [r for r in controller if r["active"] and lo <= r["sim_time"] <= hi]
        if len(s) < 2 or len(c) < 2:
            raise ValueError(f"Insufficient data in {name}")
        if (
            max(abs(v) for row in c for v in row["velocity"]) > 0.01
            or max(abs(row["pitch_rate"]) for row in s) > 0.01
        ):
            raise ValueError(
                f"{name} is not stationary enough for this static estimate"
            )
        mean = lambda rows, key, i=None: sum(
            r[key] if i is None else r[key][i] for r in rows
        ) / len(rows)
        q = [mean(c, "position", i) for i in range(6)]
        pitch = mean(s, "pitch")
        gravity = ankle_gravity(xml, q, pitch)
        ankle = 2
        terms = {
            key: mean(c, key, ankle)
            for key in (
                "error",
                "feedforward",
                "proportional",
                "damping",
                "requested",
                "command",
                "velocity",
            )
        }
        residual = terms["command"] - gravity["gravity_torque_per_ankle_nm"]
        friction = float(dynamics.get("friction"))
        passive_damping = -float(dynamics.get("damping")) * terms["velocity"]
        report["windows"][name] = dict(
            controller_samples=len(c),
            sensor_samples=len(s),
            pelvis_pitch_rad=pitch,
            joint_positions_rad=q,
            ankle_terms=terms,
            **gravity,
            configured_ankle_friction_nm=friction,
            passive_ankle_damping_nm=passive_damping,
            unaccounted_static_moment_per_ankle_nm=residual,
            simple_friction_bound_sufficient=abs(residual) <= friction + 1e-3,
            equation_closure_nm=terms["command"]
            - (terms["feedforward"] + terms["proportional"] + terms["damping"]),
        )
    report["interpretation"] = [
        "Ankle terms are logged requests, not an independent calibrated actuator measurement.",
        "Static residual assumes symmetric planar, stationary motion and no external wrench.",
        "Residual is NOT measured friction. A failed bound means the simple model does not close.",
        "There is no integral term: a constant angle error and zero speed yield a constant command.",
    ]
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    xml = xacro.process_file(
        str(root / "src/my_robot_description/urdf/effort_standing.urdf.xacro")
    ).toxml()
    reports = [diagnose(folder, xml) for folder in args.recordings]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(reports, indent=2) + "\n")
    print(json.dumps(reports, indent=2))


if __name__ == "__main__":
    main()
