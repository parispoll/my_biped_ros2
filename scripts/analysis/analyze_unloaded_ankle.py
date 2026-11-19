#!/usr/bin/env python3
"""Approximate free-foot sampled ankle dynamics; not full-robot stability proof."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse, json, os

os.environ["BIPED_ACTUATOR_PROFILE"] = "robotis_p54"
import xacro, numpy as np, xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def analyze():
    root = ET.fromstring(
        xacro.process_file(
            str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
        ).toxml()
    )
    link = root.find("link[@name='right_foot']/inertial")
    mass = float(link.find("mass").get("value"))
    _, y, z = map(float, link.find("origin").get("xyz").split())
    inertia = float(link.find("inertia").get("ixx")) + mass * (y * y + z * z)
    rows = []
    for kd in [2.0, 0.3]:

        def advance(s):
            q, v, integral = s
            integral -= 5 * q * 0.004
            u = -40 * q - kd * v + integral
            for _ in range(4):
                v += 0.001 * (u - 0.2 * v) / inertia
                q += 0.001 * v
            return [q, v, integral]

        matrix = np.column_stack([advance(np.eye(3)[i]) for i in range(3)])
        eigen = np.linalg.eigvals(matrix)
        rows.append(
            dict(
                kd=kd,
                spectral_radius=float(max(abs(eigen))),
                eigenvalues=[[float(v.real), float(v.imag)] for v in eigen],
                locally_stable=bool(max(abs(eigen)) < 1),
            )
        )
    return dict(
        inertia_about_ankle_roll_kg_m2=inertia,
        control_step_s=0.004,
        physics_step_s=0.001,
        physical_damping=0.2,
        kp=40.0,
        ki=5.0,
        cases=rows,
        scope="Small free-foot linear approximation, parent fixed, gravity feedforward cancelled; ignores full-body coupling, contact impacts, saturation and delay. Gazebo validation still required.",
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    r = analyze()
    a.output.write_text(json.dumps(r, indent=2) + "\n")
    print(json.dumps(r, indent=2))
