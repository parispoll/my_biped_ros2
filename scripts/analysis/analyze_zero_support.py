#!/usr/bin/env python3
"""Finite-difference URDF gravity/mass model at a saved zero-load endpoint.

Assumes one welded support sole, no impacts or saturation. This is a local
diagnostic, not nonlinear stability certification or a motor controller.
"""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import os, json, sys, argparse

parser = argparse.ArgumentParser(
    description="Local zero-load posture stiffness and sampled dynamics; no simulation commands"
)
parser.add_argument("--trial", type=str, required=True)
parser.add_argument("--output", required=True)
args = parser.parse_args()
os.environ["BIPED_ACTUATOR_PROFILE"] = "robotis_p54"
import xacro, numpy as np
from my_robot_description.model.roll_geometry import link_transforms, model_com
from my_robot_description.model.roll_stance import JOINT_NAMES
from my_robot_description.control.roll_posture import KP

xml = xacro.process_file("src/my_robot_description/urdf/biped_roll.urdf.xacro").toxml()
s = json.loads((Path(args.trial) / "joints.shift.json").read_text())
support = s["supporting_foot"]
offset = 0 if support == "left" else 5
q = np.array(s["endpoint"]["target_rad"])


def U(v):
    d = dict(zip(JOINT_NAMES, v))
    f = np.array(link_transforms(xml, d)[support + "_foot"])
    m, c = model_com(xml, d)
    R = f[:3, :3].T
    return m * 9.81 * (R @ (np.array(c) - f[:3, 3]))[2]


h = 1e-4
eye = np.eye(10)
H = np.zeros((10, 10))
u = U(q)
g = []
for i in range(10):
    g.append((U(q + h * eye[i]) - U(q - h * eye[i])) / (2 * h))
    for j in range(10):
        H[i, j] = (
            U(q + h * eye[i] + h * eye[j])
            - U(q + h * eye[i] - h * eye[j])
            - U(q - h * eye[i] + h * eye[j])
            + U(q - h * eye[i] - h * eye[j])
        ) / (4 * h * h)

import xml.etree.ElementTree as ET
from my_robot_description.model.roll_geometry import multiply, origin
from my_robot_description.control.roll_posture import KD, LATERAL_KI

bodies = {}
for l in ET.fromstring(xml).findall("link"):
    a = l.find("inertial")
    t = a.find("inertia")
    I = np.array(
        [
            [float(t.get("ixx")), float(t.get("ixy")), float(t.get("ixz"))],
            [float(t.get("ixy")), float(t.get("iyy")), float(t.get("iyz"))],
            [float(t.get("ixz")), float(t.get("iyz")), float(t.get("izz"))],
        ]
    )
    bodies[l.get("name")] = (
        float(a.find("mass").get("value")),
        np.array(origin(a.find("origin"))),
        I,
    )


def frames(v):
    tf = link_transforms(xml, dict(zip(JOINT_NAMES, v)))
    f = np.array(tf[support + "_foot"])
    R = f[:3, :3].T
    W = np.eye(4)
    W[:3, :3] = R
    W[:3, 3] = -R @ f[:3, 3]
    return {n: W @ np.array(tf[n]) @ o for n, (m, o, I) in bodies.items()}


base = frames(q)
eps = 1e-5
plus = [frames(q + eps * e) for e in eye]
minus = [frames(q - eps * e) for e in eye]
M = np.zeros((10, 10))
for n, (m, o, I) in bodies.items():
    Jv = np.column_stack(
        [(a[n][:3, 3] - b[n][:3, 3]) / (2 * eps) for a, b in zip(plus, minus)]
    )
    Jw = []
    for a, b in zip(plus, minus):
        W = (a[n][:3, :3] - b[n][:3, :3]) / (2 * eps) @ base[n][:3, :3].T
        Jw.append([W[2, 1], W[0, 2], W[1, 0]])
    Jw = np.array(Jw).T
    Iw = base[n][:3, :3] @ I @ base[n][:3, :3].T
    M += m * Jv.T @ Jv + Jw.T @ Iw @ Jw
print("M eigen", np.linalg.eigvalsh(M))
Mi = np.linalg.inv(M)
B = np.diag([0.2, 2.5, 3, 2.5, 0.2] * 2)
records = []
for stiffness, damping in [(40, None), (100, 2), (120, 2), (120, 8), (160, 8)]:
    kp = np.array(KP)
    kd = np.array(KD)
    kd[9 if support == "left" else 4] = 0.3
    if damping is not None:
        kp[offset + 3] = kp[offset + 4] = stiffness
        kd[offset + 3] = kd[offset + 4] = damping

    def step(s):
        x = s[:10].copy()
        v = s[10:20].copy()
        integral = s[20:].copy() - np.array(LATERAL_KI) * s[:10] * 0.004
        u = -kp * x - kd * v + integral
        for _ in range(4):
            v += 0.001 * (Mi @ (u - B @ v - H @ x))
            x += 0.001 * v
        return np.r_[x, v, integral]

    A = np.column_stack([step(e) for e in np.eye(30)])
    eig = np.linalg.eigvals(A)
    radius = float(max(abs(eig)))
    print(
        stiffness,
        damping,
        "min stiffness",
        min(np.linalg.eigvalsh(H + np.diag(kp))),
        "radius",
        radius,
        "top",
        sorted(eig, key=abs)[-4:],
    )
    records.append(
        dict(
            support_kp=stiffness,
            support_kd=damping,
            minimum_static_stiffness_eigenvalue=float(
                min(np.linalg.eigvalsh(H + np.diag(kp)))
            ),
            spectral_radius=radius,
        )
    )
Path(args.output).write_text(
    json.dumps(
        dict(
            mass_matrix=M.tolist(),
            gravity_hessian=H.tolist(),
            gravity_gradient=g,
            nominal_feedforward=s["endpoint"]["feedforward_nm"],
            cases=records,
            scope="Local fixed-support-foot, other-foot-free linearized model; no impacts or saturation; not nonlinear stability proof",
        ),
        indent=2,
    )
    + "\n"
)
