"""Offline static support search; never used by the live controller.

Vertical reactions, flat soles and zero yaw moment are assumed. Optimize first
moments of pressure, avoiding division by force when a foot has zero load.
"""

import math
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import linprog
from my_robot_description.motion.lateral_shift import LateralPlanner
from my_robot_description.model.sagittal_step import leg_angles
from my_robot_description.model.roll_geometry import (
    link_transforms,
    model_com,
    multiply,
    origin,
    vector,
    rotation,
)
from my_robot_description.model.roll_stance import JOINT_NAMES, MOTOR_CAPS


def cross(a, b):
    return np.cross(a, b)


class SupportSearch:
    def __init__(self, xml, margin=0.01):
        self.xml = xml
        self.planner = LateralPlanner(xml)
        self.root = ET.fromstring(xml)
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        self.children = {n: [] for n in self.links}
        for j in self.joints.values():
            self.children[j.find("parent").get("link")].append(
                j.find("child").get("link")
            )

        def descendants(n):
            return [n, *[i for c in self.children[n] for i in descendants(c)]]

        self.distal = {
            n: descendants(j.find("child").get("link")) for n, j in self.joints.items()
        }
        self.half = {}
        for side in ("left", "right"):
            size = [
                float(x)
                for x in self.links[side + "_foot"]
                .find("collision/geometry/box")
                .get("size")
                .split()
            ]
            self.half[side] = (size[0] / 2 - margin, size[1] / 2 - margin)
        if (
            not math.isfinite(margin)
            or margin < 0
            or min(v for pair in self.half.values() for v in pair) <= 0
        ):
            raise ValueError("Invalid sole margin")
        self.margin = margin

    def pose(self, shift, lowering):
        if (
            not all(math.isfinite(v) for v in (shift, lowering))
            or not 0 <= lowering <= 0.06
            or abs(shift) > 0.12
        ):
            raise ValueError("Search domain: lowering 0–60 mm, shift ±120 mm")
        p = self.planner
        z = p.ankle_z + lowering
        roll = math.atan2(-shift, -z)
        pitches = leg_angles(p.model, p.foot_x, -math.hypot(shift, z), 0.0)
        q = dict(zip(JOINT_NAMES, [roll, *pitches, -roll] * 2))
        return self._assemble(q, shift, lowering, 0.0)

    def lean_pose(self, shift, lowering, pelvis_roll):
        """Independent left/right IK under a world-X pelvis rotation."""
        if (
            not all(math.isfinite(v) for v in (shift, lowering, pelvis_roll))
            or not 0 <= lowering <= 0.06
            or abs(shift) > 0.12
            or abs(pelvis_roll) > 0.35
        ):
            raise ValueError(
                "Lean domain: lowering 0–60 mm, shift ±120 mm, roll ±0.35 rad"
            )
        p = self.planner
        c = math.cos(pelvis_roll)
        s = math.sin(pelvis_roll)
        q = {}
        for side in ("left", "right"):
            old = p.feet[side + "_foot"]
            hip_y = old[1][3]
            # Inverse body rotation of the world-fixed ankle, then subtract hip.
            y = old[1][3] - shift
            z = old[2][3] + lowering
            local_y = c * y + s * z - hip_y
            local_z = -s * y + c * z
            hip_roll = math.atan2(local_y, -local_z)
            pitch = leg_angles(p.model, p.foot_x, -math.hypot(local_y, local_z), 0.0)
            q.update(
                {
                    side + "_" + n: v
                    for n, v in zip(
                        [
                            "hip_roll",
                            "hip_pitch",
                            "knee_pitch",
                            "ankle_pitch",
                            "ankle_roll",
                        ],
                        [hip_roll, *pitch, -pelvis_roll - hip_roll],
                    )
                }
            )
        return self._assemble(q, shift, lowering, pelvis_roll)

    def _assemble(self, q, shift, lowering, pelvis_roll):
        p = self.planner
        for n, j in self.joints.items():
            limit = j.find("limit")
            if not float(limit.get("lower")) <= q[n] <= float(limit.get("upper")):
                raise ValueError("Joint limit")
        body_rotation = rotation((1, 0, 0), pelvis_roll)
        tf = {
            n: multiply(body_rotation, t)
            for n, t in link_transforms(self.xml, q).items()
        }
        mass, local_com = model_com(self.xml, q)
        com = [
            sum(body_rotation[i][k] * local_com[k] for k in range(3)) for i in range(3)
        ]
        bodies = {
            n: (
                float(l.find("inertial/mass").get("value")),
                multiply(tf[n], origin(l.find("inertial/origin"))),
            )
            for n, l in self.links.items()
        }
        gravity = []
        matrix = []
        for n in JOINT_NAMES:
            j = self.joints[n]
            frame = multiply(tf[j.find("parent").get("link")], origin(j.find("origin")))
            axis = np.array(
                [
                    sum(
                        frame[i][k] * vector(j.find("axis"), "xyz")[k] for k in range(3)
                    )
                    for i in range(3)
                ]
            )
            at = np.array([frame[i][3] for i in range(3)])
            moment = np.zeros(3)
            for child in self.distal[n]:
                m, b = bodies[child]
                moment += cross(
                    np.array([b[i][3] for i in range(3)]) - at, [0, 0, -m * 9.81]
                )
            gravity.append(-float(axis @ moment))
            row = np.zeros(6)
            s = 0 if n.startswith("left") else 1
            foot = tf[("left", "right")[s] + "_foot"]
            arm = np.array([foot[i][3] for i in range(3)]) - at
            row[s] = -axis @ cross(arm, [0, 0, 1])
            row[2 + s] = -axis[0]
            row[4 + s] = axis[1]
            matrix.append(row)
        error = 0.0
        for side in ("left", "right"):
            new = tf[side + "_foot"]
            old = p.feet[side + "_foot"]
            for i in range(3):
                error = max(
                    error,
                    abs(
                        new[i][3]
                        + (shift if i == 1 else -lowering if i == 2 else 0)
                        - old[i][3]
                    ),
                )
                for k in range(3):
                    error = max(error, abs(new[i][k] - old[i][k]))
        if error > 1e-10:
            raise ValueError("Fixed-foot geometry mismatch")
        return dict(
            shift_m=shift,
            lowering_m=lowering,
            pelvis_roll_rad=pelvis_roll,
            q=q,
            tf=tf,
            mass=mass,
            com=com,
            gravity=np.array(gravity),
            matrix=np.array(matrix),
            geometry_error=error,
        )

    def solve(self, pose, right_fraction, pressure_reference=None, pressure_step=None):
        if not math.isfinite(right_fraction) or not 0 <= right_fraction <= 1:
            raise ValueError("Force fraction in [0,1] required")
        weight = pose["mass"] * 9.81
        tf = pose["tf"]
        com = pose["com"]
        a = pose["matrix"]
        g = pose["gravity"]
        # x=[F_L,F_R,F_L*dy_L,F_R*dy_R,F_L*dx_L,F_R*dx_R,peak_fraction].
        eq = np.zeros((4, 7))
        rhs = [weight, weight * com[1], weight * com[0], weight * right_fraction]
        eq[0, :2] = 1
        eq[1, :2] = [tf[s + "_foot"][1][3] for s in ("left", "right")]
        eq[1, 2:4] = 1
        eq[2, :2] = [tf[s + "_foot"][0][3] for s in ("left", "right")]
        eq[2, 4:6] = 1
        eq[3, 1] = 1
        ub = []
        bounds = []
        if (pressure_reference is None) != (pressure_step is None):
            raise ValueError("Pressure reference and step must be supplied together")
        if pressure_reference is not None:
            if not math.isfinite(pressure_step) or pressure_step < 0:
                raise ValueError("Nonnegative finite pressure step required")
            for i, side in enumerate(("left", "right")):
                ref = pressure_reference[side]
                for k, key in [(2 + i, "y_offset_m"), (4 + i, "x_offset_m")]:
                    value = ref[key]
                    if not math.isfinite(value):
                        raise ValueError("Finite reference pressure required")
                    for sign in (-1, 1):
                        row = np.zeros(7)
                        row[k] = sign
                        row[i] = -sign * value - pressure_step
                        ub.append(row)
                        bounds.append(0.0)

        for i, side in enumerate(("left", "right")):
            hx, hy = self.half[side]
            for k, half in [(2 + i, hy), (4 + i, hx)]:
                for sign in (-1, 1):
                    row = np.zeros(7)
                    row[k] = sign
                    row[i] = -half
                    ub.append(row)
                    bounds.append(0.0)
        for i, cap in enumerate(MOTOR_CAPS):
            for sign in (-1, 1):
                ub.append(np.r_[sign * a[i], -cap])
                bounds.append(-sign * g[i])
        solution = linprog(
            [0] * 6 + [1],
            A_ub=ub,
            b_ub=bounds,
            A_eq=eq,
            b_eq=rhs,
            bounds=[(0, None), (0, None)] + [(None, None)] * 4 + [(0, None)],
            method="highs",
        )
        if solution.status == 2:
            return None
        if not solution.success:
            raise RuntimeError("Pressure optimizer failed: " + solution.message)
        x = solution.x
        torques = g + a @ x[:6]
        cop = {}
        for i, s in enumerate(("left", "right")):
            cop[s] = (
                None
                if x[i] < 1e-7
                else dict(
                    x_offset_m=float(x[4 + i] / x[i]), y_offset_m=float(x[2 + i] / x[i])
                )
            )
        residual = float(np.max(np.abs(eq @ x - np.array(rhs))))
        return dict(
            pelvis_roll_rad=pose["pelvis_roll_rad"],
            pelvis_shift_m=pose["shift_m"],
            pelvis_lowering_m=pose["lowering_m"],
            pelvis_height_m=self.planner.base["expected_pelvis_height_m"]
            - pose["lowering_m"],
            com_world_m=[
                com[0],
                com[1] + pose["shift_m"],
                com[2]
                + self.planner.base["expected_pelvis_height_m"]
                - pose["lowering_m"],
            ],
            right_fraction=right_fraction,
            forces_n=dict(left=float(x[0]), right=float(x[1])),
            pressure_offsets=cop,
            first_moments_n_m=x[2:6].tolist(),
            torques_nm=dict(zip(JOINT_NAMES, torques.tolist())),
            peak_utilization=float(
                max(abs(t) / c for t, c in zip(torques, MOTOR_CAPS))
            ),
            joint_targets_rad=pose["q"],
            equilibrium_residual=residual,
            fixed_foot_error_m=pose["geometry_error"],
        )
