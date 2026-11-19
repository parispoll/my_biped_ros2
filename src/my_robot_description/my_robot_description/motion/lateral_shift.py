"""Small fixed-foot pelvis translation and quasi-static feedforward; no ROS.

This is a planned COM shift under bilateral contact, not force measurement or
single-leg balance. The finite feet admit other ground pressure distributions.
"""

import math
from types import SimpleNamespace
import xml.etree.ElementTree as ET
from my_robot_description.model.roll_stance import (
    standing_plan,
    double_support_loads,
    JOINT_NAMES,
    MOTOR_CAPS,
)
from my_robot_description.model.roll_geometry import link_transforms, model_com
from my_robot_description.model.sagittal_step import leg_angles

SCHEDULE = [
    ("baseline", 3.0),
    ("move_out", 4.0),
    ("hold", 6.0),
    ("move_back", 4.0),
    ("recovery", 8.0),
]
TOTAL_TIME = sum(t for _, t in SCHEDULE)


class LateralPlanner:
    def __init__(self, xml):
        self.xml = xml
        self.base = standing_plan(xml)
        root = ET.fromstring(xml)
        joint = lambda n: root.find(f"joint[@name='left_{n}']")
        self.model = SimpleNamespace(
            thigh=-float(joint("knee_pitch").find("origin").get("xyz").split()[2]),
            shin=-float(joint("ankle_pitch").find("origin").get("xyz").split()[2]),
            limits=tuple(
                (
                    float(joint(n).find("limit").get("lower")),
                    float(joint(n).find("limit").get("upper")),
                )
                for n in ["hip_pitch", "knee_pitch", "ankle_pitch"]
            ),
        )
        self.feet = link_transforms(
            xml, dict(zip(JOINT_NAMES, self.base["target_rad"]))
        )
        self.ankle_z = self.feet["left_foot"][2][3]
        self.foot_x = self.feet["left_foot"][0][3]
        self.half_spacing = abs(self.feet["left_foot"][1][3])
        self.roll_limits = {
            j.get("name"): (
                float(j.find("limit").get("lower")),
                float(j.find("limit").get("upper")),
            )
            for j in root.findall("joint")
            if j.get("name").endswith("_roll")
        }

    def pose(self, displacement):
        """Translate upright pelvis in world Y, keeping each foot frame unchanged."""
        if not math.isfinite(displacement) or abs(displacement) > 0.01000000001:
            raise ValueError(
                "Initial lateral experiment supports at most 10 mm pelvis travel"
            )
        return self.fixed_foot_pose(displacement)

    def fixed_foot_pose(self, displacement):
        """Shared geometry for derived experiments; all joint/torque checks remain."""
        if not math.isfinite(displacement) or abs(displacement) > 0.1:
            raise ValueError("Finite fixed-foot displacement up to 0.1 m required")
        hip_roll = math.atan2(-displacement, -self.ankle_z)
        sagittal_z = -math.hypot(displacement, self.ankle_z)
        hip, knee, ankle = leg_angles(self.model, self.foot_x, sagittal_z, 0.0)
        q = {
            f"{s}_{j}": v
            for s in ["left", "right"]
            for j, v in zip(
                ["hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"],
                [hip_roll, hip, knee, ankle, -hip_roll],
            )
        }
        for name, (low, high) in self.roll_limits.items():
            if not low <= q[name] <= high:
                raise ValueError("Roll limit exceeded")
        transforms = link_transforms(self.xml, q)
        mass, com = model_com(self.xml, q)
        predicted_com_y = com[1] + displacement
        fractions = dict(
            left=0.5 + predicted_com_y / (2 * self.half_spacing),
            right=0.5 - predicted_com_y / (2 * self.half_spacing),
        )
        if min(fractions.values()) <= 0:
            raise ValueError("Static model predicts loss of bilateral support")
        forces = {s: mass * 9.81 * f for s, f in fractions.items()}
        loads = double_support_loads(self.xml, q, forces)
        if any(abs(loads[n]) >= cap for n, cap in zip(JOINT_NAMES, MOTOR_CAPS)):
            raise ValueError("Planned feedforward exceeds existing motor budget")
        geometry_error = 0.0
        for side in ["left", "right"]:
            foot = transforms[f"{side}_foot"]
            old = self.feet[f"{side}_foot"]
            for i in range(3):
                geometry_error = max(
                    geometry_error,
                    abs(foot[i][3] + (displacement if i == 1 else 0.0) - old[i][3]),
                )
                for j in range(3):
                    geometry_error = max(geometry_error, abs(foot[i][j] - old[i][j]))
        return dict(
            target_rad=[q[n] for n in JOINT_NAMES],
            feedforward_nm=[loads[n] for n in JOINT_NAMES],
            pelvis_y_m=displacement,
            predicted_com_y_m=predicted_com_y,
            predicted_force_n=forces,
            max_fixed_foot_geometry_error=geometry_error,
            force_note="Static estimate at sole centers; not measured foot forces.",
        )


class ShiftProgram:
    def __init__(self, xml, displacement=0.01):
        if not math.isfinite(displacement) or not 0 < abs(displacement) <= 0.01:
            raise ValueError("Nonzero signed pelvis shift up to 10 mm required")
        self.planner = LateralPlanner(xml)
        self.displacement = displacement
        self.schedule = SCHEDULE
        # Cache static loads/FK before activation; no XML parsing in the control loop.
        self.table = [self.planner.pose(displacement * i / 40) for i in range(41)]
        self.setup = dict(
            displacement_m=displacement,
            schedule=SCHEDULE,
            total_time_s=TOTAL_TIME,
            endpoint=self.table[-1],
            interpolation="41-point static-load table, linearly interpolated by quintic time phase",
        )

    def sample(self, elapsed):
        """Return targets for elapsed simulated seconds; angles are radians and torques N m.

        This samples a prechecked plan. It does not read sensors or publish commands."""
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("Nonnegative finite program time required")
        start = 0.0
        phase = "done"
        u = 0.0
        for name, duration in self.schedule:
            if elapsed < start + duration:
                phase = name
                s = (elapsed - start) / duration
                v = s**3 * (10 + s * (-15 + 6 * s))
                u = (
                    v
                    if name == "move_out"
                    else (
                        1 - v if name == "move_back" else 1.0 if name == "hold" else 0.0
                    )
                )
                break
            start += duration
        point = u * 40
        i = min(39, int(point))
        f = point - i
        a, b = self.table[i], self.table[i + 1]
        blend = lambda key: [x + (y - x) * f for x, y in zip(a[key], b[key])]
        return dict(
            phase=phase,
            elapsed_s=elapsed,
            target_rad=blend("target_rad"),
            feedforward_nm=blend("feedforward_nm"),
            pelvis_y_m=self.displacement * u,
            predicted_com_y_m=a["predicted_com_y_m"]
            + (b["predicted_com_y_m"] - a["predicted_com_y_m"]) * f,
            predicted_force_n={
                s: a["predicted_force_n"][s]
                + (b["predicted_force_n"][s] - a["predicted_force_n"][s]) * f
                for s in ["left", "right"]
            },
        )
