"""Offline first-lift feasibility only. No ROS, commands or measured results.

Keep the pelvis and supporting foot at the verified zero-load endpoint. Solve
the other leg independently, using a flat-foot target. Static single support
uses a vertical reaction at the actual COM projection, not at both sole centers.
"""

import math
import xml.etree.ElementTree as ET
from my_robot_description.model.roll_geometry import (
    link_transforms,
    model_com,
    multiply,
    origin,
    vector,
)
from my_robot_description.model.roll_stance import (
    JOINT_NAMES,
    MOTOR_CAPS,
    double_support_loads,
)
from my_robot_description.model.roll_actuators import ACTIVE
from my_robot_description.model.sagittal_step import leg_angles
from my_robot_description.motion.zero_load import ZeroLoadProgram


def sole_corners(xml, transforms, side):
    """Collision-box bottom corners in the same frame as link transforms."""
    collision = ET.fromstring(xml).find(f"link[@name='{side}_foot']/collision")
    size = [float(v) for v in collision.find("geometry/box").get("size").split()]
    pose = multiply(transforms[side + "_foot"], origin(collision.find("origin")))
    return [
        [
            pose[i][3]
            + sum(
                pose[i][k] * v
                for k, v in enumerate(
                    [sx * size[0] / 2, sy * size[1] / 2, -size[2] / 2]
                )
            )
            for i in range(3)
        ]
        for sx, sy in [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    ]


def single_support_loads(xml, q, support):
    """Static torques for flat support, zero swing reaction, vertical COM-CoP.

    Whole-body moment balance sets CoP xy = COM xy. Correct the shared
    sole-center moment calculation for that pressure offset. This ignores
    acceleration, impact, frictional/slip dynamics and actuator dynamics.
    """
    if support not in ("left", "right"):
        raise ValueError("Named supporting side required")
    tf = link_transforms(xml, q)
    mass, com = model_com(xml, q)
    foot = tf[support + "_foot"]
    if max(abs(foot[i][j] - float(i == j)) for i in range(3) for j in range(3)) > 1e-9:
        raise ValueError("Single-support calculation requires a flat sole")
    corners = sole_corners(xml, tf, support)
    margin = min(com[k] - min(c[k] for c in corners) for k in (0, 1))
    margin = min(margin, *(max(c[k] for c in corners) - com[k] for k in (0, 1)))
    if margin < 0.01:
        raise ValueError("Single-support COM projection lacks 10 mm sole margin")
    forces = {s: mass * 9.81 if s == support else 0.0 for s in ("left", "right")}
    loads = double_support_loads(xml, q, forces)
    # Delta moment of moving the vertical support reaction to COM xy.
    delta = [
        (com[1] - foot[1][3]) * mass * 9.81,
        -(com[0] - foot[0][3]) * mass * 9.81,
        0.0,
    ]
    for joint in ET.fromstring(xml).findall("joint"):
        name = joint.get("name")
        if not name.startswith(support + "_"):
            continue
        frame = multiply(
            tf[joint.find("parent").get("link")], origin(joint.find("origin"))
        )
        axis = vector(joint.find("axis"), "xyz")
        loads[name] -= sum(
            sum(frame[i][k] * axis[k] for k in range(3)) * delta[i] for i in range(3)
        )
    return dict(
        feedforward_nm=[loads[n] for n in JOINT_NAMES],
        predicted_force_n=forces,
        predicted_com_local_m=com,
        predicted_cop_local_xy_m=com[:2],
        support_margin_m=margin,
    )


class FootLiftPreview:
    """Offline vertical lift/lower at the selected 2, 10 or 30 mm clearance."""

    clearance_m = 0.002
    move_time_s = 10.0
    hold_time_s = 2.0

    def __init__(self, xml, support="left", clearance=0.002, *, zero_program=None):
        if ACTIVE["name"] != "robotis_p54":
            raise ValueError("First-lift preview requires robotis_p54")
        if (
            support not in ("left", "right")
            or not math.isfinite(clearance)
            or clearance not in (0.002, 0.01, 0.03)
        ):
            raise ValueError(
                "First preview supports left/right support and 2, 10 or 30 mm"
            )
        self.move_time_s = 15.0 if clearance == 0.03 else 10.0
        self.clearance_m = clearance
        self.intervals = round(clearance / 0.00005)
        self.xml = xml
        self.support = support
        self.swing = "right" if support == "left" else "left"
        self.zero = zero_program or ZeroLoadProgram(xml, support)
        self.endpoint = self.zero.setup["endpoint"]
        self.initial_q = dict(zip(JOINT_NAMES, self.endpoint["target_rad"]))
        self.initial_tf = link_transforms(xml, self.initial_q)
        self.root = ET.fromstring(xml)
        self.table = [
            self.pose(clearance * i / self.intervals) for i in range(self.intervals + 1)
        ]
        # Derivative of cached joint targets w.r.t height times the maximum
        # quintic height speed. This is a sampled estimate, not a global bound.
        speed = (
            max(
                abs(b["target_rad"][i] - a["target_rad"][i])
                / (clearance / self.intervals)
                for a, b in zip(self.table, self.table[1:])
                for i in range(10)
            )
            * clearance
            * 1.875
            / self.move_time_s
        )
        if speed > 0.05:
            raise ValueError("Preview exceeds existing slow-motion reference speed")
        continuity = max(
            abs(a - b)
            for a, b in zip(
                self.table[0]["feedforward_nm"], self.endpoint["feedforward_nm"]
            )
        )
        if continuity > 1e-7:
            raise ValueError("Single-support feedforward is discontinuous at zero load")
        self.setup = dict(
            experiment_kind="foot_lift_preview",
            supporting_foot=support,
            swing_foot=self.swing,
            clearance_m=clearance,
            schedule=[
                ["lift", self.move_time_s],
                ["lifted_hold", self.hold_time_s],
                ["lower", self.move_time_s],
            ],
            total_time_s=2 * self.move_time_s + self.hold_time_s,
            sampled_peak_reference_speed_rad_s=speed,
            zero_to_single_support_feedforward_error_nm=continuity,
            zero_load_endpoint=self.endpoint,
            proposed_live_phases=[
                "lower_pelvis",
                "transfer",
                "confirm_unloaded",
                "lift",
                "confirm_clearance_and_no_contact",
                "lifted_hold",
                "lower_foot",
                "confirm_recontact",
                "reload",
                "center",
                "rise",
            ],
            scope="Offline kinematic and static predictions only; no Gazebo execution or motor commands.",
            live_lift_implemented=False,
            foot_lift_measured=False,
            outstanding=[
                "Full-body collision and impact checks",
                "Lift-specific live phase controller",
                "Independent measured sole-clearance and contact-transition audits",
                "Landing/reloading guards and measured mirrored trials",
                "Lifted-posture controller robustness",
            ],
        )

    def pose(self, height):
        if not math.isfinite(height) or not 0 <= height <= self.clearance_m + 1e-12:
            raise ValueError(
                "Preview sole height must be within the selected clearance"
            )
        q = self.initial_q.copy()
        foot = self.initial_tf[self.swing + "_foot"]
        hip = self.root.find(f"joint[@name='{self.swing}_hip_roll']")
        hip_y = vector(hip.find("origin"), "xyz")[1]
        dy = foot[1][3] - hip_y
        z = foot[2][3] + height
        roll = math.atan2(dy, -z)
        pitches = leg_angles(
            self.zero.planner.model, foot[0][3], -math.hypot(dy, z), 0.0
        )
        for name, value in zip(
            ["hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"],
            [roll, *pitches, -roll],
        ):
            q[self.swing + "_" + name] = value
        tf = link_transforms(self.xml, q)  # Also checks every URDF joint limit.
        geometry_error = 0.0
        corners = {}
        for side in ("left", "right"):
            desired = self.initial_tf[side + "_foot"]
            actual = tf[side + "_foot"]
            for i in range(3):
                geometry_error = max(
                    geometry_error,
                    abs(
                        actual[i][3]
                        - desired[i][3]
                        - (height if side == self.swing and i == 2 else 0.0)
                    ),
                )
                for j in range(3):
                    geometry_error = max(
                        geometry_error, abs(actual[i][j] - desired[i][j])
                    )
            c = sole_corners(self.xml, tf, side)
            base = sole_corners(self.xml, self.initial_tf, side)
            corners[side] = [
                [p[k] - base[i][k] for k in range(3)] for i, p in enumerate(c)
            ]
        if geometry_error > 1e-9:
            raise ValueError("Fixed support or flat vertical swing IK mismatch")
        statics = single_support_loads(self.xml, q, self.support)
        utilization = max(
            abs(t) / c for t, c in zip(statics["feedforward_nm"], MOTOR_CAPS)
        )
        if utilization > 0.8:
            raise ValueError("Single-support static load consumes motor reserve")
        return dict(
            target_rad=[q[n] for n in JOINT_NAMES],
            requested_clearance_m=height,
            sole_corner_displacements_m=corners,
            max_geometry_error_m=geometry_error,
            peak_static_motor_utilization=utilization,
            **statics,
        )

    def sample(self, elapsed):
        """Return targets for elapsed simulated seconds; angles are radians and torques N m.

        This samples a prechecked plan. It does not read sensors or publish commands."""
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("Nonnegative finite preview time required")
        if elapsed < self.move_time_s:
            phase = "lift"
            s = elapsed / self.move_time_s
        elif elapsed < self.move_time_s + self.hold_time_s:
            phase = "lifted_hold"
            s = 1.0
        elif elapsed < 2 * self.move_time_s + self.hold_time_s:
            phase = "lower"
            s = (2 * self.move_time_s + self.hold_time_s - elapsed) / self.move_time_s
        else:
            phase = "done"
            s = 0.0
        u = s**3 * (10 + s * (-15 + 6 * s))
        point = min(float(self.intervals), max(0.0, self.intervals * u))
        i = min(self.intervals - 1, int(point))
        f = point - i
        return dict(
            phase=phase,
            elapsed_s=elapsed,
            requested_clearance_m=u * self.clearance_m,
            target_rad=[
                a + (b - a) * f
                for a, b in zip(
                    self.table[i]["target_rad"], self.table[i + 1]["target_rad"]
                )
            ],
        )


def gravity_gradient_check(xml, q_values, support, epsilon=1e-6):
    """Independent virtual-work check: perturb joints with the support welded.

    Reconstruct base pose from the inverse support-foot transform. The gradient
    of total gravitational potential is the equilibrium motor torque. No contact
    reaction torques from single_support_loads are used in this reconstruction.
    """
    if (
        support not in ("left", "right")
        or len(q_values) != 10
        or not math.isfinite(epsilon)
        or epsilon <= 0
    ):
        raise ValueError(
            "Valid support, ten angles and positive difference step required"
        )

    def potential(values):
        q = dict(zip(JOINT_NAMES, values))
        tf = link_transforms(xml, q)[support + "_foot"]
        mass, com = model_com(xml, q)
        return mass * 9.81 * sum(tf[k][2] * (com[k] - tf[k][3]) for k in range(3))

    gradient = []
    for i in range(10):
        plus = list(q_values)
        minus = list(q_values)
        plus[i] += epsilon
        minus[i] -= epsilon
        gradient.append((potential(plus) - potential(minus)) / (2 * epsilon))
    expected = single_support_loads(xml, dict(zip(JOINT_NAMES, q_values)), support)[
        "feedforward_nm"
    ]
    error = max(abs(a - b) for a, b in zip(gradient, expected))
    return dict(
        passed=error < 1e-5,
        maximum_error_nm=error,
        gravity_gradient_nm=gradient,
        static_torques_nm=expected,
    )
