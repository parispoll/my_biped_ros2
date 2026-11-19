"""Check moment balance against an independent potential-energy calculation."""

import math
from pathlib import Path
import unittest
import xacro
from my_robot_description.model.stance import model_from_urdf
from my_robot_description.model.statics import standing_loads

MODEL = (
    Path(__file__).resolve().parents[1]
    / "src/my_robot_description/urdf/my_biped.urdf.xacro"
)


class StaticsTests(unittest.TestCase):
    def setUp(self):
        self.model = model_from_urdf(xacro.process_file(str(MODEL)).toxml())

    def test_straight_pose_needs_no_motor_moment(self):
        report = standing_loads(self.model, 0)
        self.assertAlmostEqual(
            2 * report["vertical_ground_force_per_foot_n"], self.model.mass * 9.81
        )
        for torque in report["joint_torques_nm"].values():
            self.assertAlmostEqual(torque, 0)

    def test_crouch_symmetry_and_sign(self):
        torques = standing_loads(self.model, 0.4)["joint_torques_nm"]
        self.assertLess(torques["left_knee"], -2)
        for joint in ("hip", "knee", "ankle"):
            self.assertAlmostEqual(torques[f"left_{joint}"], torques[f"right_{joint}"])
        self.assertAlmostEqual(torques["left_hip"], 0)
        self.assertAlmostEqual(torques["left_ankle"], 0)

    def test_virtual_work_matches_potential_energy(self):
        m = self.model

        def energy(knee):
            hip, knee, _ = m.pose(knee)
            pelvis_z = m.geometry(m.pose(knee))["expected_pelvis_height"]
            thigh_z = pelvis_z - m.thigh / 2 * math.cos(hip)
            shin_z = (
                pelvis_z - m.thigh * math.cos(hip) - m.shin / 2 * math.cos(hip + knee)
            )
            # Constant base COM offset does not affect the energy derivative.
            return 9.81 * (
                m.base_mass * pelvis_z
                + 2
                * (
                    m.thigh_mass * thigh_z
                    + m.shin_mass * shin_z
                    + m.foot_mass * m.foot_thickness / 2
                )
            )

        for knee in (0.1, 0.4, 0.6):
            step = 1e-6
            before, after = m.pose(knee - step), m.pose(knee + step)
            q_derivative = [(b - a) / (2 * step) for a, b in zip(before, after)]
            torques = standing_loads(m, knee)["per_leg"]
            motor_work = 2 * sum(
                torques[name]["motor_torque_nm"] * dq
                for name, dq in zip(("hip", "knee", "ankle"), q_derivative)
            )
            self.assertAlmostEqual(
                motor_work,
                (energy(knee + step) - energy(knee - step)) / (2 * step),
                places=6,
            )

    def test_rejects_invalid_inputs(self):
        for gravity in (0, -1, float("nan")):
            with self.assertRaises(ValueError):
                standing_loads(self.model, 0.4, gravity)
        with self.assertRaises(ValueError):
            standing_loads(self.model, -0.1)
