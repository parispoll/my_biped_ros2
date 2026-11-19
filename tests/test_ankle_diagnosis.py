"""Gravity-moment invariants and isolation of the optional friction diagnostic."""

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import xacro
from my_robot_description.studies.ankle_diagnosis import ankle_gravity
from my_robot_description.model.stance import model_from_urdf

MODEL = (
    Path(__file__).resolve().parents[1]
    / "src/my_robot_description/urdf/effort_standing.urdf.xacro"
)


class AnkleDiagnosisTests(unittest.TestCase):
    def setUp(self):
        self.xml = xacro.process_file(str(MODEL)).toxml()
        self.pose = list(model_from_urdf(self.xml).pose(0.4)) * 2

    def test_balanced_ideal_pose_has_zero_ankle_gravity_moment(self):
        self.assertAlmostEqual(
            ankle_gravity(self.xml, self.pose, 0)["gravity_torque_per_ankle_nm"], 0
        )

    def test_virtual_work_and_gravity_slope(self):
        pitch = 0.008

        def result(p):
            q = self.pose.copy()
            q[2] = q[5] = -p - q[0] - q[1]
            return ankle_gravity(self.xml, q, p)

        step = 1e-6
        before, center, after = (
            result(pitch - step),
            result(pitch),
            result(pitch + step),
        )
        # Increasing body pitch decreases the ankle joint angle by the same amount.
        torque = -(
            after["upper_body_potential_energy_j"]
            - before["upper_body_potential_energy_j"]
        ) / (4 * step)
        self.assertAlmostEqual(center["gravity_torque_per_ankle_nm"], torque, places=7)
        slope = (
            after["gravity_torque_per_ankle_nm"] - before["gravity_torque_per_ankle_nm"]
        ) / (2 * step)
        self.assertAlmostEqual(
            center["local_gravity_slope_per_ankle_nm_per_rad"], slope, places=7
        )

    def test_rejects_unmodelled_geometry_and_nonfinite_input(self):
        asymmetric = self.pose.copy()
        asymmetric[3] += 0.01
        for q, pitch in ((asymmetric, 0), (self.pose, 0.01), (self.pose, float("nan"))):
            with self.assertRaises(ValueError):
                ankle_gravity(self.xml, q, pitch)

    def test_zero_friction_override_changes_only_joint_friction(self):
        original = ET.fromstring(self.xml)
        modified = ET.fromstring(
            xacro.process_file(str(MODEL), mappings={"joint_friction": "0.0"}).toxml()
        )
        for first, second in zip(original.findall("joint"), modified.findall("joint")):
            self.assertEqual(first.find("dynamics").attrib["friction"], "0.2")
            self.assertEqual(float(second.find("dynamics").attrib["friction"]), 0)
            first.find("dynamics").set("friction", "0")
            second.find("dynamics").set("friction", "0")
            self.assertEqual(ET.tostring(first), ET.tostring(second))
        for tag in ("link", "ros2_control", "gazebo"):
            self.assertEqual(
                [ET.tostring(e) for e in original.findall(tag)],
                [ET.tostring(e) for e in modified.findall(tag)],
            )
