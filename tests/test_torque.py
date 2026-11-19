"""Torque-law bounds, pendulum physics, and separation from biped position control."""

from pathlib import Path
import math
import unittest
import xml.etree.ElementTree as ET
import xacro
from my_robot_description.model.torque import clamp_torque, pendulum_from_urdf

PACKAGE = Path(__file__).resolve().parents[1] / "src/my_robot_description"


class TorqueBenchTests(unittest.TestCase):
    def setUp(self):
        self.xml = xacro.process_file(
            str(PACKAGE / "urdf/torque_bench.urdf.xacro")
        ).toxml()
        self.model = pendulum_from_urdf(self.xml)

    def test_positive_and_negative_commands_are_bounded(self):
        self.assertEqual(clamp_torque(10, 0.25), 0.25)
        self.assertEqual(clamp_torque(-10, 0.25), -0.25)
        self.assertEqual(clamp_torque(0.1, 0.25), 0.1)

    def test_nonfinite_and_invalid_limits_are_rejected(self):
        for command, limit in ((float("nan"), 2), (1, 0), (1, float("inf"))):
            with self.assertRaises(ValueError):
                clamp_torque(command, limit)

    def test_gravity_is_restoring_in_both_directions(self):
        self.assertLess(self.model.acceleration(0.3, 0, 0), 0)
        self.assertGreater(self.model.acceleration(-0.3, 0, 0), 0)

    def test_feedforward_holds_static_gravity_when_unsaturated(self):
        for angle in (-0.5, 0.5):
            command = self.model.requested_torque(angle, 0, angle)
            self.assertAlmostEqual(self.model.acceleration(angle, 0, command), 0)

    def test_too_small_motor_cannot_hold_requested_angle(self):
        command = self.model.requested_torque(0.5, 0, 0.5)
        self.assertGreater(command, 0.25)
        self.assertLess(self.model.acceleration(0.5, 0, clamp_torque(command, 0.25)), 0)

    def test_model_inertia_includes_parallel_axis_term(self):
        expected = (3 * (0.035 * 0.95) ** 2 + 0.26**2) / 12 + (0.26 / 2) ** 2
        self.assertAlmostEqual(self.model.inertia, expected)

    def test_only_bench_uses_effort_interface(self):
        root = ET.fromstring(self.xml)
        self.assertEqual(
            root.find("ros2_control/joint/command_interface").get("name"), "effort"
        )
        normal = ET.fromstring(
            xacro.process_file(str(PACKAGE / "urdf/my_biped.urdf.xacro")).toxml()
        )
        self.assertTrue(
            all(
                e.get("name") == "position"
                for e in normal.findall("ros2_control/joint/command_interface")
            )
        )
