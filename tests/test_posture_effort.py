"""Verify torque signs, saturation, and isolation from position mode."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import xacro
from my_robot_description.control.posture_effort import (
    posture_command,
    effort_limits_from_urdf,
    quaternion_tilt,
)

PACKAGE = Path(__file__).resolve().parents[1] / "src/my_robot_description"


class PostureEffortTests(unittest.TestCase):
    def test_tilt_detects_lean_and_rejects_invalid_quaternion(self):
        import math

        self.assertAlmostEqual(quaternion_tilt(0, math.sin(0.1), 0, math.cos(0.1)), 0.2)
        self.assertEqual(quaternion_tilt(0, 0, 0, 1), 0)
        for q in ((0, 0, 0, 0), (float("nan"), 0, 0, 1)):
            with self.assertRaises(ValueError):
                quaternion_tilt(*q)

    def command(self, **updates):
        values = dict(
            target=[0.0] * 6,
            position=[0.0] * 6,
            velocity=[0.0] * 6,
            feedforward=[-2.5] * 6,
            kp=[10.0] * 6,
            kd=[2.0] * 6,
            limits=[6.0] * 6,
        )
        values.update(updates)
        return posture_command(**values)

    def test_exact_pose_keeps_gravity_feedforward(self):
        self.assertEqual(self.command()["command"], [-2.5] * 6)

    def test_angle_and_velocity_corrections_have_opposing_signs(self):
        result = self.command(position=[-0.1] * 6, velocity=[0.2] * 6)
        self.assertAlmostEqual(result["proportional"][0], 1.0)
        self.assertAlmostEqual(result["damping"][0], -0.4)
        self.assertAlmostEqual(result["command"][0], -1.9)

    def test_large_errors_saturate_both_directions(self):
        self.assertEqual(self.command(target=[10.0] * 6)["command"], [6.0] * 6)
        self.assertEqual(self.command(target=[-10.0] * 6)["command"], [-6.0] * 6)

    def test_invalid_feedback_is_rejected(self):
        for update in (
            dict(position=[float("nan")] * 6),
            dict(velocity=[]),
            dict(kp=[-1.0] * 6),
        ):
            with self.assertRaises(ValueError):
                self.command(**update)

    def test_effort_limits_and_identical_physical_robot(self):
        position = ET.fromstring(
            xacro.process_file(str(PACKAGE / "urdf/my_biped.urdf.xacro")).toxml()
        )
        effort_xml = xacro.process_file(
            str(PACKAGE / "urdf/effort_standing.urdf.xacro")
        ).toxml()
        effort = ET.fromstring(effort_xml)
        self.assertEqual(effort_limits_from_urdf(effort_xml), [6.0, 10.0, 6.0] * 2)
        for tag in ("link", "joint"):
            self.assertEqual(
                [ET.tostring(e) for e in position.findall(tag)],
                [ET.tostring(e) for e in effort.findall(tag)],
            )
        self.assertTrue(
            all(
                e.get("name") == "position"
                for e in position.findall("ros2_control/joint/command_interface")
            )
        )
        self.assertTrue(
            all(
                e.get("name") == "effort"
                for e in effort.findall("ros2_control/joint/command_interface")
            )
        )
