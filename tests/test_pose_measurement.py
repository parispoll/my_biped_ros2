"""Independent tilt and timestamp checks for the Gazebo pose monitor."""

import math
import unittest
from scripts.checks.check_standing import parse_pose


class PoseMeasurementTests(unittest.TestCase):
    def message(self, rotation):
        return (
            'header {\n stamp { sec: 12 nsec: 500000000 }\n}\npose {\n name: "my_biped"\n position { z: 0.5545 }\n orientation { '
            + rotation
            + " }\n}\n"
        )

    def test_upright_height_and_simulation_timestamp(self):
        sample = parse_pose(self.message("w: 1"))
        self.assertEqual(sample["sim_time"], 12.5)
        self.assertEqual(sample["height"], 0.5545)
        self.assertEqual(sample["tilt_radians"], 0)

    def test_detects_pitch(self):
        sample = parse_pose(self.message(f"y: {math.sin(0.1)} w: {math.cos(0.1)}"))
        self.assertAlmostEqual(sample["tilt_radians"], 0.2)

    def test_yaw_does_not_count_as_falling(self):
        sample = parse_pose(self.message(f"z: {math.sin(0.3)} w: {math.cos(0.3)}"))
        self.assertAlmostEqual(sample["tilt_radians"], 0)

    def test_missing_robot_is_not_a_measurement(self):
        self.assertIsNone(parse_pose('pose { name: "ground" }'))

    def test_invalid_quaternion_rejected(self):
        with self.assertRaises(ValueError):
            parse_pose(self.message("w: 0"))
