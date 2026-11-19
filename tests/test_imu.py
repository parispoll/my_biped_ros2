"""Check interpretation of IMU data before it can be used for balance."""

import math
import unittest
from sensor_msgs.msg import Imu
from scripts.checks.check_imu import measurement, check_stationary


class ImuMeasurementTests(unittest.TestCase):
    def message(self):
        msg = Imu()
        msg.header.frame_id = "base_link"
        msg.orientation.w = 1.0
        msg.linear_acceleration.z = 9.81
        return msg

    def test_stationary_specific_force_includes_gravity(self):
        sample = measurement(self.message())
        check_stationary(sample)
        self.assertEqual(sample["pitch_radians"], 0)
        self.assertEqual(sample["acceleration"], (0, 0, 9.81))

    def test_pitch_signs_follow_positive_y_rotation(self):
        for pitch in (-0.2, 0.2):
            msg = self.message()
            msg.orientation.y = math.sin(pitch / 2)
            msg.orientation.w = math.cos(pitch / 2)
            self.assertAlmostEqual(measurement(msg)["pitch_radians"], pitch)

    def test_wrong_frame_rejected(self):
        msg = self.message()
        msg.header.frame_id = "unknown"
        with self.assertRaises(ValueError):
            measurement(msg)

    def test_unavailable_orientation_rejected(self):
        msg = self.message()
        msg.orientation_covariance[0] = -1.0
        with self.assertRaises(ValueError):
            measurement(msg)

    def test_invalid_quaternion_rejected(self):
        msg = self.message()
        msg.orientation.w = 0.0
        with self.assertRaises(ValueError):
            measurement(msg)

    def test_fallen_pose_fails_stationary_check(self):
        msg = self.message()
        msg.orientation.y = math.sin(0.1)
        msg.orientation.w = math.cos(0.1)
        with self.assertRaises(ValueError):
            check_stationary(measurement(msg))
