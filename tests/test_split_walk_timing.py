"""Weight transfer and foot swing clocks can change independently."""

import unittest
from my_robot_description.motion.walking_profile import WalkingProfile


class SplitTimingTests(unittest.TestCase):
    def test_profile_and_invalid_transfer(self):
        self.assertEqual(
            WalkingProfile(3, 0.075, 0.25, 0.5).validate().effective_transfer_scale, 0.5
        )
        self.assertEqual(
            WalkingProfile(3, 0.075, 0.25).validate().effective_transfer_scale, 0.25
        )
        for value in [True, float("nan"), -0.1, 0.1]:
            with self.assertRaises(ValueError):
                WalkingProfile(3, 0.075, 0.25, value).validate()

    def test_velocity_feedforward_requires_boolean(self):
        with self.assertRaises(ValueError):
            WalkingProfile(3, 0.075, 0.25, 0.5, 1).validate()
