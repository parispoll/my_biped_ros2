"""Landing force permission cannot leak into raised-foot or unloading phases."""

import unittest
from my_robot_description.motion.touchdown import contact_force_limit


class TouchdownTests(unittest.TestCase):
    def test_near_floor_only_and_original_default(self):
        for phase in ["lift", "advance", "move_out", "transfer", "reload"]:
            self.assertEqual(contact_force_limit(phase, 0.0, 22.28, True), 0.5)
        for phase in ["lower_foot", "recontact"]:
            self.assertEqual(contact_force_limit(phase, 0.0005, 22.28, True), 0.5)
            self.assertEqual(contact_force_limit(phase, 0.0, 22.28, False), 0.5)
            self.assertAlmostEqual(
                contact_force_limit(phase, 0.0001, 22.28, True), 4.371336
            )
