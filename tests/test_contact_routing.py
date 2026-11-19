"""Interactive feedback removes the relay; experiment topics stay unchanged."""

import unittest
from my_robot_description.runtime.contact_routing import feedback_topic


class ContactRoutingTests(unittest.TestCase):
    def test_direct_interactive_and_unchanged_experiments(self):
        for side in ["left", "right"]:
            self.assertEqual(
                feedback_topic(side, True, True), f"/biped/state/{side}_foot/contacts"
            )
            self.assertEqual(
                feedback_topic(side, True, False),
                f"/biped/state/balance/{side}_foot/contacts",
            )
            self.assertEqual(
                feedback_topic(side, False, False),
                f"/biped/balance/{side}_foot/contacts",
            )
        with self.assertRaises(ValueError):
            feedback_topic("left", False, True)
        with self.assertRaises(ValueError):
            feedback_topic("other", True, True)
