"""The inclusive 30 ms contact pairing limit uses ROS nanosecond resolution."""

import unittest
from unittest.mock import patch
from my_robot_description.motion.zero_load import paired_contact_states


class PairBoundaryTests(unittest.TestCase):
    def rows(self, right_stamp):
        return [
            dict(side=s, sim_time=t)
            for s, t in [
                ("right", right_stamp),
                ("left", 199.291),
                ("left", 199.311),
                ("right", 199.321),
                ("left", 199.331),
                ("right", 199.341),
            ]
        ]

    def state(self, row, now, side):
        return dict(
            side=side,
            sim_time=row["sim_time"],
            measurement_valid=True,
            state="loaded_contact",
            force_world_n=[0.0, 0.0, 100.0],
            cop_world_xy_m=[0.0, 0.0],
        )

    def test_exact_thirty_ms_is_accepted(self):
        self.assertGreater(199.311 - 199.281, 0.03)
        with patch("my_robot_description.motion.zero_load.contact_state", self.state):
            self.assertEqual(
                len(paired_contact_states(self.rows(199.281), 199.28, 199.35)), 3
            )

    def test_thirty_ms_plus_one_ns_is_rejected(self):
        with patch("my_robot_description.motion.zero_load.contact_state", self.state):
            with self.assertRaisesRegex(ValueError, "Stale paired"):
                paired_contact_states(self.rows(199.281 - 1e-9), 199.28, 199.35)
