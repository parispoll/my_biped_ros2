"""Regression checks for test contact state. See the folder README for its role."""

import copy, unittest
from test_foot_loads import sample
from my_robot_description.sensing.contact_state import contact_state
from my_robot_description.sensing.foot_loads import decode_foot_load


class ContactStateTests(unittest.TestCase):
    def test_missing_and_expired_data_never_mean_zero(self):
        self.assertEqual(contact_state(None, 1.0, "left")["state"], "unknown")
        row = sample("left", 1.0, 70.0)
        row["message"]["contacts"] = []
        r = contact_state(row, 1.1, "left")
        self.assertEqual(r["state"], "stale")
        self.assertIsNone(r["force_world_n"])

    def test_only_fresh_explicit_empty_means_no_contact(self):
        row = sample("left", 1.0, 70.0)
        row["message"]["contacts"] = []
        r = contact_state(row, 1.02, "left")
        self.assertEqual(r["state"], "no_contact")
        self.assertEqual(r["force_world_n"], [0.0, 0.0, 0.0])
        self.assertIsNone(r["cop_world_xy_m"])

    def test_low_force_is_touching_not_absent(self):
        r = contact_state(sample("left", 1.0, 0.5), 1.0, "left")
        self.assertEqual(r["state"], "touching_unloaded")
        self.assertIsNotNone(r["cop_world_xy_m"])
        self.assertEqual(
            contact_state(sample("left", 1.0, 70.0), 1.0, "left")["state"],
            "loaded_contact",
        )

    def test_explicit_zero_manifold_has_no_pressure_point(self):
        row = sample("left", 1.0, 0.0)
        with self.assertRaises(ValueError):
            decode_foot_load(row)
        r = contact_state(row, 1.0, "left")
        self.assertEqual(r["state"], "touching_unloaded")
        self.assertIsNone(r["cop_world_xy_m"])

    def test_bad_contact_is_invalid_not_air(self):
        for kind in ["missing", "reverse", "frame", "future", "side"]:
            row = sample("left", 1.0, 0.0)
            if kind == "missing":
                row["message"]["contacts"][0]["wrenches"] = []
            if kind == "reverse":
                c = row["message"]["contacts"][0]
                c["collision1"], c["collision2"] = c["collision2"], c["collision1"]
            if kind == "frame":
                row["message"]["header"]["frame_id"] = "foot"
            if kind == "future":
                row["message"]["header"]["stamp"]["sec"] = 2
                row["sim_time"] = 2.0
            if kind == "side":
                row["side"] = "right"
            self.assertEqual(contact_state(row, 1.0, "left")["state"], "invalid", kind)
