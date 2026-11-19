"""Replayed solver residue is unloaded contact, never missing data or air."""

import copy, json, unittest
from pathlib import Path
from test_foot_loads import sample
from my_robot_description.sensing.foot_loads import decode_foot_load
from my_robot_description.sensing.contact_state import contact_state


class RoundoffTests(unittest.TestCase):
    def test_actual_rejected_wrench_and_strict_bilateral_mode(self):
        path = Path(__file__).resolve().parent / "fixtures/unloaded_solver_residue.json"
        row = json.loads(path.read_text())
        with self.assertRaises(ValueError):
            decode_foot_load(row)
        load = decode_foot_load(row, allow_zero=True)
        self.assertEqual(load["force_world_n"], [0, 0, 0])
        self.assertIsNone(load["cop_world_xy_m"])
        self.assertAlmostEqual(load["raw_force_world_n"][0], 4.110239915066511e-8)
        state = contact_state(row, row["sim_time"] + 0.002, "right")
        self.assertEqual(state["state"], "touching_unloaded")
        self.assertTrue(state["numerical_zero_normalized"])
        from my_robot_description.motion.zero_load import zero_load_guard

        now = row["sim_time"] + 0.002
        support = contact_state(
            sample("left", row["sim_time"], 22.28 * 9.81), now, "left"
        )
        self.assertTrue(
            zero_load_guard({"left": support, "right": state}, now, 22.28, 0.0, "left")[
                "eligible"
            ]
        )
        self.assertEqual(
            contact_state(row, row["sim_time"] + 0.06, "right")["state"], "stale"
        )

    def test_larger_and_cancelling_forces_are_rejected(self):
        for magnitude in [1.01e-6, 1e-3, 1.0]:
            row = sample("left", 1.0, 0.0)
            for i, w in enumerate(row["message"]["contacts"][0]["wrenches"]):
                value = magnitude if i % 2 else -magnitude
                w["body_1_wrench"]["force"]["x"] = value
                w["body_2_wrench"]["force"]["x"] = -value
            with self.assertRaises(ValueError):
                decode_foot_load(row, allow_zero=True)

    def test_threshold_and_negative_normal_are_separate(self):
        row = sample("left", 1.0, 0.0)
        w = row["message"]["contacts"][0]["wrenches"][0]
        w["body_1_wrench"]["force"]["x"] = 1e-6
        w["body_2_wrench"]["force"]["x"] = -1e-6
        self.assertTrue(
            decode_foot_load(row, allow_zero=True)["numerical_zero_normalized"]
        )
        row = sample("left", 1.0, -1e-5)
        with self.assertRaises(ValueError):
            decode_foot_load(row, allow_zero=True)
