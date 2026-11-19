"""Regression checks for test com geometry. See the folder README for its role."""

import math
import unittest
from my_robot_description.sensing.com_geometry import (
    decode_com_state,
    convex_hull,
    geometric_support,
    contact_at,
)


class ComGeometryTests(unittest.TestCase):
    def state(self):
        values = [1.0, 14.1, 0.0, 0.0, 0.42]
        for y in (0.085, -0.085):
            for x, dy in ((-0.1, -0.06), (0.1, -0.06), (0.1, 0.06), (-0.1, 0.06)):
                values.extend((x, y + dy, 0.0))
        return decode_com_state(values)

    def test_schema_and_all_foot_corners(self):
        state = self.state()
        self.assertEqual(state["mass_kg"], 14.1)
        self.assertEqual(len(state["feet"]["left"]), 4)
        for values in ([0.0] * 28, [0.0] * 29, [float("nan")] * 29):
            with self.assertRaises(ValueError):
                decode_com_state(values)

    def test_two_flat_grounded_feet_define_expected_hull(self):
        contacts = {side: dict(ground=True, age=0.01) for side in ("left", "right")}
        result = geometric_support(self.state(), contacts)
        self.assertEqual(len(result["hull"]), 4)
        self.assertEqual(result["qualified_feet"], ["left", "right"])
        self.assertAlmostEqual(min(p[1] for p in result["hull"]), -0.145)

    def test_stale_unknown_tilted_and_airborne_feet_do_not_claim_full_support(self):
        for ground, age in (
            (False, 0.01),
            (True, 0.1),
            (True, -0.1),
            (True, float("nan")),
        ):
            self.assertFalse(
                geometric_support(self.state(), {"left": dict(ground=ground, age=age)})[
                    "hull"
                ]
            )
        state = self.state()
        state["feet"]["left"][0][2] = 0.02
        self.assertFalse(
            geometric_support(state, {"left": dict(ground=True, age=0.0)})["hull"]
        )
        self.assertFalse(geometric_support(self.state(), {})["hull"])

    def test_one_qualified_foot_never_includes_other_foot(self):
        result = geometric_support(self.state(), {"left": dict(ground=True, age=0.0)})
        self.assertEqual(result["qualified_feet"], ["left"])
        self.assertGreater(min(p[1] for p in result["hull"]), 0)

    def test_degenerate_points_do_not_form_a_polygon(self):
        self.assertEqual(convex_hull([(0, 0), (1, 0), (2, 0)]), [])

    def test_contact_selection_does_not_use_future_evidence(self):
        history = [dict(time=0.98, ground=True), dict(time=1.02, ground=False)]
        self.assertEqual(contact_at(1.0, history), dict(ground=True, age=1.0 - 0.98))
        self.assertEqual(contact_at(0.9, history), dict(ground=False, age=None))
