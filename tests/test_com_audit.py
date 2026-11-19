"""Regression checks for test com audit. See the folder README for its role."""

import unittest
from my_robot_description.validation.com_audit import audit_com


class ComAuditTests(unittest.TestCase):
    def rows(self):
        return [
            dict(sim_time=i * 0.05, mass_kg=14.1, com=[0.0, 0.0, 0.4])
            for i in range(21)
        ]

    def test_complete_stream_passes(self):
        self.assertTrue(audit_com(self.rows(), 0, 1)["passed"])

    def test_missing_interval_does_not_certify_coverage(self):
        self.assertFalse(
            audit_com([r for r in self.rows() if not 0.3 < r["sim_time"] < 0.6], 0, 1)[
                "passed"
            ]
        )
        self.assertFalse(audit_com(self.rows()[4:], 0, 1)["passed"])

    def test_invalid_mass_or_coordinates_do_not_pass(self):
        for key, value in (
            ("mass_kg", 14.2),
            ("mass_kg", float("nan")),
            ("com", [float("nan"), 0, 0]),
        ):
            rows = self.rows()
            rows[10][key] = value
            self.assertFalse(audit_com(rows, 0, 1)["passed"])
