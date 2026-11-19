"""Duration comparisons must keep infrastructure errors separate from recovery."""

import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location(
    "duration_comparison",
    Path(__file__).resolve().parents[1] / "scripts/studies/run_duration_comparison.py",
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class DurationComparisonTests(unittest.TestCase):
    def result(self, passed=True):
        return dict(
            completed=True,
            passed=passed,
            push=dict(
                requested={"force_x": -2.0},
                command_window_s=0.201,
                peak_pitch_change_rad=0.01,
                peak_pitch_rate_rad_s=0.1,
                final_second_within_return_band=passed,
                continuous_contact_evidence=True,
                saturated_controller_samples=0,
                last_outside_band_after_clear_s=0.8,
            ),
        )

    def test_incomplete_run_cannot_be_reported_as_physical_failure(self):
        with self.assertRaises(ValueError):
            module.compare([(0.2, {"completed": False, "error": "clock"})])

    def test_impulse_increase_and_units_are_explicit(self):
        rows = module.compare([(0.2, self.result()), (0.3, self.result())])
        self.assertAlmostEqual(
            rows[1]["nominal_requested_impulse_n_s"]
            / rows[0]["nominal_requested_impulse_n_s"],
            1.5,
        )
        self.assertAlmostEqual(rows[0]["peak_pitch_deg"], 0.5729577951308232)
        self.assertEqual(rows[0]["force_x_n"], -2.0)
        self.assertAlmostEqual(rows[0]["last_outside_band_after_push_s"], 1.001)

    def test_completed_failed_recovery_is_retained_in_results(self):
        row = module.compare([(0.2, self.result(False))])[0]
        self.assertFalse(row["recovery_passed"])
        self.assertFalse(row["returned_within_bands"])
