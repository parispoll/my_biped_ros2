"""Regression checks for test body damping metrics. See the folder README for its role."""

from pathlib import Path
import sys
import unittest
import math

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from scripts.analysis.body_damping_metrics import response_metrics


class BodyResponseMetricsTests(unittest.TestCase):
    def test_rebound_is_direction_aware_and_subtracts_baseline(self):
        summary = dict(
            completed=True,
            baseline_pitch_rad=0.2,
            passed=True,
            last_outside_band_after_clear_s=2.0,
            events=[
                dict(event="push_published", sim_time=1.0),
                dict(event="clear_published", sim_time=1.2),
            ],
        )
        for sign in (-1, 1):
            rows = [
                dict(sim_time=t, pitch=0.2 + sign * d, pitch_rate=0.1)
                for t, d in ((1.1, 0.1), (1.5, -0.04), (3.0, 0.0))
            ]
            result = response_metrics(rows, summary, sign * 25)
            self.assertAlmostEqual(result["primary_pitch_deg"], math.degrees(0.1))
            self.assertAlmostEqual(result["opposite_rebound_deg"], math.degrees(0.04))

    def test_incomplete_run_cannot_provide_full_rebound_metrics(self):
        with self.assertRaises(ValueError):
            response_metrics([], {"completed": False}, 25)
