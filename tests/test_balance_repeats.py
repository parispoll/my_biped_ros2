"""Regression checks for test balance repeats. See the folder README for its role."""

import copy
import unittest
from my_robot_description.studies.balance_repeats import summarize_repeats


def fixture():
    repeats = []
    for n, (off, on) in enumerate(((1.0, 0.8), (1.2, 0.9)), 1):
        comparisons = {}
        for direction, force in (("forward", 2.0), ("backward", -2.0)):
            rows = [
                dict(
                    mode=mode,
                    peak_pitch_deg=peak,
                    peak_pitch_rate_rad_s=0.03,
                    last_outside_band_after_clear_s=None if mode == "on" else 1.0,
                    final_joint_offset_rad=0.0002,
                    recovery_passed=True,
                    contacts_passed=True,
                    saturated_samples=0,
                    audit={"passed": True},
                )
                for mode, peak in (("off", off), ("on", on))
            ]
            comparisons[direction] = dict(
                completed=True,
                requested=dict(force_x=force, duration=0.2, recovery=10.0),
                rows=rows,
                published_window_relative_difference=0.0,
            )
        repeats.append(dict(repeat=n, comparisons=comparisons))
    return repeats


class RepeatTests(unittest.TestCase):
    def test_means_ranges_and_paired_percentages(self):
        r = summarize_repeats(fixture())
        g = r["groups"]["forward"]
        self.assertTrue(r["peak_reduction_consistent"])
        self.assertAlmostEqual(g["metrics"]["off"]["peak_pitch_deg"]["mean"], 1.1)
        self.assertAlmostEqual(g["metrics"]["on"]["peak_pitch_deg"]["mean"], 0.85)
        self.assertAlmostEqual(g["peak_reduction_percent"]["mean"], 22.5)
        self.assertEqual(g["metrics"]["on"]["last_outside_band_s"]["mean"], 0.0)

    def test_failures_are_reported_not_discarded(self):
        r = fixture()
        r[1]["comparisons"]["backward"]["rows"][1]["recovery_passed"] = False
        s = summarize_repeats(r)
        self.assertFalse(s["all_comparison_checks_passed"])
        self.assertFalse(s["peak_reduction_consistent"])
        self.assertEqual(s["groups"]["backward"]["n_pairs"], 2)

    def test_worse_peak_prevents_consistency_claim(self):
        r = fixture()
        r[1]["comparisons"]["forward"]["rows"][1]["peak_pitch_deg"] = 1.3
        self.assertFalse(summarize_repeats(r)["peak_reduction_consistent"])

    def test_incomplete_duplicate_or_changed_protocol_is_rejected(self):
        base = fixture()
        for mutation in ("duplicate", "protocol", "incomplete", "nonfinite"):
            r = copy.deepcopy(base)
            if mutation == "duplicate":
                r[1]["repeat"] = 1
            if mutation == "protocol":
                r[1]["comparisons"]["forward"]["requested"]["duration"] = 0.3
            if mutation == "incomplete":
                r[1]["comparisons"]["forward"]["completed"] = False
            if mutation == "nonfinite":
                r[1]["comparisons"]["forward"]["rows"][0]["peak_pitch_deg"] = float(
                    "nan"
                )
            with self.assertRaises(ValueError):
                summarize_repeats(r)
