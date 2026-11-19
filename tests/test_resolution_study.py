"""Keep decreasing offsets, repeatability, input quality and recovery distinct."""

import unittest
from my_robot_description.studies.resolution_study import summarize_resolution_study


class ResolutionStudyTests(unittest.TestCase):
    def fixture(self):
        rows = []
        for step, pitch, joint in (
            (0.001, 0.48, 0.0084),
            (0.0005, 0.12, 0.0021),
            (0.00025, 0.115, 0.0020),
        ):
            for repeat, offset in ((1, -1), (2, 1)):
                rows.append(
                    dict(
                        physics_step_s=step,
                        repeat=repeat,
                        final_pitch_change_deg=pitch + offset * 0.0005,
                        final_joint_change_rad=joint + offset * 0.000005,
                        physics_input_impulse_n_s=0.2,
                        ankle_implicit_damping_nm=-500 * step,
                        physics_audit_passed=True,
                        recovery_passed=False,
                    )
                )
        return rows

    def test_stable_resolution_can_still_fail_recovery(self):
        result = summarize_resolution_study(self.fixture())
        self.assertTrue(result["resolution_screen_passed"])
        self.assertEqual(result["groups"][-1]["recoveries_passed"], 0)

    def test_decreasing_but_large_refinement_change_is_not_converged(self):
        rows = self.fixture()
        for r in rows:
            if r["physics_step_s"] == 0.00025:
                r["final_pitch_change_deg"] -= 0.05
        result = summarize_resolution_study(rows)
        self.assertFalse(result["finest_resolution_change_within_tolerance"])
        self.assertFalse(result["resolution_screen_passed"])

    def test_repeat_variation_prevents_a_positive_screen(self):
        rows = self.fixture()
        rows[-1]["final_pitch_change_deg"] += 0.02
        rows[-2]["final_pitch_change_deg"] -= 0.02
        result = summarize_resolution_study(rows)
        self.assertTrue(result["finest_resolution_change_within_tolerance"])
        self.assertFalse(result["fine_repeat_variation_within_tolerance"])
        self.assertFalse(result["resolution_screen_passed"])

    def test_unequal_inputs_prevent_a_positive_screen(self):
        rows = self.fixture()
        rows[-1]["physics_input_impulse_n_s"] = 0.15
        result = summarize_resolution_study(rows)
        self.assertFalse(result["input_impulses_within_five_percent"])
        self.assertFalse(result["resolution_screen_passed"])

    def test_failed_force_audit_prevents_a_positive_screen(self):
        rows = self.fixture()
        rows[-1]["physics_audit_passed"] = False
        self.assertFalse(summarize_resolution_study(rows)["resolution_screen_passed"])

    def test_missing_repeat_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "two independent"):
            summarize_resolution_study(self.fixture()[:-1])

    def test_duplicate_identity_is_rejected(self):
        rows = self.fixture()
        rows.append(rows[-1].copy())
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            summarize_resolution_study(rows)

    def test_nonfinite_measurement_is_rejected(self):
        rows = self.fixture()
        rows[-1]["final_pitch_change_deg"] = float("nan")
        with self.assertRaisesRegex(ValueError, "Non-finite"):
            summarize_resolution_study(rows)

    def test_zero_push_input_is_rejected(self):
        rows = self.fixture()
        rows[-1]["physics_input_impulse_n_s"] = 0
        with self.assertRaisesRegex(ValueError, "positive"):
            summarize_resolution_study(rows)
