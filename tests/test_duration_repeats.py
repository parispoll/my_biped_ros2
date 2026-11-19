"""Count real distinct repeats and reject invalid/incompatible raw evidence."""

import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from scripts.analysis.duration_repeat_analysis import describe, verify


class RepeatStatisticsTests(unittest.TestCase):
    def entries(self):
        return [
            dict(
                direction=direction,
                repeat=repeat,
                path=f"/{direction}-{repeat}",
                responses=[
                    dict(
                        duration_s=duration,
                        force_x_n=(2 if direction == "forward" else -2),
                        peak_pitch_deg=peak + repeat * 0.001,
                        recovery_passed=True,
                    )
                    for duration, peak in ((0.2, 0.4), (0.3, 0.6))
                ],
            )
            for direction in ("forward", "backward")
            for repeat in (1, 2)
        ]

    def test_reports_observed_values_and_paired_effects(self):
        result = describe(self.entries())
        self.assertEqual(result["forward"]["repeats"], 2)
        self.assertAlmostEqual(
            result["forward"]["peak_pitch_deg"]["0.2"]["mean"], 0.4015
        )
        self.assertTrue(result["backward"]["longer_pulse_increased_peak_in_both"])
        self.assertAlmostEqual(
            result["forward"]["paired_peak_increase_percent"][0],
            (0.601 / 0.401 - 1) * 100,
        )

    def test_same_evidence_cannot_be_counted_twice(self):
        values = self.entries()
        values[1]["path"] = values[0]["path"]
        with self.assertRaises(ValueError):
            describe(values)

    def test_missing_repeat_or_physical_failure_cannot_be_success(self):
        with self.assertRaises(ValueError):
            describe(self.entries()[:-1])
        values = self.entries()
        values[0]["responses"][0]["recovery_passed"] = False
        with self.assertRaises(ValueError):
            describe(values)

    def test_wrong_force_and_nonfinite_peak_are_rejected(self):
        for update in ({"force_x_n": -2}, {"peak_pitch_deg": float("nan")}):
            values = self.entries()
            values[0]["responses"][0].update(update)
            with self.assertRaises(ValueError):
                describe(values)


class RawVerificationTests(unittest.TestCase):
    def study(self, root):
        cases = []
        for duration in (0.2, 0.3):
            folder = root / str(duration)
            folder.mkdir()
            trial = dict(
                requested={"force_x": 2.0, "duration": duration},
                damping_model="explicit",
                controller_mode="pid",
                balance_enabled=True,
                physics_step_requested_s=0.001,
                real_time_factor_requested=0.25,
                control_gap_frozen_samples=0,
                balance_observer={"passed": True},
                viscous_damping_audit={"passed": True},
                force_handling={
                    key: True
                    for key in (
                        "queued_in_gazebo",
                        "cleared_in_gazebo",
                        "clear_observed_while_recording",
                    )
                },
                command_window_s=duration,
                peak_pitch_change_rad=0.01,
                peak_pitch_rate_rad_s=0.1,
                final_second_within_return_band=True,
                continuous_contact_evidence=True,
                saturated_controller_samples=0,
                last_outside_band_after_clear_s=1.0,
            )
            reference = copy.deepcopy(trial)
            reference["requested"]["force_x"] = 0.0
            (folder / "summary.json").write_text(
                json.dumps(
                    dict(completed=True, passed=True, reference=reference, push=trial)
                )
            )
            cases.append(
                dict(duration_s=duration, path=str(folder), runner_returncode=0)
            )
        (root / "comparison.json").write_text(
            json.dumps(
                dict(completed=True, source_hashes={"example": "hash"}, cases=cases)
            )
        )

    def test_recomputes_metrics_from_raw_and_checks_direction_and_sources(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.study(root)
            result = verify(root, {"example": "hash"}, "forward")
            self.assertEqual(len(result["responses"]), 2)
            with self.assertRaises(ValueError):
                verify(root, {"example": "changed"}, "forward")
            with self.assertRaises(ValueError):
                verify(root, {"example": "hash"}, "backward")

    def test_invalid_audit_is_rejected_even_if_summary_says_passed(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.study(root)
            path = root / "0.2/summary.json"
            raw = json.loads(path.read_text())
            raw["push"]["control_gap_frozen_samples"] = 1
            path.write_text(json.dumps(raw))
            with self.assertRaises(ValueError):
                verify(root, {"example": "hash"}, "forward")
