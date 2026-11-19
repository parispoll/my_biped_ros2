"""Regression checks for test force repeat check. See the folder README for its role."""

import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from scripts.studies.run_force_repeat_check import assess


class RepeatOutcomeTests(unittest.TestCase):
    def test_guard_requires_passed_reference_and_published_push(self):
        log = "RuntimeError: Pelvis tilt exceeded 0.15 rad"
        events = [dict(event="push_published")]
        self.assertEqual(
            assess({"reference": {"passed": True}}, 1, log, events),
            "posture_guard_stopped_partial_trial",
        )
        for report, seen in (
            ({}, events),
            ({"reference": {"passed": False}}, events),
            ({"reference": {"passed": True}}, []),
        ):
            self.assertEqual(
                assess(report, 1, log, seen), "incomplete_requires_diagnosis"
            )

    def test_infrastructure_failure_is_not_classified_as_posture_guard(self):
        self.assertEqual(
            assess(
                {"reference": {"passed": True}},
                1,
                "Feedback became stale",
                [dict(event="push_published")],
            ),
            "incomplete_requires_diagnosis",
        )

    def test_completed_nonpass_is_not_overridden_by_log_text(self):
        self.assertEqual(
            assess(
                {"completed": True, "passed": False},
                0,
                "Pelvis tilt exceeded 0.15 rad",
                [],
            ),
            "recovery_criteria_not_met",
        )
