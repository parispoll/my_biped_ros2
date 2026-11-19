"""Regression checks for test force limit study. See the folder README for its role."""

import subprocess
import tempfile
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from scripts.studies.run_force_limit_study import classify


class ForceStudyTests(unittest.TestCase):
    def test_incomplete_cannot_be_classified_as_recovery_failure(self):
        self.assertEqual(
            classify({"completed": False, "passed": False}, 1),
            "incomplete_requires_diagnosis",
        )
        self.assertEqual(
            classify({"completed": True, "passed": True}, 1),
            "incomplete_requires_diagnosis",
        )

    def test_completed_outcomes(self):
        self.assertEqual(classify({"completed": True, "passed": True}, 0), "recovered")
        self.assertEqual(
            classify({"completed": True, "passed": False}, 0),
            "recovery_criteria_not_met",
        )

    def test_cli_rejects_duplicate_or_descending_escalation_before_creating_output(
        self,
    ):
        for forces in ((20, 15), (15, 15)):
            with tempfile.TemporaryDirectory() as d:
                output = Path(d) / "study"
                result = subprocess.run(
                    [
                        sys.executable,
                        "scripts/studies/run_force_limit_study.py",
                        "--output",
                        str(output),
                        "--forces",
                        *map(str, forces),
                    ],
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(result.returncode, 2)
                self.assertIn("strictly increasing", result.stderr)
                self.assertFalse(output.exists())
