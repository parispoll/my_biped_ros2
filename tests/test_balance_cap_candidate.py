"""Regression checks for test balance cap candidate. See the folder README for its role."""

import importlib.util
from pathlib import Path
import unittest
from my_robot_description.sensing.balance import BalanceConfig
from my_robot_description.control.balance_control import active_correction
from my_robot_description.model.torque import clamp_torque


class CandidateCapTests(unittest.TestCase):
    def correction(self, cap):
        return active_correction(
            0.02,
            0.4,
            0.01,
            [dict(ground=True, age=0.01)] * 2,
            2.0,
            BalanceConfig(ankle_limit=cap),
        )

    def test_candidate_only_changes_clipped_balance_proposal(self):
        old = self.correction(0.5)["balance_nm"]
        new = self.correction(1.5)["balance_nm"]
        self.assertEqual(old, [0, 0, 0.5, 0, 0, 0.5])
        self.assertEqual(new, [0, 0, 1.0, 0, 0, 1.0])
        self.assertEqual(clamp_torque(5.8 + new[2], 6.0), 6.0)

    def test_stale_contact_still_disables_larger_correction(self):
        result = active_correction(
            0.1,
            0.7,
            0.01,
            [dict(ground=True, age=0.1)] * 2,
            2.0,
            BalanceConfig(ankle_limit=1.5),
        )
        self.assertFalse(result["balance_gate"])
        self.assertEqual(result["balance_nm"], [0.0] * 6)

    def test_runner_defaults_to_original_cap_and_forwards_explicit_candidate(self):
        spec = importlib.util.spec_from_file_location(
            "cap_runner", Path("scripts/experiments/run_effort_push.py")
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        args = (
            Path("/tmp/cap-test"),
            False,
            0.2,
            False,
            False,
            0.25,
            0.001,
            "explicit",
            "pid",
            True,
            True,
        )
        self.assertIn("ankle_balance_limit:=0.5", module.launch_command(*args))
        self.assertIn(
            "ankle_balance_limit:=1.5",
            module.launch_command(*args, ankle_balance_limit=1.5),
        )
