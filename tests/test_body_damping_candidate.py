"""Regression checks for test body damping candidate. See the folder README for its role."""

import importlib.util
from pathlib import Path
import unittest
from my_robot_description.sensing.balance import BalanceConfig
from my_robot_description.control.balance_control import active_correction


class BodyDampingTests(unittest.TestCase):
    def correction(self, kd, rate=0.1, age=0.01):
        return active_correction(
            0.02,
            rate,
            age,
            [dict(ground=True, age=0.01)] * 2,
            2.0,
            BalanceConfig(kd=kd, ankle_limit=1.5),
        )

    def test_candidate_changes_rate_term_without_raising_proposal_cap(self):
        self.assertAlmostEqual(self.correction(4.0)["balance_nm"][2], 0.4)
        self.assertAlmostEqual(self.correction(8.0)["balance_nm"][2], 0.6)
        self.assertEqual(self.correction(8.0, rate=1.0)["balance_nm"][2], 1.5)

    def test_opposite_rate_brakes_rebound_and_stale_imu_stays_gated(self):
        self.assertLess(
            self.correction(8.0, rate=-0.1)["balance_nm"][2],
            self.correction(4.0, rate=-0.1)["balance_nm"][2],
        )
        self.assertEqual(self.correction(8.0, age=0.1)["balance_nm"], [0.0] * 6)

    def test_launch_defaults_preserve_previous_gain(self):
        spec = importlib.util.spec_from_file_location(
            "body_damping_runner", Path("scripts/experiments/run_effort_push.py")
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        args = (
            Path("/tmp/damping-test"),
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
            1.5,
            True,
        )
        self.assertIn("balance_kd:=4.0", module.launch_command(*args))
        self.assertIn("balance_kd:=8.0", module.launch_command(*args, balance_kd=8.0))
