"""Balance signs, gyro frame conversion, proposal clipping and non-application."""

import ast
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import xacro
from my_robot_description.sensing.balance import (
    BalanceConfig,
    balance_proposal,
    sagittal_imu,
)
from my_robot_description.validation.balance_audit import audit_balance


class BalanceTests(unittest.TestCase):
    def test_current_robot_axes_support_the_pitch_sign_derivation(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "src/my_robot_description/urdf/effort_standing.urdf.xacro"
        )
        robot = ET.fromstring(xacro.process_file(str(path)).toxml())
        for side in ("left", "right"):
            for name in ("hip", "knee", "ankle"):
                joint = robot.find(f"joint[@name='{side}_{name}']")
                self.assertEqual(joint.find("axis").get("xyz"), "0 1 0")
                self.assertEqual(joint.find("origin").get("rpy"), "0 0 0")

    def test_forward_lean_requires_positive_ankle_motor_proposal(self):
        r = balance_proposal(0.01, 0.0)
        self.assertAlmostEqual(r["proposal_per_ankle_nm"], 0.1)
        self.assertEqual(r["joint_proposal_nm"], [0.0, 0.0, 0.1, 0.0, 0.0, 0.1])
        self.assertAlmostEqual(
            balance_proposal(-0.01, 0.0)["proposal_per_ankle_nm"], -0.1
        )

    def test_rate_term_opposes_body_motion_and_splits_total_equally(self):
        self.assertAlmostEqual(balance_proposal(0.0, 0.1)["proposal_per_ankle_nm"], 0.2)
        self.assertAlmostEqual(
            balance_proposal(0.0, -0.1)["proposal_per_ankle_nm"], -0.2
        )
        self.assertAlmostEqual(
            balance_proposal(0.02, 0.0, BalanceConfig(target_pitch=0.02))[
                "proposal_per_ankle_nm"
            ],
            0.0,
        )

    def test_proposals_are_clipped_in_both_directions(self):
        for sign in (-1, 1):
            r = balance_proposal(sign * 0.1, sign * 1.0)
            self.assertEqual(r["proposal_per_ankle_nm"], sign * 0.5)
            self.assertTrue(r["proposal_clipped"])

    def test_pitch_and_pitch_rate_use_orientation_and_body_gyro(self):
        p, v = sagittal_imu(0, math.sin(0.05), 0, math.cos(0.05), 0, 0.2, 0)
        self.assertAlmostEqual(p, 0.1)
        self.assertAlmostEqual(v, 0.2)
        p, v = sagittal_imu(
            math.sin(math.pi / 12), 0, 0, math.cos(math.pi / 12), 0, 2, 1
        )
        self.assertAlmostEqual(p, 0.0)
        self.assertAlmostEqual(v, math.sqrt(3) - 0.5)

    def test_bad_sensor_data_and_settings_are_rejected(self):
        for config in (
            BalanceConfig(kp=-1),
            BalanceConfig(ankle_limit=0),
            BalanceConfig(kd=float("nan")),
        ):
            with self.assertRaises(ValueError):
                balance_proposal(0.0, 0.0, config)
        with self.assertRaises(ValueError):
            balance_proposal(float("nan"), 0.0)
        with self.assertRaises(ValueError):
            sagittal_imu(0, 0, 0, 0, 0, 0, 0)
        with self.assertRaises(ValueError):
            sagittal_imu(0, 0, 0, 1, 0, float("nan"), 0)

    def test_observer_has_no_publishers_or_actuator_clients(self):
        source = (
            Path(__file__).resolve().parents[1]
            / "src/my_robot_description/nodes/legacy/observe_balance.py"
        )
        tree = ast.parse(source.read_text())
        calls = [
            n.func.attr
            for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
        ]
        self.assertNotIn("create_publisher", calls)
        self.assertNotIn("create_client", calls)

    def fixture(self):
        rows = [
            dict(
                sim_time=i * 0.01,
                pitch_rad=0.01,
                pitch_rate_rad_s=0.0,
                settings=dict(kp=20.0, kd=4.0, ankle_limit=0.5, target_pitch=0.0),
                applied=False,
                applied_balance_nm=0.0,
                **balance_proposal(0.01, 0.0),
            )
            for i in range(101)
        ]
        c = [
            dict(
                feedforward=[1.0] * 6,
                proportional=[0.1] * 6,
                damping=[-0.1] * 6,
                integral=[0.2] * 6,
                requested=[1.2] * 6,
                command=[1.2] * 6,
            )
        ]
        return rows, c

    def test_audit_checks_coverage_and_motor_composition(self):
        r, c = self.fixture()
        self.assertTrue(audit_balance(r, c, 0.0, 1.0)["passed"])
        c[0]["requested"][2] += 0.1
        self.assertFalse(audit_balance(r, c, 0.0, 1.0)["passed"])
        r, c = self.fixture()
        c[0]["command"][2] += 0.1
        self.assertFalse(audit_balance(r, c, 0.0, 1.0)["passed"])
        r, c = self.fixture()
        self.assertFalse(audit_balance(r[:50], c, 0.0, 1.0)["passed"])
        r[0]["applied"] = True
        with self.assertRaises(ValueError):
            audit_balance(r, c, 0.0, 1.0)
        r, c = self.fixture()
        r[10]["proportional_total_nm"] += 0.1
        self.assertFalse(audit_balance(r, c, 0.0, 1.0)["passed"])
        r, c = self.fixture()
        r[10]["joint_proposal_nm"][0] = 0.1
        self.assertFalse(audit_balance(r, c, 0.0, 1.0)["passed"])
