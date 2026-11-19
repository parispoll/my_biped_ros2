"""Regression checks for test unloaded ankle. See the folder README for its role."""

import importlib.util, unittest
from pathlib import Path
from my_robot_description.control.roll_posture import RollPosture, KD
from my_robot_description.model.roll_stance import JOINT_NAMES, MOTOR_CAPS

ROOT = Path(__file__).resolve().parents[1]


class UnloadedAnkleTests(unittest.TestCase):
    def test_invalid_derivative_inputs_reject(self):
        p = dict(
            joint_names=JOINT_NAMES,
            motor_caps_nm=MOTOR_CAPS,
            target_rad=[0.0] * 10,
            feedforward_nm=[0.0] * 10,
        )
        c = RollPosture(p)
        for d in [[0.3] * 9, [-1.0] * 10, [float("nan")] * 10]:
            with self.assertRaises(ValueError):
                c.update([0.0] * 10, [0.0] * 10, derivative_gains=d)
        self.assertEqual(c.update([0.0] * 10, [1.0] * 10)["derivative_gains"], KD)

    def test_scheduled_derivative_is_used_without_changing_global_gains(self):
        p = dict(
            joint_names=JOINT_NAMES,
            motor_caps_nm=MOTOR_CAPS,
            target_rad=[0.0] * 10,
            feedforward_nm=[0.0] * 10,
        )
        c = RollPosture(p)
        original = list(KD)
        d = list(KD)
        d[9] = 0.3
        r = c.update([0.0] * 10, [1.0] * 10, derivative_gains=d)
        self.assertEqual(r["damping"][9], -0.3)
        self.assertEqual(KD, original)

    def test_invalid_proportional_inputs_reject(self):
        p = dict(
            joint_names=JOINT_NAMES,
            motor_caps_nm=MOTOR_CAPS,
            target_rad=[0.0] * 10,
            feedforward_nm=[0.0] * 10,
        )
        c = RollPosture(p)
        for gains in [[120.0] * 9, [-1.0] * 10, [float("nan")] * 10]:
            with self.assertRaises(ValueError):
                c.update([0.0] * 10, [0.0] * 10, proportional_gains=gains)

    def test_scheduled_stiffness_is_used_and_defaults_preserved(self):
        from my_robot_description.control.roll_posture import KP

        p = dict(
            joint_names=JOINT_NAMES,
            motor_caps_nm=MOTOR_CAPS,
            target_rad=[0.0] * 10,
            feedforward_nm=[0.0] * 10,
        )
        c = RollPosture(p)
        original = list(KP)
        gains = list(KP)
        gains[4] = 120.0
        r = c.update([-0.01] * 10, [0.0] * 10, proportional_gains=gains)
        self.assertAlmostEqual(r["proportional"][4], 1.2)
        self.assertEqual(KP, original)
