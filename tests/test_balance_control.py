"""Active-feedback gating, ramping, total-command accounting and anti-windup."""

import unittest
import importlib.util
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from my_robot_description.sensing.balance import BalanceConfig, balance_proposal
from my_robot_description.control.balance_control import active_correction
from my_robot_description.control.integral import BoundedIntegral
from my_robot_description.validation.balance_audit import audit_balance


class ActiveBalanceTests(unittest.TestCase):
    def test_contact_sampler_forwards_original_message_without_restamping(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "src/my_robot_description/nodes/sensing/sample_contacts.py"
        )
        spec = importlib.util.spec_from_file_location("contact_sampler", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        msg = SimpleNamespace(header=SimpleNamespace(stamp=123.456))
        pub = Mock()
        node = SimpleNamespace(latest={"left": msg}, publishers_owned={"left": pub})
        module.ContactSampler.forward(node)
        module.ContactSampler.forward(node)
        self.assertIs(pub.publish.call_args.args[0], msg)
        self.assertEqual(msg.header.stamp, 123.456)

    def correction(self, **updates):
        values = dict(
            pitch=0.02,
            rate=0.1,
            imu_age=0.01,
            contacts=[dict(ground=True, age=0.002)] * 2,
            elapsed=2.0,
            config=BalanceConfig(),
        )
        values.update(updates)
        return active_correction(**values)

    def test_full_correction_only_changes_ankles(self):
        r = self.correction()
        self.assertTrue(r["balance_gate"])
        self.assertEqual(r["balance_nm"], [0.0, 0.0, 0.4, 0.0, 0.0, 0.4])

    def test_ramp_uses_elapsed_simulation_time(self):
        self.assertEqual(self.correction(elapsed=0.0)["balance_nm"], [0.0] * 6)
        self.assertAlmostEqual(self.correction(elapsed=0.25)["balance_nm"][2], 0.1)

    def test_missing_stale_or_future_measurements_disable_the_correction(self):
        for update in (
            dict(elapsed=None),
            dict(imu_age=None),
            dict(imu_age=0.051),
            dict(imu_age=-0.02),
            dict(contacts=[]),
            dict(contacts=[dict(ground=False, age=0.0)] * 2),
            dict(contacts=[dict(ground=True, age=0.051)] * 2),
        ):
            r = self.correction(**update)
            self.assertFalse(r["balance_gate"])
            self.assertEqual(r["balance_nm"], [0.0] * 6)

    def test_combined_command_saturation_freezes_joint_integral(self):
        integral = BoundedIntegral([1.0] * 6, [1.0] * 6)
        # PD alone is under the cap; PD plus balance is already over it.
        pd = [5.8] * 6
        balance = [0.4] * 6
        integral.update(
            [1.0] * 6, 0.01, [a + b for a, b in zip(pd, balance)], [6.0] * 6
        )
        self.assertEqual(integral.state, [0.0] * 6)
        integral.update(
            [-1.0] * 6, 0.01, [a + b for a, b in zip(pd, balance)], [6.0] * 6
        )
        self.assertEqual(integral.state, [-0.01] * 6)

    def test_active_audit_requires_sensor_gate_and_exact_extra_torque(self):
        settings = dict(kp=20.0, kd=4.0, ankle_limit=0.5, target_pitch=0.0)
        obs = [
            dict(
                sim_time=i * 0.01,
                pitch_rad=0.02,
                pitch_rate_rad_s=0.1,
                settings=settings,
                applied=False,
                applied_balance_nm=0.0,
                **balance_proposal(0.02, 0.1)
            )
            for i in range(101)
        ]
        extra = self.correction()["balance_nm"]
        c = dict(
            feedforward=[0.0] * 6,
            proportional=[0.0] * 6,
            damping=[0.0] * 6,
            integral=[0.0] * 6,
            command=extra.copy(),
            requested=extra.copy(),
            balance_enabled=True,
            balance_nm=extra.copy(),
            balance_pitch_rad=0.02,
            balance_rate_rad_s=0.1,
            balance_imu_age=0.01,
            balance_contacts=[dict(ground=True, age=0.002)] * 2,
            balance_elapsed=2.0,
            balance_settings=settings,
            balance_ramp_seconds=1.0,
        )
        self.assertTrue(audit_balance(obs, [c], 0.0, 1.0, active=True)["passed"])
        c["balance_nm"][2] += 0.1
        self.assertFalse(audit_balance(obs, [c], 0.0, 1.0, active=True)["passed"])
        c["balance_nm"] = extra.copy()
        c["balance_imu_age"] = 0.1
        self.assertFalse(audit_balance(obs, [c], 0.0, 1.0, active=True)["passed"])
