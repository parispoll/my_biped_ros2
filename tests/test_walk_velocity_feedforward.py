"""Reference motion supplies bounded-motor damping torque, not gain changes."""

import unittest
from types import SimpleNamespace
from my_robot_description.motion.profiled_walk import ProfiledWalkProgram


class VelocityFeedforwardTests(unittest.TestCase):
    def program(self, enabled):
        p = ProfiledWalkProgram.__new__(ProfiledWalkProgram)
        p.profile = SimpleNamespace(velocity_feedforward=enabled)
        p.base = SimpleNamespace(
            sample=lambda t: dict(
                target_rad=[t * t] * 10, feedforward_nm=[3.0] * 10, step_index=1
            )
        )
        p.steps = [dict(lower_start=0.0, recontact_start=5.0, reload_start=6.0)]
        p.touchdown_torques = {1: [0.0] * 10}
        p.base_time = lambda t: t
        p.total = 10.0
        p.viscous = [2.0] * 10
        p.derivative_gains = lambda t: [8.0] * 10
        return p

    def test_reference_derivative_and_damping_compensation(self):
        r = self.program(True).sample(4.0)
        for v, ff, base in zip(
            r["reference_velocity_rad_s"],
            r["feedforward_nm"],
            r["gravity_feedforward_nm"],
        ):
            self.assertAlmostEqual(v, 8.0, places=9)
            self.assertAlmostEqual(ff, 83.0, places=8)
            self.assertEqual(base, 3.0)

    def test_default_preserves_feedforward_and_final_velocity_zero(self):
        self.assertEqual(self.program(False).sample(4.0)["feedforward_nm"], [3.0] * 10)
        self.assertEqual(
            self.program(True).sample(11.0)["velocity_feedforward_nm"], [0.0] * 10
        )

    def test_touchdown_seed_is_ramped_and_removed_before_final_hold(self):
        p = self.program(True)
        p.touchdown_torques = {1: [0.01] * 10}
        self.assertEqual(p.sample(0.0)["touchdown_preload_torque_nm"], [0.0] * 10)
        self.assertEqual(p.sample(5.5)["touchdown_preload_torque_nm"], [0.005] * 10)
        self.assertEqual(p.sample(8.0)["touchdown_preload_torque_nm"], [0.0] * 10)
        r = p.sample(5.5)
        for total, static, velocity, contact in zip(
            r["feedforward_nm"],
            r["gravity_feedforward_nm"],
            r["velocity_feedforward_nm"],
            r["touchdown_preload_torque_nm"],
        ):
            self.assertAlmostEqual(total, static + velocity + contact)
