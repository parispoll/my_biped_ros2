"""Integral contribution, simulation-time integration and motor anti-windup."""

import unittest
from my_robot_description.control.integral import BoundedIntegral


class IntegralTests(unittest.TestCase):
    def make(self):
        return BoundedIntegral([0.0, 0.0, 30.0] * 2, [0.0, 0.0, 1.0] * 2)

    def test_only_ankles_integrate_and_contribution_is_bounded(self):
        term = self.make()
        for _ in range(100):
            value = term.update([1.0] * 6, 0.01, [0.0] * 6, [6.0, 10.0, 6.0] * 2)
        self.assertEqual(value, [0.0, 0.0, 1.0] * 2)
        value[2] = 100
        self.assertEqual(term.state[2], 1.0)

    def test_simulation_duration_determines_integral(self):
        a, b = self.make(), self.make()
        for _ in range(10):
            a.update([0.01] * 6, 0.01, [0.0] * 6, [6.0] * 6)
        for _ in range(20):
            b.update([0.01] * 6, 0.005, [0.0] * 6, [6.0] * 6)
        self.assertAlmostEqual(a.state[2], 0.03)
        self.assertAlmostEqual(a.state[2], b.state[2])

    def test_saturation_freezes_further_windup_but_allows_unwinding(self):
        term = self.make()
        term.update([0.1] * 6, 0.05, [0.0] * 6, [6.0] * 6)
        old = term.state[2]
        term.update([0.1] * 6, 0.05, [7.0] * 6, [6.0] * 6)
        self.assertEqual(term.state[2], old)
        term.update([-0.1] * 6, 0.01, [7.0] * 6, [6.0] * 6)
        self.assertLess(term.state[2], old)
        term.update([-0.1] * 6, 0.05, [-7.0] * 6, [6.0] * 6)
        self.assertGreaterEqual(term.state[2], 0.0)

    def test_invalid_parameters_feedback_and_time_are_rejected(self):
        for gains, limits in [
            ([1.0] * 5, [1.0] * 6),
            ([-1.0] * 6, [1.0] * 6),
            ([1.0] * 6, [0.0] * 6),
        ]:
            with self.assertRaises(ValueError):
                BoundedIntegral(gains, limits)
        for dt in (-0.01, 0.051, float("nan")):
            with self.assertRaises(ValueError):
                self.make().update([0.0] * 6, dt, [0.0] * 6, [6.0] * 6)
        with self.assertRaises(ValueError):
            self.make().update([float("nan")] * 6, 0.01, [0.0] * 6, [6.0] * 6)
