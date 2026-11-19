"""No integration across unknown feedback intervals; stale samples still fail."""

import unittest
from my_robot_description.control.feedback_timing import (
    integration_timing,
    startup_timing_ready,
)


class TimingTests(unittest.TestCase):
    def test_startup_waits_for_clock_discovery(self):
        # Reproduce the failed GUI run: joint time .572, local ROS clock zero.
        self.assertFalse(startup_timing_ready(0.572, 0.0))
        self.assertFalse(startup_timing_ready(0.572, 0.1))
        self.assertTrue(startup_timing_ready(0.572, 0.573))

    def test_startup_uses_the_same_freshness_bounds_as_active_control(self):
        for stamp, receipt in (
            (1.0, 1.06),
            (1.0, 0.97),
            (0.0, 0.0),
            (float("nan"), 1.0),
            (1.0, float("inf")),
        ):
            with self.subTest(stamp=stamp, receipt=receipt):
                self.assertFalse(startup_timing_ready(stamp, receipt))
        self.assertTrue(startup_timing_ready(1.0, 1.01))

    def test_normal_interval_integrates(self):
        r = integration_timing(1.0, 1.004, 1.006)
        self.assertAlmostEqual(r["integral_dt_s"], 0.004)
        self.assertFalse(r["integral_frozen_gap"])

    def test_fresh_sample_after_gap_freezes_integral(self):
        r = integration_timing(1.0, 1.08, 1.081)
        self.assertEqual(r["integral_dt_s"], 0.0)
        self.assertTrue(r["integral_frozen_gap"])

    def test_stale_future_or_reversed_feedback_is_rejected(self):
        for values in ((1.0, 1.08, 1.14), (1.0, 1.08, 1.0), (1.1, 1.0, 1.0)):
            with self.assertRaises(ValueError):
                integration_timing(*values)


class StartupActivationTests(unittest.TestCase):
    def node(self, clock_seconds):
        import importlib.util
        import time
        from pathlib import Path
        from types import SimpleNamespace
        from unittest.mock import Mock

        path = (
            Path(__file__).resolve().parents[1]
            / "src/my_robot_description/nodes/legacy/hold_effort_stance.py"
        )
        spec = importlib.util.spec_from_file_location("effort_startup_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        now = time.monotonic()
        node = SimpleNamespace(
            active=False,
            started_wall=now,
            last_wall=now,
            imu_wall=now,
            last_terms={},
            last_stamp=0.572,
            parameter_future=None,
            switch_future=None,
            clock_wait_logged=False,
            clock_ready_logged=False,
            pub=Mock(),
            parameters=Mock(),
            switch=Mock(),
            get_logger=Mock(),
        )
        node.pub.get_subscription_count.return_value = 1
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=int(clock_seconds * 1e9))
        )
        return module, node

    def test_clock_zero_prevents_parameter_request_and_activation(self):
        module, node = self.node(0.0)
        module.EffortStanding.supervise(node)
        node.parameters.get_parameters.assert_not_called()
        node.switch.call_async.assert_not_called()
        self.assertFalse(node.active)
        self.assertTrue(node.clock_wait_logged)

    def test_synchronized_clock_allows_normal_startup(self):
        module, node = self.node(0.573)
        module.EffortStanding.supervise(node)
        node.parameters.get_parameters.assert_called_once_with(
            ["enforce_command_limits"]
        )
        self.assertTrue(node.clock_ready_logged)

    def test_clock_that_never_arrives_still_times_out(self):
        import time

        module, node = self.node(0.0)
        node.started_wall = time.monotonic() - 31
        with self.assertRaises(TimeoutError):
            module.EffortStanding.supervise(node)
        node.switch.call_async.assert_not_called()
