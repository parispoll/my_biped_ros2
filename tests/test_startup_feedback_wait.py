"""Startup drops must never command/integrate, nor weaken ready-state guards."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from sensor_msgs.msg import JointState
from my_robot_description.control.feedback_timing import integration_timing
from my_robot_description.control.integral import BoundedIntegral
from my_robot_description.model.stance import JOINT_NAMES


class StartupTimingPolicyTests(unittest.TestCase):
    def test_bad_startup_age_is_rejected_without_integration_terms(self):
        self.assertIsNone(integration_timing(1.0, 1.004, 1.087, wait_for_startup=True))
        self.assertIsNone(integration_timing(1.0, 1.004, 0.98, wait_for_startup=True))

    def test_same_age_still_stops_ready_control(self):
        with self.assertRaises(ValueError):
            integration_timing(1.0, 1.004, 1.087)

    def test_reversed_and_nonfinite_times_always_fail(self):
        for values in ((1.0, 0.9, 1.0), (1.0, float("nan"), 1.0)):
            with self.assertRaises(ValueError):
                integration_timing(*values, wait_for_startup=True)


class StartupFeedbackWiringTests(unittest.TestCase):
    def node(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "src/my_robot_description/nodes/legacy/hold_effort_stance.py"
        )
        spec = importlib.util.spec_from_file_location("waiting_effort_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node = SimpleNamespace(
            active=True,
            ready=False,
            last_stamp=1.0,
            last_wall=10.0,
            settled_since=0.9,
            startup_rejected_feedback_samples=0,
            startup_settle_restart=False,
            active_time=0.0,
            pub=Mock(),
            get_logger=Mock(),
            target=[0.0] * 6,
            ff=[0.0] * 6,
            kp=[10.0] * 6,
            kd=[1.0] * 6,
            limits=[6.0] * 6,
            visual_fast_ready=False,
            mode="pid",
            integrator=BoundedIntegral([1.0] * 6, [1.0] * 6),
            balance_enabled=False,
            tilt=0.0,
            last_terms=None,
            recording=None,
            balance_ready=False,
        )
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=node.clock_ns)
        )
        node.clock_ns = 1_087_000_000
        return module, node

    def message(self, stamp):
        msg = JointState()
        msg.header.stamp.sec = 1
        msg.header.stamp.nanosec = round((stamp - 1) * 1e9)
        msg.name = JOINT_NAMES
        msg.position = [0.0] * 6
        msg.velocity = [0.0] * 6
        return msg

    def test_stale_startup_sample_does_not_publish_advance_or_integrate(self):
        module, node = self.node()
        module.EffortStanding.feedback(node, self.message(1.004))
        node.pub.publish.assert_not_called()
        self.assertEqual(node.last_stamp, 1.0)
        self.assertEqual(node.last_wall, 10.0)
        self.assertEqual(node.integrator.state, [0.0] * 6)
        self.assertIsNone(node.settled_since)
        self.assertTrue(node.startup_settle_restart)

    def test_fresh_sample_restarts_full_settling_and_freezes_gap_integral(self):
        module, node = self.node()
        module.EffortStanding.feedback(node, self.message(1.004))
        node.clock_ns = 1_101_000_000
        with patch.object(module.time, "monotonic", return_value=10.2):
            module.EffortStanding.feedback(node, self.message(1.1))
        self.assertEqual(node.active_time, 1.1)
        self.assertTrue(node.last_terms["integral_frozen_gap"])
        self.assertFalse(node.ready)
        self.assertEqual(node.integrator.state, [0.0] * 6)
        node.pub.publish.assert_called_once()

    def test_visual_readiness_is_faster_without_changing_measured_default(self):
        for visual in (False, True):
            module, node = self.node()
            node.visual_fast_ready = visual
            node.settled_since = 0.4
            node.clock_ns = 1_004_000_000
            module.EffortStanding.feedback(node, self.message(1.004))
            self.assertEqual(node.ready, visual)

    def test_ready_state_still_fails_on_stale_sample(self):
        module, node = self.node()
        node.ready = True
        with self.assertRaises(ValueError):
            module.EffortStanding.feedback(node, self.message(1.004))
        node.pub.publish.assert_not_called()

    def test_stale_startup_data_does_not_refresh_wall_watchdog(self):
        module, node = self.node()
        node.imu_wall = 11.1
        module.EffortStanding.feedback(node, self.message(1.004))
        with patch.object(
            module.time, "monotonic", return_value=11.1
        ), self.assertRaisesRegex(RuntimeError, "Joint feedback absent"):
            module.EffortStanding.supervise(node)
