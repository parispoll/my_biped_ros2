"""Initial DDS discovery must not be treated as a broken established IMU stream."""

import importlib.util
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
from my_robot_description.control.imu_watchdog import check_imu_watchdog
from my_robot_description.sensing.balance import BalanceConfig
from sensor_msgs.msg import Imu


class ImuWatchdogTests(unittest.TestCase):
    def test_exact_startup_failure_now_waits_for_first_message(self):
        check_imu_watchdog(100.0, None, 102.2)
        check_imu_watchdog(100.0, None, 109.9)

    def test_no_first_sample_still_times_out(self):
        with self.assertRaisesRegex(RuntimeError, "first IMU sample"):
            check_imu_watchdog(100.0, None, 110.1)

    def test_running_deadline_is_two_seconds_even_inside_discovery_window(self):
        check_imu_watchdog(100.0, 101.0, 102.9)
        with self.assertRaisesRegex(RuntimeError, "absent for two wall seconds"):
            check_imu_watchdog(100.0, 101.0, 103.1)


class ObserverWiringTests(unittest.TestCase):
    def setup_node(self):
        path = (
            Path(__file__).resolve().parents[1]
            / "src/my_robot_description/nodes/legacy/observe_balance.py"
        )
        spec = importlib.util.spec_from_file_location("imu_observer_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node = SimpleNamespace(
            started_wall=0.0,
            last_wall=None,
            last_stamp=None,
            recording=io.StringIO(),
            config=BalanceConfig(),
            get_logger=Mock(),
        )
        node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)
        )
        return module, node

    def message(self):
        msg = Imu()
        msg.header.frame_id = "base_link"
        msg.header.stamp.sec = 1
        msg.orientation.w = 1.0
        return msg

    def test_supervisor_allows_delayed_discovery(self):
        module, node = self.setup_node()
        with patch.object(module.time, "monotonic", return_value=3.2):
            module.BalanceObserver.supervise(node)
        self.assertIsNone(node.last_wall)

    def test_first_valid_sample_arms_existing_stream_watchdog(self):
        module, node = self.setup_node()
        with patch.object(module.time, "monotonic", return_value=3.2):
            module.BalanceObserver.observe(node, self.message())
        self.assertEqual(node.last_wall, 3.2)
        self.assertIn("sim_time", node.recording.getvalue())
        node.get_logger().info.assert_called_once()
        with patch.object(
            module.time, "monotonic", return_value=5.3
        ), self.assertRaises(RuntimeError):
            module.BalanceObserver.supervise(node)

    def test_invalid_first_imu_does_not_mark_observer_ready(self):
        module, node = self.setup_node()
        msg = self.message()
        msg.orientation_covariance[0] = -1
        with self.assertRaises(ValueError):
            module.BalanceObserver.observe(node, msg)
        self.assertIsNone(node.last_wall)
        self.assertEqual(node.recording.getvalue(), "")
