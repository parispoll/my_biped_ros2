"""Shutdown ordering and error propagation for the effort controller."""

import importlib.util
from pathlib import Path
import signal
import unittest
from unittest.mock import Mock, patch

PATH = (
    Path(__file__).resolve().parents[1]
    / "src/my_robot_description/nodes/control/hold_roll_stance.py"
)


class RollControllerLifecycleTests(unittest.TestCase):
    def module(self):
        spec = importlib.util.spec_from_file_location("roll_lifecycle", PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def exercise(self, error=None):
        m = self.module()
        calls = []
        handlers = {}
        node = Mock()
        executor = Mock()
        node.supervision_timer.cancel.side_effect = lambda: calls.append("cancel_timer")
        executor.shutdown.side_effect = lambda: calls.append("stop_executor")
        node.close.side_effect = lambda: calls.append("zero_and_close")
        node.destroy_node.side_effect = lambda: calls.append("destroy_node")

        def install(sig, handler):
            handlers[sig] = handler
            return signal.SIG_DFL

        def callback(**kwargs):
            calls.append("callback")
            if error:
                raise error
            handlers[signal.SIGTERM](signal.SIGTERM, None)
            calls.append("callback_finished")

        executor.spin_once.side_effect = callback
        with patch.object(m.signal, "signal", side_effect=install), patch.object(
            m.rclpy, "init"
        ) as initialize, patch.object(m.rclpy, "ok", return_value=True), patch.object(
            m.rclpy, "shutdown", side_effect=lambda: calls.append("shutdown_context")
        ), patch.object(
            m, "RollStanding", return_value=node
        ), patch.object(
            m, "SingleThreadedExecutor", return_value=executor
        ):
            if error:
                with self.assertRaisesRegex(RuntimeError, "real callback failure"):
                    m.main()
            else:
                m.main()
            initialize.assert_called_once_with(
                signal_handler_options=m.SignalHandlerOptions.NO
            )
        return calls

    def test_signal_finishes_callback_before_context_shutdown(self):
        self.assertEqual(
            self.exercise(),
            [
                "callback",
                "callback_finished",
                "cancel_timer",
                "stop_executor",
                "zero_and_close",
                "destroy_node",
                "shutdown_context",
            ],
        )

    def test_real_callback_failure_is_not_suppressed(self):
        self.assertEqual(
            self.exercise(RuntimeError("real callback failure")),
            [
                "callback",
                "cancel_timer",
                "stop_executor",
                "zero_and_close",
                "destroy_node",
                "shutdown_context",
            ],
        )
