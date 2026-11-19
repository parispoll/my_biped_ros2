"""Check that startup failures cannot launch downstream stages.

Run after sourcing ROS and this workspace: python3 -m unittest discover -s tests
"""

from types import SimpleNamespace
import unittest
from launch.actions import EmitEvent
from my_robot_description.runtime.startup import after_success, stop_on_exit


class StartupFailureTests(unittest.TestCase):
    def test_failed_spawn_does_not_start_controllers(self):
        controller = object()
        actions = after_success("spawn", [controller])(
            SimpleNamespace(returncode=1), SimpleNamespace(is_shutdown=False)
        )
        self.assertNotIn(controller, actions)
        self.assertTrue(any(isinstance(action, EmitEvent) for action in actions))

    def test_success_starts_next_stage(self):
        controller = object()
        actions = after_success("spawn", [controller])(
            SimpleNamespace(returncode=0), SimpleNamespace(is_shutdown=False)
        )
        self.assertIn(controller, actions)

    def test_essential_process_exit_stops_running_launch(self):
        actions = stop_on_exit("gazebo")(
            SimpleNamespace(returncode=0), SimpleNamespace(is_shutdown=False)
        )
        self.assertTrue(any(isinstance(action, EmitEvent) for action in actions))

    def test_expected_shutdown_does_not_report_failure(self):
        actions = stop_on_exit("gazebo")(
            SimpleNamespace(returncode=0), SimpleNamespace(is_shutdown=True)
        )
        self.assertEqual(actions, [])
