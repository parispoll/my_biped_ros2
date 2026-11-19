"""A runner's readiness error must include the underlying launch exception."""

from pathlib import Path
import tempfile
import unittest
from my_robot_description.validation.experiment_failure import launch_failure


class LaunchFailureTests(unittest.TestCase):
    def test_secondary_shutdown_exception_cannot_hide_triggering_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / "launch.log"
            log.write_text(
                "[hold] ValueError: Joint feedback timing invalid\n[observer] RuntimeError: Cleanup context invalid\n"
            )
            self.assertEqual(
                launch_failure(log), "ValueError: Joint feedback timing invalid"
            )

    def test_controller_exception_survives_later_shutdown_messages(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / "launch.log"
            log.write_text(
                "[hold-12] \x1b[31mValueError: Joint feedback timing invalid: source_age=-0.572000 s\x1b[0m\n[INFO] [gazebo-1]: process has finished cleanly\n"
            )
            self.assertEqual(
                launch_failure(log),
                "ValueError: Joint feedback timing invalid: source_age=-0.572000 s",
            )

    def test_timeout_without_python_exception_keeps_diagnostic_hint(self):
        with tempfile.TemporaryDirectory() as folder:
            log = Path(folder) / "launch.log"
            log.write_text("[INFO] waiting for service\n")
            self.assertIn("readiness timeout", launch_failure(log))
