"""A preflight for a different count/distance/timing must not unlock a live run."""

import sys, tempfile, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from force_panel.config import ExperimentRequest
from force_panel.jobs import JobManager


class ProfilePanelTests(unittest.TestCase):
    def test_profile_command_and_mismatched_preflight(self):
        request = dict(
            mode="alternating_walk",
            walk_steps=10,
            walk_step_mm=75,
            walk_time_scale=0.25,
            lowering_mm=40,
            lift_mm=30,
            shift_side="left",
            force_n=0,
            balance_enabled=False,
            gazebo_gui=False,
        )
        command = ExperimentRequest.parse(request).command(
            ROOT, Path("/tmp/profile-test")
        )
        for flag, value in [
            ("--walk-steps", "10"),
            ("--walk-step-mm", "75"),
            ("--walk-time-scale", "0.25"),
        ]:
            self.assertEqual(command[command.index(flag) + 1], value)
        for field, value in [
            ("walk_steps", True),
            ("walk_step_mm", 75.0),
            ("walk_time_scale", True),
        ]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(request, **{field: value}))
        with tempfile.TemporaryDirectory() as folder:
            manager = JobManager(folder)
            manager.jobs["first"] = dict(
                status="completed",
                request=dict(mode="first_step", shift_side="left"),
                result=dict(passed=True),
            )
            for changes in [
                dict(step_count=3),
                dict(nominal_step_length_m=0.05),
                dict(time_scale=0.5),
            ]:
                plan = dict(step_count=10, nominal_step_length_m=0.075, time_scale=0.25)
                plan.update(changes)
                manager.jobs["preview"] = dict(
                    status="completed",
                    request=dict(mode="alternating_walk_preview", shift_side="left"),
                    result=dict(
                        passed=True, preview_kind="alternating_walk", motion_plan=plan
                    ),
                )
                with self.assertRaisesRegex(ValueError, "matching full-path"):
                    manager.start(request)


if __name__ == "__main__":
    unittest.main()
