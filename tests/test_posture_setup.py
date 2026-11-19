"""Regression checks for test posture setup. See the folder README for its role."""

import json
from pathlib import Path
import tempfile
import unittest
import importlib.util
import sys
from unittest.mock import Mock, patch
from my_robot_description.control.posture_setup import load_posture_setup


class SetupTests(unittest.TestCase):
    def test_recorder_setup_path_never_queries_live_controller_parameters(self):
        scripts = Path(__file__).resolve().parents[1] / "scripts"
        sys.path.insert(0, str(scripts))
        try:
            spec = importlib.util.spec_from_file_location(
                "effort_recorder_setup_test",
                scripts / "experiments/effort_push_experiment.py",
            )
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            with tempfile.TemporaryDirectory() as d:
                p = Path(d) / "setup.json"
                p.write_text(
                    json.dumps(
                        dict(
                            schema=1,
                            target_joint_positions_rad=[0.1] * 6,
                            motor_limits_nm=[6.0, 10.0, 6.0] * 2,
                        )
                    )
                )
                node = Mock()
                with patch.object(
                    module,
                    "AsyncParameterClient",
                    side_effect=AssertionError("Live metadata query forbidden"),
                ):
                    module.attach_effort_stream(node, {}, [], None, setup=p)
                self.assertEqual(node.create_subscription.call_count, 2)
        finally:
            sys.path.pop(0)

    def test_uses_actual_targets_without_assuming_nominal_knee(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "setup.json"
            p.write_text(
                json.dumps(
                    dict(
                        schema=1,
                        target_joint_positions_rad=[0.1] * 6,
                        motor_limits_nm=[6.0, 10.0, 6.0] * 2,
                    )
                )
            )
            self.assertEqual(load_posture_setup(p), ([0.1] * 6, [6.0, 10.0, 6.0] * 2))
            p.write_text(
                json.dumps(
                    dict(
                        schema=1,
                        target_joint_positions_rad=[0.1] * 6,
                        motor_limits_nm=[0.0] * 6,
                    )
                )
            )
            with self.assertRaises(ValueError):
                load_posture_setup(p)
