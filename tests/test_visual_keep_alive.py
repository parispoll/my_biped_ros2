"""Check visual launch exit handling independently of a live ROS graph."""

import ast
from pathlib import Path
import unittest


class VisualExitPolicyTests(unittest.TestCase):
    def policy(self, enabled):
        source = Path(
            "src/my_robot_description/launch/effort_standing.launch.py"
        ).read_text()
        setup = next(
            n
            for n in ast.parse(source).body
            if isinstance(n, ast.FunctionDef) and n.name == "setup"
        )
        policy = next(
            n
            for n in setup.body
            if isinstance(n, ast.FunctionDef) and n.name == "process_exit"
        )
        namespace = dict(
            visual_keep_alive=enabled,
            LogInfo=lambda **kw: kw,
            stop_on_exit=lambda name: ("shutdown", name),
        )
        exec(
            compile(
                ast.Module(body=[policy], type_ignores=[]),
                "<launch-exit-policy>",
                "exec",
            ),
            namespace,
        )
        return namespace["process_exit"]

    def test_visual_control_exits_preserve_gazebo(self):
        for name in ("posture", "contact-sampler", "balance-observer"):
            self.assertIn("Gazebo remains open", self.policy(True)(name)[0]["msg"])

    def test_measured_experiment_exits_still_shutdown(self):
        for name in (
            "posture",
            "contact-sampler",
            "balance-observer",
            "gazebo",
            "clock",
        ):
            self.assertEqual(self.policy(False)(name), ("shutdown", name))

    def test_visual_infrastructure_exits_still_shutdown(self):
        for name in ("gazebo", "clock", "description", "bridge-0"):
            self.assertEqual(self.policy(True)(name), ("shutdown", name))

    def test_full_visual_settling_never_changes_measured_readiness(self):
        source = Path(
            "src/my_robot_description/launch/effort_standing.launch.py"
        ).read_text()
        tree = ast.parse(source)
        value = next(
            value
            for node in ast.walk(tree)
            if isinstance(node, ast.Dict)
            for key, value in zip(node.keys, node.values)
            if isinstance(key, ast.Constant) and key.value == "visual_fast_ready"
        )
        expression = compile(ast.Expression(body=value), "<readiness-routing>", "eval")
        for keep, flag, expected in (
            (False, "true", False),
            (False, "false", False),
            (True, "true", True),
            (True, "false", False),
        ):
            self.assertEqual(
                eval(
                    expression, dict(visual_keep_alive=keep, setting=lambda name: flag)
                ),
                expected,
            )
