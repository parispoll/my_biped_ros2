"""Keep the physical model intact while changing only damping integration."""

import copy
import importlib.util
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
import xacro
from my_robot_description.studies.damping import validate_damping_trace

ROOT = Path(__file__).resolve().parents[1]


class DampingTests(unittest.TestCase):
    def rows(self):
        return [
            dict(
                joint=f"{side}_{joint}",
                coefficient=b,
                velocity=0.2,
                torque=-b * 0.2,
                power=-b * 0.04,
                max_power_in_block=0.0,
                steps=10,
            )
            for side in ("left", "right")
            for joint, b in (("hip", 2.5), ("knee", 3.0), ("ankle", 2.5))
        ]

    def test_constitutive_law_and_block_passivity(self):
        self.assertTrue(validate_damping_trace(self.rows())["passed"])
        for key, value in [
            ("torque", 1.0),
            ("power", 1.0),
            ("max_power_in_block", 0.01),
        ]:
            rows = self.rows()
            rows[0][key] = value
            self.assertFalse(validate_damping_trace(rows)["passed"])
        for key, value in [
            ("coefficient", 0.0),
            ("velocity", float("nan")),
            ("steps", 11),
        ]:
            rows = self.rows()
            rows[0][key] = value
            with self.assertRaises(ValueError):
                validate_damping_trace(rows)
        with self.assertRaises(ValueError):
            validate_damping_trace(self.rows()[:-1])

    def test_recorded_motor_and_total_inputs_respect_individual_caps(self):
        rows = self.rows()
        for r in rows:
            r.update(
                motor_nm=0.25,
                total_nm=0.25 + r["torque"],
                max_abs_motor_in_block_nm=0.3,
            )
        self.assertTrue(validate_damping_trace(rows)["motor_input_bounds_checked"])
        self.assertTrue(validate_damping_trace(rows)["passed"])
        rows[0]["max_abs_motor_in_block_nm"] = 6.1
        self.assertFalse(validate_damping_trace(rows)["passed"])
        rows[0]["max_abs_motor_in_block_nm"] = 0.3
        rows[0]["total_nm"] = 1.0
        self.assertFalse(validate_damping_trace(rows)["passed"])

    def test_explicit_damping_preserves_every_other_link_and_joint_property(self):
        path = ROOT / "src/my_robot_description/urdf/effort_standing.urdf.xacro"
        implicit = ET.fromstring(xacro.process_file(str(path)).toxml())
        explicit = ET.fromstring(
            xacro.process_file(
                str(path), mappings={"damping_model": "explicit"}
            ).toxml()
        )
        self.assertEqual(
            [ET.tostring(e) for e in implicit.findall("link")],
            [ET.tostring(e) for e in explicit.findall("link")],
        )
        plugin = explicit.find("gazebo/plugin[@name='biped::ViscousDamping']")
        self.assertIsNotNone(plugin)
        self.assertIsNone(implicit.find("gazebo/plugin[@name='biped::ViscousDamping']"))
        for before, after in zip(implicit.findall("joint"), explicit.findall("joint")):
            adjusted = copy.deepcopy(after)
            b = before.find("dynamics").get("damping")
            self.assertEqual(
                float(plugin.find(f"joint[@name='{before.get('name')}']/damping").text),
                float(b),
            )
            self.assertEqual(float(adjusted.find("dynamics").get("damping")), 0.0)
            adjusted.find("dynamics").set("damping", b)
            self.assertEqual(ET.tostring(before), ET.tostring(adjusted))

    def test_disabled_optional_recordings_are_omitted_from_ros_cli(self):
        spec = importlib.util.spec_from_file_location(
            "effort_runner", ROOT / "scripts/experiments/run_effort_push.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        folder = Path("/tmp/folder with spaces")
        args = module.launch_command(
            folder, False, 0.2, False, False, 1.0, 0.001, "implicit"
        )
        self.assertFalse(any(a.endswith(":=") for a in args))
        self.assertFalse(any(a.startswith("physics_recording:=") for a in args))
        self.assertFalse(any(a.startswith("balance_recording:=") for a in args))
        args = module.launch_command(
            folder, False, 0.2, True, True, 1.0, 0.001, "explicit", "pid"
        )
        self.assertIn(f"physics_recording:={folder}/physics.csv", args)
        self.assertIn(f"damping_recording:={folder}/viscous.csv", args)
        self.assertIn("controller_mode:=pid", args)
        args = module.launch_command(
            folder, False, 0.2, False, False, 0.5, 0.001, "explicit", "pid", True
        )
        self.assertIn(f"balance_recording:={folder}/balance.jsonl", args)

    def test_existing_evidence_is_rejected_before_launching_simulation(self):
        spec = importlib.util.spec_from_file_location(
            "effort_runner", ROOT / "scripts/experiments/run_effort_push.py"
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        with tempfile.TemporaryDirectory() as folder, patch(
            "sys.argv", ["run_effort_push.py", "--output", folder]
        ), patch.object(module.subprocess, "Popen") as launch:
            with contextlib.redirect_stderr(
                io.StringIO()
            ) as message, self.assertRaises(SystemExit) as error:
                module.main()
            self.assertEqual(error.exception.code, 2)
            self.assertIn("Choose a new --output name", message.getvalue())
            launch.assert_not_called()
