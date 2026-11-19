"""A saved fault preserves the actual exception and last plan/sensor stamps."""

import json, tempfile, unittest
from pathlib import Path
from types import SimpleNamespace
from my_robot_description.runtime.fault_report import save_fault


class FaultTests(unittest.TestCase):
    def test_preserve_failure(self):
        node = SimpleNamespace(
            zero_contact_rows={"left": {"sim_time": 12, "message": {"contacts": []}}},
            last_stamp=12,
            imu_stamp=11.99,
            tilt=0.01,
            interactive=SimpleNamespace(
                status=lambda ready: dict(ready=ready, completed_steps=1)
            ),
            ready=True,
        )
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "nested/fault.json"
            save_fault(path, node, "RuntimeError: example guard")
            report = json.loads(path.read_text())
            self.assertIn("example guard", report["traceback"])
            self.assertEqual(report["session"]["completed_steps"], 1)
            self.assertEqual(report["joint_stamp"], 12)
            self.assertEqual(
                report["raw_contact_rows"]["left"]["message"]["contacts"], []
            )
        save_fault("", node, "no file requested")
