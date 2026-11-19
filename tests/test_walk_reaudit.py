"""Read-only re-audit cannot execute a simulator or hide unrelated source edits."""

import hashlib, sys, tempfile, unittest
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.append(str(ROOT / "scripts"))
from force_panel.config import ExperimentRequest
import scripts.analysis.reaudit_alternating_walk as audit


class ReauditTests(unittest.TestCase):
    def test_fixed_saved_recording_command(self):
        p = dict(
            mode="alternating_walk",
            source_job="20261113-175441-03995179",
            gazebo_gui=False,
            lift_mm=30,
            lowering_mm=40,
            force_n=0,
            balance_enabled=False,
        )
        command = ExperimentRequest.parse(p).command(ROOT, Path("/tmp/re-audit"))
        self.assertIn(
            str(ROOT / "scripts/analysis/reaudit_alternating_walk.py"), command
        )
        self.assertNotIn("--gui", command)
        self.assertNotIn("--alternating-walk", command)
        for bad in [
            dict(source_job="../escape"),
            dict(gazebo_gui=True),
            dict(mode="first_step"),
        ]:
            with self.assertRaises(ValueError):
                ExperimentRequest.parse(dict(p, **bad))

    def test_only_precise_postrun_pairing_fix_can_differ(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            name = "src/my_robot_description/my_robot_description/motion/zero_load.py"
            file = root / name
            file.parent.mkdir(parents=True)
            file.write_text("        " + audit.FIX + "\n")
            recorded = {
                name: hashlib.sha256(
                    ("        " + audit.OLD + "\n").encode()
                ).hexdigest()
            }
            with patch.object(audit, "ROOT", root):
                self.assertEqual(len(audit.verify_runtime_sources(recorded)), 1)
                file.write_text(file.read_text() + "# unrelated change\n")
                with self.assertRaisesRegex(ValueError, "precise integer"):
                    audit.verify_runtime_sources(recorded)

    def test_motor_source_change_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            file = root / "motor.py"
            file.write_text("changed motor caps")
            with patch.object(audit, "ROOT", root):
                with self.assertRaisesRegex(ValueError, "Unexpected post-run"):
                    audit.verify_runtime_sources(
                        {"motor.py": hashlib.sha256(b"original motor caps").hexdigest()}
                    )
