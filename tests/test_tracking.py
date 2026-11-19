"""Failure traces must survive normal exception cleanup and name the worst joint."""

import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as Item
import unittest
from my_robot_description.validation.tracking import TrackingRecorder


def point(positions):
    return Item(
        positions=positions, velocities=[], time_from_start=Item(sec=1, nanosec=0)
    )


class TrackingRecorderTests(unittest.TestCase):
    def test_failed_motion_preserves_desired_actual_and_worst_error(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "trace.jsonl"
            recorder = TrackingRecorder(str(path))
            recorder.phase = "active"
            message = Item(
                header=Item(stamp=Item(sec=2, nanosec=0)),
                joint_names=["hip", "knee"],
                reference=point([-0.04, 0.08]),
                feedback=point([-0.039, 0]),
                error=point([-0.001, 0.08]),
                output=point([-0.04, 0.08]),
            )
            recorder.record(message, 2)
            report = recorder.finish("failed", {"path_tolerance": 0.08})
            sample = json.loads(path.read_text())
            self.assertEqual(sample["desired"], [-0.04, 0.08])
            self.assertEqual(sample["actual"], [-0.039, 0])
            self.assertEqual(report["worst_tracking"]["joint"], "knee")
            self.assertEqual(report["worst_tracking"]["error"], 0.08)
            self.assertEqual(
                json.loads(path.with_suffix(".summary.json").read_text())["result"],
                "failed",
            )
