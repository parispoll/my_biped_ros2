"""Regression checks for test lateral pulse. See the folder README for its role."""

import json
from pathlib import Path
import tempfile
import unittest
from my_robot_description.experiments.lateral_pulse import latest_record
from my_robot_description.visualization.force_visual import lateral_arrow_markers
from gz.msgs10.marker_pb2 import Marker


class LateralPulseTests(unittest.TestCase):
    def test_partial_tail_never_becomes_a_complete_control_record(self):
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "joints.jsonl"
            p.write_text(
                json.dumps(dict(sim_time=1.0, shift=dict(elapsed_s=0.5)))
                + "\n"
                + str('{"sim_time":2')
            )
            self.assertEqual(latest_record(p)["sim_time"], 1.0)
            p.write_text("")
            self.assertIsNone(latest_record(p))

    def test_lateral_marker_points_along_y_and_has_no_physics(self):
        for force in [-1.0, 1.0]:
            b = lateral_arrow_markers(force, dict(x=0.12, y=0.3, height=0.55), "test")
            shaft, head = b.marker
            self.assertAlmostEqual(shaft.pose.position.x, 0.12)
            self.assertGreater(shaft.scale.y, shaft.scale.x)
            self.assertEqual(head.type, Marker.CONE)
            self.assertAlmostEqual(head.pose.orientation.y, 0.0)
            self.assertLess(head.pose.orientation.x * force, 0.0)
            self.assertEqual(head.pose.position.z, 0.55)
        self.assertTrue(
            all(
                m.action == Marker.DELETE_MARKER
                for m in lateral_arrow_markers(
                    0.0, dict(x=0, y=0, height=0.5), "test"
                ).marker
            )
        )
