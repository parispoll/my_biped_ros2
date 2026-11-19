"""Regression checks for test com markers. See the folder README for its role."""

import unittest
from gz.msgs10.marker_pb2 import Marker
from my_robot_description.visualization.com_markers import (
    make_markers,
    delete_markers,
    IDS,
)


class ComMarkerTests(unittest.TestCase):
    def test_marker_geometry_preserves_true_com_and_projection(self):
        state = dict(
            com=[0.2, 0.1, 0.4],
            feet={
                s: [[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0]]
                for s in ("left", "right")
            },
        )
        batch = make_markers(state, dict(hull=[]), [(0.1, 0.1), (0.2, 0.1)], "owned")
        markers = {m.id: m for m in batch.marker}
        self.assertEqual(markers[1].pose.position.z, 0.4)
        self.assertEqual(markers[3].pose.position.x, 0.2)
        self.assertAlmostEqual(markers[3].pose.position.z, 0.006)
        self.assertEqual(markers[7].action, Marker.DELETE_MARKER)
        self.assertEqual(markers[8].point[-1].x, 0.2)
        self.assertTrue(all(m.ns == "owned" for m in batch.marker))
        self.assertTrue(
            all(
                m.lifetime.nsec > 0
                for m in batch.marker
                if m.action == Marker.ADD_MODIFY
            )
        )

    def test_cleanup_only_deletes_owned_ids(self):
        batch = delete_markers("owned")
        self.assertEqual(tuple(m.id for m in batch.marker), IDS)
        self.assertTrue(
            all(
                m.ns == "owned" and m.action == Marker.DELETE_MARKER
                for m in batch.marker
            )
        )
