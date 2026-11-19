"""Verify schematic direction, marker ownership and independence from force timing."""

import threading
import time
import unittest
from types import SimpleNamespace
from gz.msgs10.marker_pb2 import Marker
from my_robot_description.visualization.force_visual import (
    arrow_markers,
    delete_markers,
    ForceArrowVisualizer,
)

BODY = dict(x=0.4, y=0.2, height=0.55)


class ArrowGeometryTests(unittest.TestCase):
    def test_positive_and_negative_heads_point_toward_pelvis(self):
        for force in (1.0, -1.0):
            with self.subTest(force=force):
                shaft, head = arrow_markers(force, BODY, "owned").marker
                direction = 1 if force > 0 else -1
                self.assertLess(
                    direction * shaft.pose.position.x, direction * head.pose.position.x
                )
                self.assertAlmostEqual(
                    head.pose.position.x + direction * head.scale.z / 2,
                    BODY["x"] - direction * 0.13,
                )
                # Rotate the cone's local +Z tip direction by its quaternion.
                self.assertAlmostEqual(
                    2 * head.pose.orientation.w * head.pose.orientation.y, direction
                )
                self.assertEqual(shaft.type, Marker.BOX)
                self.assertEqual(head.type, Marker.CONE)
                self.assertEqual(head.pose.position.z, BODY["height"])
                self.assertGreater(head.lifetime.nsec, 0)

    def test_magnitude_changes_length_but_is_clamped_for_readability(self):
        length = lambda force: arrow_markers(force, BODY, "test").marker[0].scale.x
        self.assertGreater(length(2), length(1))
        self.assertEqual(length(100), length(1000))
        self.assertEqual(length(2), length(-2))

    def test_zero_and_cleanup_delete_only_owned_ids(self):
        for batch in (arrow_markers(0.0, BODY, "mine"), delete_markers("mine")):
            self.assertEqual([m.id for m in batch.marker], [1, 2])
            self.assertTrue(
                all(
                    m.ns == "mine" and m.action == Marker.DELETE_MARKER
                    for m in batch.marker
                )
            )


class ArrowWorkerTests(unittest.TestCase):
    def wait_for(self, predicate):
        deadline = time.monotonic() + 2
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertTrue(predicate())

    def test_show_hide_and_close_clear_only_this_visualizer(self):
        class FakeNode:
            def __init__(self):
                self.messages = []

            def service_list(self):
                return ["/marker_array"]

            def request(self, service, request, *args):
                self.messages.append(request)
                return True, SimpleNamespace(data=True)

        node = FakeNode()
        visual = ForceArrowVisualizer(lambda: node)
        try:
            visual.update(1.0, BODY)
            self.wait_for(lambda: visual.status()["updates_accepted"] > 0)
            visual.hide()
            self.wait_for(lambda: visual.status()["clear_accepted"])
            self.assertTrue(
                all(
                    m.ns == visual.namespace
                    for batch in node.messages
                    for m in batch.marker
                )
            )
            self.assertEqual(node.messages[-1].marker[0].action, Marker.DELETE_MARKER)
        finally:
            visual.close()
        self.assertFalse(visual.worker.is_alive())

    def test_slow_gui_never_blocks_force_loop_updates(self):
        entered = threading.Event()
        release = threading.Event()

        class SlowNode:
            def service_list(self):
                return ["/marker_array"]

            def request(self, *args):
                entered.set()
                release.wait(1)
                return True, SimpleNamespace(data=True)

        visual = ForceArrowVisualizer(SlowNode)
        try:
            visual.update(1.0, BODY)
            self.assertTrue(entered.wait(1))
            # Exercise update/hide while a marker request is still blocked.
            visual.update(-1.0, BODY)
            visual.hide()
            self.assertFalse(release.is_set())
        finally:
            release.set()
            visual.close()
        self.assertTrue(visual.status()["clear_accepted"])

    def test_missing_renderer_is_optional_and_zero_force_never_draws(self):
        class HeadlessNode:
            def service_list(self):
                return []

            def request(self, *args):
                raise AssertionError("No renderer service")

        visual = ForceArrowVisualizer(HeadlessNode)
        try:
            visual.update(0.0, BODY)
            self.assertEqual(visual.status()["updates_accepted"], 0)
        finally:
            visual.close()
        self.assertFalse(visual.status()["service_seen"])
