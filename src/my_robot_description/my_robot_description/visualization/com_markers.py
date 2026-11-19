"""Draw actual COM, its vertical projection, sole outlines and a short trail."""

import math
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V

IDS = tuple(range(1, 10))


def delete_markers(namespace):
    batch = Marker_V()
    for marker_id in IDS:
        batch.marker.add(ns=namespace, id=marker_id, action=Marker.DELETE_MARKER)
    return batch


def make_markers(state, support, trail, namespace):
    batch = Marker_V()
    cyan = (0.05, 0.9, 1.0, 1.0)
    yellow = (1.0, 0.85, 0.05, 1.0)
    white = (0.8, 0.85, 0.9, 0.9)
    green = (0.2, 1.0, 0.4, 1.0)

    def marker(marker_id, kind, color):
        m = batch.marker.add(
            ns=namespace, id=marker_id, action=Marker.ADD_MODIFY, type=kind
        )
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.lifetime.nsec = 300_000_000
        for c in (m.material.ambient, m.material.diffuse, m.material.emissive):
            c.r, c.g, c.b, c.a = color
        m.material.lighting = False
        return m

    def lines(marker_id, points, color, closed=False):
        if not points:
            batch.marker.add(ns=namespace, id=marker_id, action=Marker.DELETE_MARKER)
            return
        m = marker(marker_id, Marker.LINE_STRIP, color)
        for p in list(points) + ([points[0]] if closed else []):
            q = m.point.add()
            q.x, q.y, q.z = p

    x, y, z = state["com"]
    m = marker(1, Marker.SPHERE, cyan)
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.scale.x = m.scale.y = m.scale.z = 0.075
    # A true-location crossbar reaches outside the robot to remain visible.
    lines(2, [(x, y - 0.20, z), (x, y + 0.20, z)], cyan)
    m = marker(3, Marker.CYLINDER, yellow)
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = 0.006
    m.scale.x = m.scale.y = 0.045
    m.scale.z = 0.008
    lines(4, [(x, y, 0.012), (x, y, z)], yellow)
    for marker_id, side in ((5, "left"), (6, "right")):
        lines(
            marker_id,
            [(p[0], p[1], max(0.004, p[2] + 0.004)) for p in state["feet"][side]],
            white,
            closed=True,
        )
    lines(7, [(p[0], p[1], 0.009) for p in support["hull"]], green, closed=True)
    lines(8, [(p[0], p[1], 0.012) for p in trail], yellow)
    # A large locator ring is schematic; its center is the true projection.
    lines(
        9,
        [
            (
                x + 0.12 * math.cos(i * math.tau / 64),
                y + 0.12 * math.sin(i * math.tau / 64),
                0.008,
            )
            for i in range(64)
        ],
        yellow,
        closed=True,
    )
    return batch
