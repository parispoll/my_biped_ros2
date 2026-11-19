"""Optional Gazebo GUI markers for a commanded sagittal force.

Marker transport runs independently so waiting for a renderer never delays the
experiment's force publication or clearing. Markers have no collision/inertia.
"""

import math
import threading
import uuid
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.msgs10.boolean_pb2 import Boolean
from gz.transport13 import Node


def arrow_markers(force_x, body, namespace):
    """A world-aligned box shaft and cone head, ending beside the pelvis.

    This schematic scale is clamped for readability, not a physical force scale.
    A cone is used because Harmonic's MarkerManager does not render ARROW.
    """
    if not math.isfinite(force_x) or not all(
        math.isfinite(body[k]) for k in ("x", "y", "height")
    ):
        raise ValueError("Finite force and body position required")
    if force_x == 0:
        return delete_markers(namespace)
    direction = 1 if force_x > 0 else -1
    length = min(0.55, 0.20 + 0.10 * abs(force_x))
    head_length = 0.09
    tip = body["x"] - direction * 0.13
    batch = Marker_V()
    for marker_id, kind, center, scale in (
        (
            1,
            Marker.BOX,
            tip - direction * (head_length + (length - head_length) / 2),
            (length - head_length, 0.025, 0.025),
        ),
        (
            2,
            Marker.CONE,
            tip - direction * head_length / 2,
            (0.075, 0.075, head_length),
        ),
    ):
        marker = batch.marker.add(
            ns=namespace, id=marker_id, action=Marker.ADD_MODIFY, type=kind
        )
        marker.pose.position.x = center
        marker.pose.position.y = body["y"]
        marker.pose.position.z = body["height"]
        marker.pose.orientation.w = 1.0
        if kind == Marker.CONE:
            marker.pose.orientation.y = direction * math.sin(math.pi / 4)
            marker.pose.orientation.w = math.cos(math.pi / 4)
        marker.scale.x, marker.scale.y, marker.scale.z = scale
        for color in (
            marker.material.ambient,
            marker.material.diffuse,
            marker.material.emissive,
        ):
            color.r, color.g, color.b, color.a = 1.0, 0.32, 0.04, 1.0
        marker.material.lighting = False
        # A finite simulated lifetime also removes a stranded visual after a crash.
        marker.lifetime.nsec = 200_000_000
    return batch


def lateral_arrow_markers(force_y, body, namespace):
    """Rotate the existing schematic arrow into world Y; physics is unchanged."""
    batch = arrow_markers(
        force_y, dict(x=body["y"], y=-body["x"], height=body["height"]), namespace
    )
    direction = 1 if force_y > 0 else -1
    for marker in batch.marker:
        if marker.action == Marker.DELETE_MARKER:
            continue
        x, y = marker.pose.position.x, marker.pose.position.y
        marker.pose.position.x = -y
        marker.pose.position.y = x
        if marker.type == Marker.BOX:
            marker.scale.x, marker.scale.y = marker.scale.y, marker.scale.x
        else:
            marker.pose.orientation.x = -direction * math.sin(math.pi / 4)
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.w = math.cos(math.pi / 4)
    return batch


def delete_markers(namespace):
    batch = Marker_V()
    for marker_id in (1, 2):
        batch.marker.add(ns=namespace, id=marker_id, action=Marker.DELETE_MARKER)
    return batch


class ForceArrowVisualizer:
    """A latest-value mailbox; GUI services are never called on the force loop."""

    def __init__(self, node_factory=Node, axis="x"):
        if axis not in ("x", "y"):
            raise ValueError("Force marker axis must be x or y")
        self.axis = axis
        self.namespace = "biped_force_" + uuid.uuid4().hex
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.target = None
        self.node_factory = node_factory
        self.report = dict(
            service_seen=False,
            updates_accepted=0,
            clear_accepted=False,
            errors=0,
            meaning="Optional GUI illustration of the command window; not measured force or exact actuation timing.",
        )
        self.worker = threading.Thread(
            target=self._run, name="force-arrow", daemon=True
        )
        self.worker.start()

    def update(self, force_x, body):
        with self.lock:
            self.target = (
                (force_x, {key: body[key] for key in ("x", "y", "height")})
                if force_x
                else None
            )

    def hide(self):
        with self.lock:
            self.target = None

    def status(self):
        with self.lock:
            return dict(self.report)

    def _send(self, node, message):
        accepted, response = node.request(
            "/marker_array", message, Marker_V, Boolean, 50
        )
        return accepted and response.data

    def _run(self):
        shown = False
        node = None
        try:
            node = self.node_factory()
            while not self.stop.is_set():
                if "/marker_array" not in node.service_list():
                    self.stop.wait(0.25)
                    continue
                with self.lock:
                    self.report["service_seen"] = True
                    target = self.target
                if target is not None:
                    accepted = self._send(
                        node,
                        (lateral_arrow_markers if self.axis == "y" else arrow_markers)(
                            *target, self.namespace
                        ),
                    )
                    # Even an unacknowledged request could have reached the GUI.
                    shown = True
                    with self.lock:
                        self.report["updates_accepted"] += int(accepted)
                elif shown:
                    accepted = self._send(node, delete_markers(self.namespace))
                    with self.lock:
                        self.report["clear_accepted"] = accepted
                    shown = not accepted
                self.stop.wait(0.025)
        except Exception as error:
            # Visualization is optional; report its failure without failing physics.
            with self.lock:
                self.report.update(errors=self.report["errors"] + 1, error=str(error))
        finally:
            if node is not None and shown:
                try:
                    accepted = self._send(node, delete_markers(self.namespace))
                    with self.lock:
                        self.report["clear_accepted"] = accepted
                except Exception:
                    with self.lock:
                        self.report["errors"] += 1

    def close(self):
        self.hide()
        self.stop.set()
        self.worker.join(timeout=0.3)
