"""Optional GUI mailbox: marker-service delays never block the force loop."""

import threading
import time
import uuid
from gz.transport13 import Node
from gz.msgs10.marker_v_pb2 import Marker_V
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.boolean_pb2 import Boolean
from my_robot_description.visualization.com_markers import make_markers, delete_markers


class ComVisualizer:
    def __init__(self):
        self.namespace = "biped_com_" + uuid.uuid4().hex
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.target = None
        self.accepted = 0
        self.errors = 0
        self.last_error = None
        self.worker = threading.Thread(target=self.run, daemon=True)
        self.worker.start()

    def update(self, state, support, trail):
        with self.lock:
            self.target = (state, support, list(trail), time.monotonic())

    def hide(self):
        with self.lock:
            self.target = None

    def run(self):
        node = Node()
        shown = False
        active_ids = set()
        try:
            while not self.stop.is_set():
                if "/marker_array" not in node.service_list():
                    self.stop.wait(0.25)
                    continue
                with self.lock:
                    target = self.target
                if target is not None and time.monotonic() - target[3] <= 1.0:
                    message = make_markers(*target[:3], self.namespace)
                    shown = True
                elif shown:
                    message = delete_markers(self.namespace)
                    shown = False
                else:
                    self.stop.wait(0.025)
                    continue
                filtered = Marker_V()
                for marker in message.marker:
                    if (
                        marker.action == Marker.DELETE_MARKER
                        and marker.id not in active_ids
                    ):
                        continue
                    filtered.marker.add().CopyFrom(marker)
                    if marker.action == Marker.ADD_MODIFY:
                        active_ids.add(marker.id)
                    elif marker.action == Marker.DELETE_MARKER:
                        active_ids.discard(marker.id)
                if not filtered.marker:
                    self.stop.wait(0.025)
                    continue
                accepted, response = node.request(
                    "/marker_array", filtered, Marker_V, Boolean, 50
                )
                self.accepted += int(accepted and response.data)
                self.stop.wait(0.025)
        except Exception as error:
            self.errors += 1
            self.last_error = str(error)
        finally:
            if shown:
                try:
                    cleanup = delete_markers(self.namespace)
                    for i in reversed(range(len(cleanup.marker))):
                        if cleanup.marker[i].id not in active_ids:
                            del cleanup.marker[i]
                    if cleanup.marker:
                        node.request("/marker_array", cleanup, Marker_V, Boolean, 50)
                except Exception as error:
                    self.errors += 1
                    self.last_error = str(error)

    def close(self):
        self.stop.set()
        self.worker.join(timeout=0.3)
