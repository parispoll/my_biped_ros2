#!/usr/bin/env python3
"""Read simulated COM, show optional markers and record geometric observations."""

from collections import deque
import json
from pathlib import Path
import threading
import time
import rclpy
import signal
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from gz.transport13 import Node as GazeboNode
from gz.msgs10.double_v_pb2 import Double_V
from my_robot_description.sensing.contacts import ground_contact_pairs
from my_robot_description.sensing.com_geometry import (
    decode_com_state,
    geometric_support,
    contact_at,
)
from my_robot_description.visualization.com_visual import ComVisualizer


class ComObserver(Node):
    def __init__(self):
        super().__init__("center_of_mass_visual")
        self.declare_parameter("recording", "")
        path = self.get_parameter("recording").value
        self.output = Path(path).open("x", buffering=1) if path else None
        self.pending = deque()
        self.capture_last = None
        self.lock = threading.Lock()
        self.state = None
        self.received = None
        self.contacts = {side: deque(maxlen=100) for side in ("left", "right")}
        self.trail = deque(maxlen=100)
        self.last = None
        self.accepted_states = 0
        self.bad_states = 0
        self.visual = ComVisualizer()
        self.transport = GazeboNode()
        self.transport.subscribe(Double_V, "/biped/com_state", self.on_state)
        self.subs = [
            self.create_subscription(
                Contacts,
                f"/biped/{side}_foot/contacts",
                lambda msg, side=side: self.on_contact(msg, side),
                qos_profile_sensor_data,
            )
            for side in ("left", "right")
        ]
        self.timer = self.create_timer(0.05, self.draw)
        self.get_logger().info(
            "[com:visual] Cyan: whole-body COM; yellow: ground projection/trail; white: soles; green: contact-qualified flat-sole hull. Geometry is not a dynamic stability verdict."
        )

    def on_state(self, message):
        try:
            state = decode_com_state(list(message.data))
        except ValueError:
            self.bad_states += 1
            return
        with self.lock:
            self.state = state
            self.received = time.monotonic()
            self.accepted_states += 1
            if self.output:
                self.pending.append((state, self.received))

    def on_contact(self, message, side):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        previous = self.contacts[side][-1] if self.contacts[side] else None
        if previous and 0 <= stamp - previous["time"] < 0.02:
            return
        try:
            ground = bool(ground_contact_pairs(message, side))
        except ValueError:
            ground = False
        self.contacts[side].append(dict(time=stamp, ground=ground))

    def flush_capture(self):
        # Drawing may block on marker transport. Preserve every observation
        # received by the independent physics callback, with its original stamp.
        with self.lock:
            pending = list(self.pending)
            self.pending.clear()
        for state, received in pending:
            if self.capture_last is not None and state["sim_time"] <= self.capture_last:
                raise ValueError("COM recording source stamps must increase")
            contacts = {
                side: contact_at(state["sim_time"], history)
                for side, history in self.contacts.items()
            }
            support = geometric_support(state, contacts)
            self.output.write(
                json.dumps(
                    dict(
                        **state,
                        contacts=contacts,
                        support=support,
                        capture_wall_time_monotonic=received,
                    )
                )
                + "\n"
            )
            self.capture_last = state["sim_time"]

    def draw(self):
        self.flush_capture()
        with self.lock:
            state = self.state
            received = self.received
        now = self.get_clock().now().nanoseconds / 1e9
        if (
            state is None
            or received is None
            or time.monotonic() - received > 1.0
            or not -0.02 <= now - state["sim_time"] <= 0.1
        ):
            self.visual.hide()
            return
        if self.last is not None and state["sim_time"] < self.last:
            self.trail.clear()
        if state["sim_time"] == self.last:
            return
        self.last = state["sim_time"]
        contacts = {
            side: contact_at(state["sim_time"], history)
            for side, history in self.contacts.items()
        }
        support = geometric_support(state, contacts)
        self.trail.append((state["com"][0], state["com"][1], state["sim_time"]))
        while self.trail and state["sim_time"] - self.trail[0][2] > 5.0:
            self.trail.popleft()
        self.visual.update(state, support, self.trail)

    def close(self):
        self.transport.unsubscribe("/biped/com_state")
        self.flush_capture()
        self.visual.close()
        if self.output:
            self.output.close()
        self.get_logger().info(
            f"[com:closed] valid_states={self.accepted_states}, rejected_states={self.bad_states}, marker_acceptances={self.visual.accepted}, marker_errors={self.visual.errors}, last_error={self.visual.last_error}"
        )


def main():
    node = None
    executor = None
    stopping = [False]
    previous = {}

    def request_stop(signum, frame):
        stopping[0] = True

    # Finish the current ROS callback before invalidating its context.
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, request_stop)
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = ComObserver()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while not stopping[0] and rclpy.ok():
            executor.spin_once(timeout_sec=0.02)
    finally:
        try:
            if node is not None:
                node.timer.cancel()
            if executor is not None:
                executor.shutdown()
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == "__main__":
    main()
