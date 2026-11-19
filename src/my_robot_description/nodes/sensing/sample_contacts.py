#!/usr/bin/env python3
"""Forward latest raw contacts at 50 simulated Hz, preserving source stamps."""

import rclpy
import signal
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts


class ContactSampler(Node):
    def __init__(self):
        super().__init__("balance_contact_sampler")
        self.latest = {}
        self.subscriptions_owned = []
        self.publishers_owned = {}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        for side in ("left", "right"):
            self.publishers_owned[side] = self.create_publisher(
                Contacts, f"/biped/balance/{side}_foot/contacts", qos
            )
            self.subscriptions_owned.append(
                self.create_subscription(
                    Contacts,
                    f"/biped/{side}_foot/contacts",
                    lambda msg, side=side: self.latest.update({side: msg}),
                    qos,
                )
            )
        self.timer = self.create_timer(0.02, self.forward)

    def forward(self):
        for side, msg in self.latest.items():
            self.publishers_owned[side].publish(msg)


def main():
    node = None
    executor = None
    stopping = [False]
    previous = {}

    def request_stop(signum, frame):
        stopping[0] = True

    # Keep the ROS context valid until an in-progress subscription callback
    # finishes. Default ROS signal handling can invalidate it during take_message.
    # A plain flag also avoids doing ROS cleanup inside a signal handler.
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, request_stop)
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = ContactSampler()
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
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == "__main__":
    main()
