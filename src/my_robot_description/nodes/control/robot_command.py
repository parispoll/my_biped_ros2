#!/usr/bin/env python3
"""Send one high-level request and wait for its matching controller reply."""

import argparse
import json
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["step", "walk", "stop", "stand", "status"])
    parser.add_argument("--steps", type=int, default=1)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node("biped_command_client")
    identifier = uuid.uuid4().hex
    replies = []

    def receive(msg):
        reply = json.loads(msg.data)
        if reply.get("request_id") == identifier:
            replies.append(reply)

    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscription = node.create_subscription(
        String, "/biped/controller/status", receive, qos
    )
    publisher = node.create_publisher(String, "/biped/controller/command", 10)
    try:
        deadline = time.monotonic() + 8
        sent = False
        while not replies and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if not sent and publisher.get_subscription_count():
                publisher.publish(
                    String(
                        data=json.dumps(
                            dict(id=identifier, command=args.command, steps=args.steps)
                        )
                    )
                )
                sent = True
        if not replies:
            raise SystemExit(
                "No controller acknowledgement. Check launch terminal and ROS_DOMAIN_ID."
            )
        print(json.dumps(replies[0], indent=2))
        if not replies[0]["accepted"]:
            raise SystemExit(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
