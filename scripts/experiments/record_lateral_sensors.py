#!/usr/bin/env python3
"""Start read-only sensor capture before warm-up; never command robot joints."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import threading
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import Contacts
from gz.transport13 import Node as GazeboNode
from gz.msgs10.pose_v_pb2 import Pose_V
from my_robot_description.sensing.contacts import ground_contact_pairs
from my_robot_description.control.posture_effort import quaternion_tilt
from scripts.checks.check_imu import measurement


class Recorder(Node):
    def __init__(self, folder, capture_contacts=False, state_contacts=False):
        super().__init__(
            "lateral_sensor_recorder",
            parameter_overrides=[Parameter("use_sim_time", value=True)],
        )
        self.stream = (folder / "sensors.jsonl").open("x", buffering=1)
        self.imu_stream = (folder / "imu-raw.jsonl").open("x", buffering=1)
        self.contact_stream = (
            (folder / "contact-wrenches.jsonl").open("x", buffering=1)
            if capture_contacts
            else None
        )
        self.last_contact_recorded = {}
        self.lock = threading.Lock()
        self.pose = None
        self.latest_imu = None
        self.contacts = {}
        self.last = None
        self.transport = GazeboNode()
        self.transport.subscribe(Pose_V, "/world/standing/pose/info", self.on_pose)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, "/biped/imu", self.on_imu, qos)
        self.subs = [
            self.create_subscription(
                Contacts,
                (
                    f"/biped/state/{s}_foot/contacts"
                    if state_contacts
                    else f"/biped/{s}_foot/contacts"
                ),
                lambda m, s=s: self.on_contact(m, s),
                qos,
            )
            for s in ["left", "right"]
        ]
        self.create_timer(0.02, self.record)

    def on_pose(self, msg):
        p = next((p for p in msg.pose if p.name == "my_biped"), None)
        if p is None:
            return
        q = p.orientation
        pose = dict(
            sim_time=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9,
            position=[p.position.x, p.position.y, p.position.z],
            tilt_radians=quaternion_tilt(q.x, q.y, q.z, q.w),
        )
        with self.lock:
            self.pose = pose

    def on_imu(self, msg):
        self.latest_imu = measurement(msg)
        self.imu_stream.write(json.dumps(self.latest_imu) + "\n")

    def on_contact(self, msg, side):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.contacts[side] = dict(
            sim_time=stamp, ground=bool(ground_contact_pairs(msg, side))
        )
        if (
            self.contact_stream
            and stamp - self.last_contact_recorded.get(side, -1.0) >= 0.019
        ):
            from rosidl_runtime_py.convert import message_to_ordereddict

            self.contact_stream.write(
                json.dumps(
                    dict(side=side, sim_time=stamp, message=message_to_ordereddict(msg))
                )
                + "\n"
            )
            self.last_contact_recorded[side] = stamp

    def record(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.last is not None and now <= self.last:
            return
        with self.lock:
            pose = self.pose
        if (
            pose is None
            or self.latest_imu is None
            or set(self.contacts) != {"left", "right"}
        ):
            return
        self.stream.write(
            json.dumps(
                dict(
                    sim_time=now,
                    pose=pose,
                    imu=self.latest_imu,
                    contacts=self.contacts.copy(),
                )
            )
            + "\n"
        )
        self.last = now

    def close(self):
        self.transport.unsubscribe("/world/standing/pose/info")
        self.stream.close()
        self.imu_stream.close()
        if self.contact_stream:
            self.contact_stream.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--capture-contacts", action="store_true")
    parser.add_argument("--state-contacts", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = None
    try:
        node = Recorder(args.output, args.capture_contacts, args.state_contacts)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
