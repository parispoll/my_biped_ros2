#!/usr/bin/env python3
"""Check fresh ground-contact evidence from both feet in a settled crouch."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts
from my_robot_description.sensing.contacts import ground_contact_pairs


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec / 1e9


def collect(args):
    node = Node("check_foot_contacts")
    clock = [None]
    origin = [None]
    window_start = [None]
    errors = []
    stats = {
        side: {
            "samples": 0,
            "ground_samples": 0,
            "last_stamp": None,
            "first_stamp": None,
            "max_gap": 0.0,
            "collision_pairs": set(),
        }
        for side in ("left", "right")
    }

    def on_clock(message):
        current = stamp_seconds(message.clock)
        if clock[0] is not None and current < clock[0]:
            errors.append(ValueError("Simulation time moved backwards"))
        clock[0] = current
        if origin[0] is None:
            origin[0] = current
        if window_start[0] is None and current - origin[0] >= args.settle_time:
            window_start[0] = current

    def on_contact(message, side):
        try:
            stamp = stamp_seconds(message.header.stamp)
            if window_start[0] is None or stamp < window_start[0]:
                return
            if not math.isfinite(stamp):
                raise ValueError("Invalid contact timestamp")
            state = stats[side]
            if state["last_stamp"] is not None:
                if stamp <= state["last_stamp"]:
                    raise ValueError(f"{side}: contact timestamps did not advance")
                state["max_gap"] = max(state["max_gap"], stamp - state["last_stamp"])
            pairs = ground_contact_pairs(message, side)
            state["samples"] += 1
            state["ground_samples"] += bool(pairs)
            state["collision_pairs"].update(pairs)
            state["first_stamp"] = (
                stamp if state["first_stamp"] is None else state["first_stamp"]
            )
            state["last_stamp"] = stamp
        except Exception as error:
            errors.append(error)

    node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
    for side in stats:
        node.create_subscription(
            Contacts,
            f"/biped/{side}_foot/contacts",
            lambda message, side=side: on_contact(message, side),
            qos_profile_sensor_data,
        )
    deadline = time.monotonic() + args.wall_timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if errors:
                raise errors[0]
            if window_start[0] is None or clock[0] - window_start[0] < args.duration:
                continue
            report = {
                "passed": True,
                "simulated_duration": clock[0] - window_start[0],
                "meaning": "Fresh collision evidence for both feet touching ground_plane; no force validation",
                "feet": {},
            }
            for side, state in stats.items():
                if not state["samples"]:
                    raise ValueError(
                        f"{side}: no fresh contact data; contact status is unverified"
                    )
                fraction = state["ground_samples"] / state["samples"]
                age = max(0, clock[0] - state["last_stamp"])
                start_gap = max(0, state["first_stamp"] - window_start[0])
                if (
                    fraction < 0.99
                    or max(state["max_gap"], age, start_gap) > args.max_gap
                ):
                    raise ValueError(
                        f'{side}: insufficient continuous ground-contact evidence; fraction={fraction}, age={age}, max gap={state["max_gap"]}'
                    )
                elapsed = state["last_stamp"] - state["first_stamp"]
                report["feet"][side] = {
                    "ground_contact_confirmed": True,
                    "samples": state["samples"],
                    "ground_sample_fraction": fraction,
                    "max_received_stamp_gap_s": state["max_gap"],
                    "last_sample_age_s": age,
                    "received_rate_hz": (
                        (state["samples"] - 1) / elapsed if elapsed else None
                    ),
                    "collision_pairs": sorted(state["collision_pairs"]),
                }
            return report
        raise TimeoutError(
            "Contact check timed out: inspect /clock, both contact topics, bridges and ROS_DOMAIN_ID"
        )
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5)
    parser.add_argument("--settle-time", type=float, default=2)
    parser.add_argument("--wall-timeout", type=float, default=20)
    parser.add_argument("--max-gap", type=float, default=0.1)
    args = parser.parse_args()
    if (
        any(not math.isfinite(value) for value in vars(args).values())
        or min(args.duration, args.wall_timeout, args.max_gap) <= 0
        or args.settle_time < 0
    ):
        parser.error("Use finite positive durations/gap; settling time may be zero")
    rclpy.init()
    try:
        print(json.dumps(collect(args), indent=2))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
