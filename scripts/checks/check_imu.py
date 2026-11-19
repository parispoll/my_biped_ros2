#!/usr/bin/env python3
"""Verify an ideal, stationary pelvis IMU on /biped/imu; use matching ROS_DOMAIN_ID."""

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
from sensor_msgs.msg import Imu


def measurement(message):
    """Decode orientation and validate frame/availability before using measurements."""
    if message.header.frame_id != "base_link":
        raise ValueError(f"Expected base_link frame, got {message.header.frame_id!r}")
    if any(
        covariance[0] == -1
        for covariance in (
            message.orientation_covariance,
            message.angular_velocity_covariance,
            message.linear_acceleration_covariance,
        )
    ):
        raise ValueError("IMU marks a required measurement as unavailable")
    q = message.orientation
    quaternion = (q.x, q.y, q.z, q.w)
    omega = (
        message.angular_velocity.x,
        message.angular_velocity.y,
        message.angular_velocity.z,
    )
    acceleration = (
        message.linear_acceleration.x,
        message.linear_acceleration.y,
        message.linear_acceleration.z,
    )
    if not all(math.isfinite(value) for value in (*quaternion, *omega, *acceleration)):
        raise ValueError("Non-finite IMU measurement")
    norm = math.sqrt(sum(value * value for value in quaternion))
    if abs(norm - 1) > 0.001:
        raise ValueError(f"IMU quaternion is not normalized: norm={norm}")
    x, y, z, w = (value / norm for value in quaternion)
    return {
        "sim_time": message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
        "pitch_radians": math.asin(max(-1, min(1, 2 * (w * y - z * x)))),
        "tilt_radians": math.acos(max(-1, min(1, 1 - 2 * (x * x + y * y)))),
        "angular_speed": math.sqrt(sum(value * value for value in omega)),
        "acceleration": acceleration,
    }


def check_stationary(sample):
    """These thresholds apply to a settled, upright, noise-free simulation."""
    if sample["tilt_radians"] > 0.05 or sample["angular_speed"] > 0.05:
        raise ValueError(f"IMU is not near upright and stationary: {sample}")
    ax, ay, az = sample["acceleration"]
    if max(abs(ax), abs(ay), abs(az - 9.81)) > 0.25:
        raise ValueError(f"Expected specific force near (0,0,+9.81) m/s^2: {sample}")


def collect(args):
    node = Node("check_pelvis_imu")
    samples = []
    first = None
    previous = None
    error = []

    def callback(message):
        nonlocal first, previous
        try:
            sample = measurement(message)
            stamp = sample["sim_time"]
            if previous is not None and stamp <= previous:
                raise ValueError("IMU timestamps did not strictly increase")
            previous = stamp
            first = stamp if first is None else first
            if stamp - first < args.settle_time:
                return
            check_stationary(sample)
            samples.append(sample)
        except Exception as exc:
            error.append(exc)

    node.create_subscription(Imu, "/biped/imu", callback, qos_profile_sensor_data)
    deadline = time.monotonic() + args.wall_timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if error:
                raise error[0]
            if (
                len(samples) > 1
                and samples[-1]["sim_time"] - samples[0]["sim_time"] >= args.duration
            ):
                elapsed = samples[-1]["sim_time"] - samples[0]["sim_time"]
                rate = (len(samples) - 1) / elapsed
                if not 90 <= rate <= 110:
                    raise ValueError(
                        f"Expected approximately 100 Hz; received {rate:.2f} Hz in simulation time"
                    )
                return {
                    "passed": True,
                    "topic": "/biped/imu",
                    "frame_id": "base_link",
                    "samples": len(samples),
                    "simulated_duration": elapsed,
                    "received_rate_hz": rate,
                    "max_tilt_radians": max(s["tilt_radians"] for s in samples),
                    "max_abs_pitch_radians": max(
                        abs(s["pitch_radians"]) for s in samples
                    ),
                    "max_angular_speed_rad_s": max(s["angular_speed"] for s in samples),
                    "mean_acceleration_m_s2": [
                        sum(s["acceleration"][axis] for s in samples) / len(samples)
                        for axis in range(3)
                    ],
                }
        raise TimeoutError(
            "IMU check timed out: inspect /biped/imu, ROS_DOMAIN_ID, simulator play state and IMU bridge"
        )
    finally:
        node.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=5)
    parser.add_argument("--settle-time", type=float, default=2)
    parser.add_argument("--wall-timeout", type=float, default=30)
    args = parser.parse_args()
    if (
        any(not math.isfinite(value) for value in vars(args).values())
        or args.duration <= 0
        or args.settle_time < 0
        or args.wall_timeout <= 0
    ):
        parser.error("Use finite positive durations; settle time may be zero")
    rclpy.init()
    try:
        print(json.dumps(collect(args), indent=2))
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
