#!/usr/bin/env python3
"""Read IMU and record hypothetical ankle corrections. Creates NO publishers."""

from dataclasses import asdict
import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from my_robot_description.sensing.balance import (
    BalanceConfig,
    balance_proposal,
    sagittal_imu,
)
from my_robot_description.control.imu_watchdog import check_imu_watchdog


class BalanceObserver(Node):
    def __init__(self):
        super().__init__("balance_observer")
        for name, value in [
            ("recording", ""),
            ("kp", 20.0),
            ("kd", 4.0),
            ("ankle_limit", 0.5),
            ("target_pitch", 0.0),
        ]:
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.config = BalanceConfig(
            **{k: value(k) for k in ("kp", "kd", "ankle_limit", "target_pitch")}
        ).validate()
        if not value("recording"):
            raise ValueError("Provide a balance recording path")
        output = Path(value("recording"))
        output.parent.mkdir(parents=True, exist_ok=True)
        self.recording = output.open("x", buffering=1)
        self.last_stamp = None
        self.started_wall = time.monotonic()
        self.last_wall = None
        self.sub = self.create_subscription(
            Imu, "/biped/imu", self.observe, qos_profile_sensor_data
        )
        self.timer = self.create_timer(
            0.1, self.supervise, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.get_logger().info(
            "[balance:observer] Recording proposals only; no command publishers."
        )

    def observe(self, msg):
        if (
            msg.header.frame_id != "base_link"
            or msg.orientation_covariance[0] == -1
            or msg.angular_velocity_covariance[0] == -1
        ):
            raise ValueError(
                "Balance observer needs base_link orientation and angular velocity"
            )
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if self.last_stamp is not None and stamp <= self.last_stamp:
            raise ValueError("IMU timestamps must increase")
        q, v = msg.orientation, msg.angular_velocity
        pitch, rate = sagittal_imu(q.x, q.y, q.z, q.w, v.x, v.y, v.z)
        terms = balance_proposal(pitch, rate, self.config)
        self.recording.write(
            json.dumps(
                dict(
                    sim_time=stamp,
                    receipt_sim_time=self.get_clock().now().nanoseconds / 1e9,
                    pitch_rad=pitch,
                    pitch_rate_rad_s=rate,
                    settings=asdict(self.config),
                    applied=False,
                    applied_balance_nm=0.0,
                    **terms,
                )
            )
            + "\n"
        )
        first_sample = self.last_wall is None
        self.last_stamp = stamp
        self.last_wall = time.monotonic()
        if first_sample:
            self.get_logger().info(
                f"[balance:imu-ready] First valid IMU sample after {self.last_wall-self.started_wall:.3f} wall seconds."
            )

    def supervise(self):
        check_imu_watchdog(self.started_wall, self.last_wall, time.monotonic())


def main():
    rclpy.init()
    node = None
    try:
        node = BalanceObserver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.recording.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
