#!/usr/bin/env python3
"""Hold one balanced crouch using bounded joint torques; with optional IMU ankle feedback."""

import json
from pathlib import Path
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController
from my_robot_description.model.stance import JOINT_NAMES, model_from_urdf
from my_robot_description.model.statics import standing_loads
from my_robot_description.control.posture_effort import (
    posture_command,
    effort_limits_from_urdf,
    quaternion_tilt,
)
from my_robot_description.control.integral import BoundedIntegral
from my_robot_description.sensing.balance import BalanceConfig, sagittal_imu
from my_robot_description.control.balance_control import active_correction
from my_robot_description.control.feedback_timing import (
    integration_timing,
    startup_timing_ready,
)
from my_robot_description.sensing.contacts import ground_contact_pairs
from ros_gz_interfaces.msg import Contacts
from rclpy.qos import QoSProfile, ReliabilityPolicy
from my_robot_description.model.torque import clamp_torque


class EffortStanding(Node):
    def __init__(self):
        super().__init__("standing_effort")
        for name, default in [
            ("robot_description", ""),
            ("knee_bend", 0.4),
            ("kp", [80.0, 100.0, 60.0]),
            ("kd", [0.10, 0.15, 0.05]),
            ("recording", ""),
            ("controller_mode", "pd"),
            ("visual_fast_ready", False),
            ("balance_enabled", False),
            ("balance_kp", 20.0),
            ("balance_kd", 4.0),
            ("balance_limit", 0.5),
            ("balance_target", 0.0),
            ("balance_ramp_seconds", 1.0),
            ("ki", [80.0, 100.0, 60.0]),
            ("integral_limits", [1.0, 1.0, 1.0]),
        ]:
            self.declare_parameter(name, default)
        value = lambda name: self.get_parameter(name).value
        xml = value("robot_description")
        model = model_from_urdf(xml)
        knee = value("knee_bend")
        self.target = list(model.pose(knee)) * 2
        loads = standing_loads(model, knee)["joint_torques_nm"]
        self.ff = [loads[name] for name in JOINT_NAMES]
        self.limits = effort_limits_from_urdf(xml)
        self.kp, self.kd = list(value("kp")) * 2, list(value("kd")) * 2
        self.visual_fast_ready = value("visual_fast_ready")
        self.balance_enabled = value("balance_enabled")
        self.balance_config = BalanceConfig(
            value("balance_kp"),
            value("balance_kd"),
            value("balance_limit"),
            value("balance_target"),
        ).validate()
        self.balance_ramp_seconds = value("balance_ramp_seconds")
        if self.balance_ramp_seconds <= 0:
            raise ValueError("Positive balance ramp required")
        self.balance_imu = None
        self.balance_contacts = {}
        self.ready_time = None
        self.balance_ready = False
        self.contact_subs = []
        if self.balance_enabled:
            contact_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            for side in ("left", "right"):
                self.contact_subs.append(
                    self.create_subscription(
                        Contacts,
                        f"/biped/balance/{side}_foot/contacts",
                        lambda msg, side=side: self.contact(msg, side),
                        contact_qos,
                    )
                )
        self.mode = value("controller_mode")
        if self.mode not in ("pd", "pid"):
            raise ValueError("Controller mode must be pd or pid")
        self.integrator = BoundedIntegral(
            list(value("ki")) * 2, list(value("integral_limits")) * 2
        )
        posture_command(
            self.target, self.target, [0.0] * 6, self.ff, self.kp, self.kd, self.limits
        )
        path = value("recording")
        self.recording = None
        if path:
            output = Path(path)
            output.parent.mkdir(parents=True, exist_ok=True)
            self.recording = output.open("x", buffering=1)
            setup = dict(
                schema=1,
                target_joint_positions_rad=self.target,
                motor_limits_nm=self.limits,
                knee_bend=knee,
                controller_mode=self.mode,
                balance_enabled=self.balance_enabled,
            )
            with output.with_suffix(".setup.json").open("x") as stream:
                json.dump(setup, stream, indent=2)
        self.pub = self.create_publisher(
            Float64MultiArray, "/standing_effort_controller/commands", 1
        )
        # Control needs the newest state after a scheduling delay, not a backlog
        # of five sensor samples. Genuine stale feedback still fails its guard.
        feedback_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            JointState, "/joint_states", self.feedback, feedback_qos
        )
        self.imu_sub = self.create_subscription(
            Imu, "/biped/imu", self.imu, feedback_qos
        )
        self.imu_wall = None
        self.tilt = None
        self.switch = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.parameters = AsyncParameterClient(self, "/controller_manager")
        self.parameter_future = None
        self.switch_future = None
        self.active = False
        self.ready = False
        self.last_wall = None
        self.started_wall = time.monotonic()
        self.active_time = None
        self.last_recorded = -1.0
        self.last_stamp = None
        self.last_terms = None
        self.settled_since = None
        self.clock_wait_logged = False
        self.clock_ready_logged = False
        self.startup_rejected_feedback_samples = 0
        self.startup_settle_restart = False
        self.timer = self.create_timer(
            0.02, self.supervise, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.get_logger().info(
            f"Target {self.target}; static feedforward {self.ff}; caps {self.limits} N m"
        )

    def contact(self, msg, side):
        self.balance_contacts[side] = (
            msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
            bool(ground_contact_pairs(msg, side)),
        )

    def imu(self, msg):
        if msg.header.frame_id != "base_link" or msg.orientation_covariance[0] == -1:
            raise RuntimeError(
                "Expected an available pelvis IMU orientation in base_link"
            )
        q = msg.orientation
        self.tilt = quaternion_tilt(q.x, q.y, q.z, q.w)
        if self.balance_enabled:
            if msg.angular_velocity_covariance[0] == -1:
                raise ValueError("Balance needs angular velocity")
            v = msg.angular_velocity
            pitch, rate = sagittal_imu(q.x, q.y, q.z, q.w, v.x, v.y, v.z)
            self.balance_imu = (
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                pitch,
                rate,
            )
        self.imu_wall = time.monotonic()
        if self.tilt > 0.15:
            raise RuntimeError(
                "Pelvis tilt exceeded 0.15 rad; stopping this stationary experiment"
            )

    def feedback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        previous_stamp = self.last_stamp
        if self.last_stamp is not None and stamp < self.last_stamp:
            raise RuntimeError(
                "Simulation clock moved backwards; restart the experiment"
            )
        timing = dict(
            feedback_gap_s=0.0,
            joint_source_age_s=0.0,
            integral_frozen_gap=False,
            integral_dt_s=0.0,
        )
        if self.active and previous_stamp is not None:
            timing = integration_timing(
                previous_stamp,
                stamp,
                self.get_clock().now().nanoseconds / 1e9,
                wait_for_startup=not self.ready,
            )
            if timing is None:
                self.startup_rejected_feedback_samples += 1
                self.startup_settle_restart = True
                self.settled_since = None
                if self.startup_rejected_feedback_samples == 1:
                    self.get_logger().warning(
                        "[effort:feedback-wait] Discarding unsynchronized startup feedback; no command/integral update. Settling will restart on fresh data."
                    )
                return
            if self.startup_settle_restart:
                self.active_time = stamp
                self.startup_settle_restart = False
                self.get_logger().info(
                    f"[effort:feedback-resumed] Fresh feedback restored after {self.startup_rejected_feedback_samples} rejected startup samples; restarting full settling window."
                )
        self.last_stamp = stamp
        indices = [msg.name.index(name) for name in JOINT_NAMES]
        position = [msg.position[i] for i in indices]
        velocity = [msg.velocity[i] for i in indices]
        terms = posture_command(
            self.target, position, velocity, self.ff, self.kp, self.kd, self.limits
        )
        terms["pd_requested"] = terms["requested"].copy()
        balance = dict(balance_nm=[0.0] * 6, balance_gate=False, balance_ramp=0.0)
        if self.balance_enabled:
            from dataclasses import asdict

            stamp_imu, pitch, rate = (
                self.balance_imu if self.balance_imu else (stamp - 1.0, 0.0, 0.0)
            )
            contacts = [
                dict(
                    age=stamp
                    - self.balance_contacts.get(side, (stamp - 1.0, False))[0],
                    ground=self.balance_contacts.get(side, (0.0, False))[1],
                )
                for side in ("left", "right")
            ]
            elapsed = stamp - self.ready_time if self.ready_time is not None else None
            balance = active_correction(
                pitch,
                rate,
                stamp - stamp_imu,
                contacts,
                elapsed,
                self.balance_config,
                self.balance_ramp_seconds,
            )
            terms.update(
                balance_pitch_rad=pitch,
                balance_rate_rad_s=rate,
                balance_imu_age=stamp - stamp_imu,
                balance_contacts=contacts,
                balance_elapsed=elapsed,
                balance_settings=asdict(self.balance_config),
                balance_ramp_seconds=self.balance_ramp_seconds,
            )
        terms.update(balance_enabled=self.balance_enabled, **balance)
        # Include balance in the integral's saturation prediction (combined anti-windup).
        base_with_balance = [
            p + b for p, b in zip(terms["requested"], balance["balance_nm"])
        ]
        integral = self.integrator.state.copy()
        if timing["integral_frozen_gap"]:
            self.get_logger().warning(
                f'Feedback gap {timing["feedback_gap_s"]:.3f} s; latest sample fresh, integral frozen for this update.'
            )
            self.settled_since = None
        terms.update(
            timing,
            startup_rejected_feedback_samples=self.startup_rejected_feedback_samples,
        )
        if self.mode == "pid" and self.active and previous_stamp is not None:
            integral = self.integrator.update(
                terms["error"], timing["integral_dt_s"], base_with_balance, self.limits
            )
        terms["integral"] = integral
        terms["requested"] = [p + i for p, i in zip(base_with_balance, integral)]
        terms["command"] = [
            clamp_torque(t, limit) for t, limit in zip(terms["requested"], self.limits)
        ]
        self.last_wall = time.monotonic()
        self.last_terms = terms
        self.pub.publish(Float64MultiArray(data=terms["command"]))
        if self.active:
            if (
                max(abs(e) for e in terms["error"]) > 0.2
                or max(abs(v) for v in velocity) > 3
            ):
                raise RuntimeError(
                    "Posture exceeded this stationary experiment’s angle/speed bounds"
                )
            if self.mode == "pid":
                settled = max(abs(e) for e in terms["error"]) < (
                    0.03 if self.visual_fast_ready else 1e-4
                ) and max(abs(v) for v in velocity) < (
                    0.05 if self.visual_fast_ready else 0.001
                )
                if not settled:
                    self.settled_since = None
                elif self.settled_since is None:
                    self.settled_since = stamp
            stable_pid = self.mode == "pd" or (
                self.settled_since is not None
                and stamp - self.settled_since >= (0.5 if self.visual_fast_ready else 2)
            )
            minimum_settle = (
                0.5 if self.visual_fast_ready else (2 if self.mode == "pd" else 15)
            )
            if (
                not self.ready
                and stamp - self.active_time >= minimum_settle
                and stable_pid
            ):
                if (
                    max(abs(e) for e in terms["error"]) > 0.03
                    or max(abs(v) for v in velocity) > 0.05
                ):
                    raise RuntimeError(
                        "Effort posture did not settle within 0.03 rad and 0.05 rad/s"
                    )
                self.ready = True
                self.ready_time = stamp
                self.get_logger().info(
                    "[effort:ready] Bounded effort posture active and settled; check body pose separately."
                )
        if (
            self.balance_enabled
            and self.ready_time is not None
            and not self.balance_ready
            and stamp - self.ready_time
            >= (self.balance_ramp_seconds if self.visual_fast_ready else 3)
            and balance["balance_gate"]
        ):
            self.balance_ready = True
            self.get_logger().info(
                "[balance:ready] IMU correction ramp complete; recording may begin."
            )
        if self.recording and stamp - self.last_recorded >= 0.009:
            self.recording.write(
                json.dumps(
                    dict(
                        sim_time=stamp,
                        active=self.active,
                        pelvis_tilt_rad=self.tilt,
                        position=position,
                        velocity=velocity,
                        target=self.target,
                        **terms,
                    )
                )
                + "\n"
            )
            self.last_recorded = stamp

    def supervise(self):
        now = time.monotonic()
        if not self.active and now - self.started_wall > 30:
            raise TimeoutError("Effort startup did not complete in 30 wall seconds")
        if self.last_wall is not None and now - self.last_wall > 1:
            raise RuntimeError(
                "Joint feedback absent for one wall second; shutting down experiment"
            )
        if self.imu_wall is not None and now - self.imu_wall > 1:
            raise RuntimeError(
                "IMU feedback absent for one wall second; shutting down experiment"
            )
        if (
            self.last_terms is None
            or self.imu_wall is None
            or self.pub.get_subscription_count() == 0
        ):
            return
        if not self.active:
            ros_time = self.get_clock().now().nanoseconds / 1e9
            if not startup_timing_ready(self.last_stamp, ros_time):
                if not self.clock_wait_logged:
                    self.get_logger().info(
                        f"[effort:clock-wait] Waiting for initialized simulation time and fresh joints: clock={ros_time:.6f}, joint={self.last_stamp:.6f}"
                    )
                    self.clock_wait_logged = True
                return
            if not self.clock_ready_logged:
                self.get_logger().info(
                    f"[effort:clock-ready] Simulation time synchronized before activation: clock={ros_time:.6f}, joint={self.last_stamp:.6f}"
                )
                self.clock_ready_logged = True
        if self.parameter_future is None:
            if self.parameters.services_are_ready():
                self.parameter_future = self.parameters.get_parameters(
                    ["enforce_command_limits"]
                )
            return
        if not self.parameter_future.done():
            return
        response = self.parameter_future.result()
        if (
            not response
            or len(response.values) != 1
            or not response.values[0].bool_value
        ):
            raise RuntimeError("controller_manager must enforce command limits")
        if self.switch_future is None and self.switch.service_is_ready():
            request = SwitchController.Request()
            request.activate_controllers = ["standing_effort_controller"]
            request.strictness = SwitchController.Request.STRICT
            request.activate_asap = True
            request.timeout.sec = 5
            self.switch_future = self.switch.call_async(request)
        elif (
            self.switch_future is not None
            and self.switch_future.done()
            and not self.active
        ):
            if not self.switch_future.result().ok:
                raise RuntimeError("Effort controller activation failed")
            self.active = True
            self.active_time = self.last_stamp
            self.get_logger().info(
                "[effort:activated] Position holding has handed over to effort commands."
            )

    def close(self):
        if rclpy.ok():
            self.pub.publish(Float64MultiArray(data=[0.0] * 6))
        if self.recording:
            self.recording.close()


def main():
    rclpy.init()
    node = None
    try:
        node = EffortStanding()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
