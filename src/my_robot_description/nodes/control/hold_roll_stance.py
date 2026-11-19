#!/usr/bin/env python3
"""ROS adapter for ten-joint standing, transfers, foot lifts and walking.

The motion program supplies targets and feedforward; RollPosture computes joint
PID torques. Sensor callbacks and guards decide whether motion may continue.
IMU is used for safety here, not a new whole-body balance controller."""

import json
from pathlib import Path
import time
import rclpy
import signal
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController
from my_robot_description.model.roll_stance import (
    standing_plan,
    JOINT_NAMES,
    MOTOR_CAPS,
)
from my_robot_description.model.roll_actuators import ACTIVE, MOTOR_SPEEDS
from my_robot_description.control.roll_posture import (
    RollPosture,
    LATERAL_KI,
    KP,
    KD,
    INTEGRAL_CAPS,
)
from my_robot_description.control.roll_timing import imu_source_age
from my_robot_description.control.posture_effort import quaternion_tilt
from my_robot_description.control.feedback_timing import (
    integration_timing,
    startup_timing_ready,
)


class RollStanding(Node):
    def __init__(self):
        super().__init__("roll_standing")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("recording", "")
        self.declare_parameter("fault_report", "")
        self.declare_parameter("shift_enabled", False)
        self.declare_parameter("shift_displacement", 0.01)
        self.declare_parameter("load_feedback_enabled", False)
        self.declare_parameter("load_reference_bias", 0.0)
        self.load_control = None
        self.last_load_update = None
        self.declare_parameter("unload_enabled", False)
        self.declare_parameter("unload_fraction", 0.35)
        self.declare_parameter("unload_direction", "left")
        self.declare_parameter("lowering_enabled", False)
        self.declare_parameter("lowering_distance", 0.01)
        self.declare_parameter("lowering_lateral_shift", 0.0)
        self.declare_parameter("lowered_unload_fraction", 0.0)
        self.declare_parameter("alternating_walk_enabled", False)
        self.declare_parameter("walk_steps", 3)
        self.declare_parameter("walk_step_length", 0.05)
        self.declare_parameter("walk_time_scale", 1.0)
        self.declare_parameter("walk_transfer_time_scale", 0.0)
        self.declare_parameter("walk_velocity_feedforward", False)
        self.declare_parameter("interactive_enabled", False)
        self.interactive = None
        self.walking = self.get_parameter("alternating_walk_enabled").value
        self.declare_parameter("first_step_enabled", False)
        self.first_step = self.get_parameter("first_step_enabled").value
        self.declare_parameter("foot_lift_clearance", 0.002)
        self.declare_parameter("foot_lift_enabled", False)
        self.foot_lift = (
            self.get_parameter("foot_lift_enabled").value
            or self.first_step
            or self.walking
        )
        self.lift_observation = None
        self.lift_transport = None
        if self.foot_lift:
            from gz.transport13 import Node as GazeboNode
            from gz.msgs10.double_v_pb2 import Double_V

            self.lift_transport = GazeboNode()
            self.lift_transport.subscribe(
                Double_V, "/biped/com_state", self.lift_geometry
            )
        self.declare_parameter("zero_load_enabled", False)
        self.zero_load = self.get_parameter("zero_load_enabled").value or self.foot_lift
        self.zero_contact_rows = {}
        self.lowering = self.get_parameter("lowering_enabled").value
        if self.zero_load and (
            not self.lowering or self.get_parameter("lowering_distance").value != 0.04
        ):
            raise ValueError(
                "Zero-load mode requires the separate 40 mm lowering program"
            )
        self.unloading = self.get_parameter("unload_enabled").value
        self.loads = {}
        load_enabled = self.get_parameter("load_feedback_enabled").value
        load_bias = self.get_parameter("load_reference_bias").value
        if (load_enabled or load_bias) and not self.unloading:
            raise ValueError("Load feedback/test bias requires partial unloading")
        self.plan = standing_plan(self.get_parameter("robot_description").value)
        self.control = RollPosture(self.plan)
        self.shift = None
        self.shift_origin = None
        self.last_phase = None
        self.contacts = {}
        self.contact_subs = []
        from my_robot_description.runtime.contact_routing import feedback_topic

        if (
            sum(
                [
                    self.unloading,
                    self.lowering,
                    self.get_parameter("shift_enabled").value,
                ]
            )
            > 1
        ):
            raise ValueError("Select one motion program")
        if self.get_parameter("shift_enabled").value or self.unloading or self.lowering:
            from my_robot_description.motion.lateral_shift import ShiftProgram
            from ros_gz_interfaces.msg import Contacts
            from rclpy.qos import QoSProfile, ReliabilityPolicy

            if self.unloading or self.lowering:
                from my_robot_description.motion.partial_unload import (
                    PartialUnloadProgram,
                )
                from rosidl_runtime_py.convert import message_to_ordereddict

                self.contact_to_dict = message_to_ordereddict
                if self.lowering:
                    from my_robot_description.motion.bilateral_lowering import (
                        LoweringProgram,
                    )

                    lateral = self.get_parameter("lowering_lateral_shift").value
                    fraction = self.get_parameter("lowered_unload_fraction").value
                    if self.zero_load:
                        from my_robot_description.motion.zero_load import (
                            ZeroLoadProgram,
                        )

                        if self.foot_lift:
                            from my_robot_description.motion.live_foot_lift import (
                                LiveFootLiftProgram,
                            )

                            if self.walking:
                                from my_robot_description.motion.alternating_walk import (
                                    AlternatingWalkProgram,
                                )

                                from my_robot_description.motion.profiled_walk import (
                                    ProfiledWalkProgram,
                                )
                                from my_robot_description.motion.walking_profile import (
                                    WalkingProfile,
                                )

                                self.shift = ProfiledWalkProgram(
                                    self.get_parameter("robot_description").value,
                                    self.get_parameter("unload_direction").value,
                                    WalkingProfile(
                                        self.get_parameter("walk_steps").value,
                                        self.get_parameter("walk_step_length").value,
                                        self.get_parameter("walk_time_scale").value,
                                        self.get_parameter(
                                            "walk_transfer_time_scale"
                                        ).value,
                                        self.get_parameter(
                                            "walk_velocity_feedforward"
                                        ).value,
                                    ),
                                )
                            elif self.first_step:
                                from my_robot_description.motion.first_step import (
                                    FirstStepProgram,
                                )

                                self.shift = FirstStepProgram(
                                    self.get_parameter("robot_description").value,
                                    self.get_parameter("unload_direction").value,
                                )
                            else:
                                self.shift = LiveFootLiftProgram(
                                    self.get_parameter("robot_description").value,
                                    self.get_parameter("unload_direction").value,
                                    self.get_parameter("foot_lift_clearance").value,
                                )
                        else:
                            self.shift = ZeroLoadProgram(
                                self.get_parameter("robot_description").value,
                                self.get_parameter("unload_direction").value,
                            )
                    elif fraction:
                        from my_robot_description.motion.lowered_unload import (
                            LoweredUnloadProgram,
                        )

                        self.shift = LoweredUnloadProgram(
                            self.get_parameter("robot_description").value,
                            fraction,
                            self.get_parameter("unload_direction").value,
                        )
                    elif lateral:
                        from my_robot_description.motion.lowered_shift import (
                            LoweredShiftProgram,
                        )

                        if self.get_parameter("lowering_distance").value != 0.04:
                            raise ValueError("Lowered transfer requires 40 mm lowering")
                        self.shift = LoweredShiftProgram(
                            self.get_parameter("robot_description").value, lateral
                        )
                    else:
                        self.shift = LoweringProgram(
                            self.get_parameter("robot_description").value,
                            self.get_parameter("lowering_distance").value,
                        )
                elif load_enabled or load_bias:
                    from my_robot_description.motion.load_feedback import (
                        LoadTransferProgram,
                        LoadShareFeedback,
                    )

                    self.shift = LoadTransferProgram(
                        self.get_parameter("robot_description").value,
                        self.get_parameter("unload_fraction").value,
                        self.get_parameter("unload_direction").value,
                        load_enabled,
                        load_bias,
                    )
                    self.load_control = LoadShareFeedback(load_enabled)
                    self.load_control_state = None
                else:
                    self.shift = PartialUnloadProgram(
                        self.get_parameter("robot_description").value,
                        self.get_parameter("unload_fraction").value,
                        self.get_parameter("unload_direction").value,
                    )
            else:
                self.shift = ShiftProgram(
                    self.get_parameter("robot_description").value,
                    self.get_parameter("shift_displacement").value,
                )
            self.control = RollPosture(self.plan, integral_gains=LATERAL_KI)
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            for side in ["left", "right"]:
                self.contact_subs.append(
                    self.create_subscription(
                        Contacts,
                        feedback_topic(
                            side,
                            self.zero_load,
                            self.get_parameter("interactive_enabled").value,
                        ),
                        lambda m, s=side: self.contact(m, s),
                        qos,
                    )
                )
        self.pub = self.create_publisher(
            Float64MultiArray, "/roll_effort_controller/commands", 1
        )
        from rclpy.qos import QoSProfile, ReliabilityPolicy

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/joint_states", self.feedback, qos)
        self.create_subscription(Imu, "/biped/imu", self.imu, qos)
        self.parameters = AsyncParameterClient(self, "/controller_manager")
        self.switch = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        self.parameter_future = None
        self.switch_future = None
        self.active = False
        self.ready = False
        self.active_time = None
        self.stable_since = None
        self.imu_startup_waits = 0
        self.last_stamp = None
        self.last_wall = None
        self.imu_stamp = None
        self.imu_wall = None
        self.tilt = None
        self.start_wall = time.monotonic()
        self.last_recorded = -1.0
        self.stream = None
        path = self.get_parameter("recording").value
        if path:
            path = Path(path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.stream = path.open("x", buffering=1)
            path.with_suffix(".setup.json").write_text(
                json.dumps(self.plan, indent=2) + "\n"
            )
            if self.shift:
                path.with_suffix(".shift.json").write_text(
                    json.dumps(self.shift.setup, indent=2) + "\n"
                )
            path.with_suffix(".actuators.json").write_text(
                json.dumps(ACTIVE, indent=2) + "\n"
            )
            path.with_suffix(".gains.json").write_text(
                json.dumps(
                    dict(
                        kp=KP, kd=KD, ki=self.control.ki, integral_caps_nm=INTEGRAL_CAPS
                    ),
                    indent=2,
                )
                + "\n"
            )
        self.supervision_timer = self.create_timer(
            0.02, self.supervise, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        if self.get_parameter("interactive_enabled").value:
            if not self.walking:
                raise ValueError("Interactive mode requires the walking planner")
            from my_robot_description.runtime.command_session import CommandSession
            from my_robot_description.runtime.command_interface import CommandInterface

            self.interactive = CommandSession(self.shift)
            self.command_interface = CommandInterface(self, self.interactive)
        self.get_logger().info("[roll:plan] " + json.dumps(self.plan))

    def lift_geometry(self, message):
        from my_robot_description.sensing.com_geometry import decode_com_state

        try:
            self.lift_observation = decode_com_state(list(message.data))
        except ValueError:
            self.lift_observation = None

    def contact(self, msg, side):
        from my_robot_description.sensing.contacts import ground_contact_pairs

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.contacts[side] = (stamp, bool(ground_contact_pairs(msg, side)))
        if self.zero_load:
            self.zero_contact_rows[side] = dict(
                side=side, sim_time=stamp, message=self.contact_to_dict(msg)
            )
            return
        if self.unloading or self.lowering:
            from my_robot_description.sensing.foot_loads import decode_foot_load

            try:
                self.loads[side] = decode_foot_load(
                    dict(side=side, sim_time=stamp, message=self.contact_to_dict(msg))
                )
            except (ValueError, KeyError, TypeError) as error:
                self.loads.pop(side, None)
                if self.ready:
                    raise RuntimeError(f"Invalid live foot load: {error}")

    def contact_state(self):
        now = self.get_clock().now().nanoseconds / 1e9
        return {
            s: dict(
                age_s=now - self.contacts.get(s, (now - 1.0, False))[0],
                ground=self.contacts.get(s, (0.0, False))[1],
            )
            for s in ["left", "right"]
        }

    def imu(self, msg):
        if msg.header.frame_id != "base_link" or msg.orientation_covariance[0] == -1:
            raise RuntimeError("Valid base_link IMU required")
        q = msg.orientation
        self.tilt = quaternion_tilt(q.x, q.y, q.z, q.w)
        self.imu_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.imu_wall = time.monotonic()
        if self.tilt > 0.15:
            raise RuntimeError("Roll standing pelvis tilt exceeded 0.15 rad")

    def feedback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        previous = self.last_stamp
        dt = 0.0
        timing = {}
        if previous is not None and stamp < previous:
            raise RuntimeError("Joint clock moved backwards")
        if self.active and previous is not None:
            timing = integration_timing(
                previous,
                stamp,
                self.get_clock().now().nanoseconds / 1e9,
                wait_for_startup=not self.ready,
            )
            if timing is None:
                self.stable_since = None
                self.active_time = stamp
                return
            dt = timing["integral_dt_s"]
            if timing["integral_frozen_gap"]:
                self.stable_since = None
        if len(set(msg.name)) != len(msg.name):
            raise RuntimeError("Duplicate joint names")
        indices = [msg.name.index(n) for n in JOINT_NAMES]
        q = [msg.position[i] for i in indices]
        v = [msg.velocity[i] for i in indices]
        if self.active:
            if self.imu_stamp is None:
                raise RuntimeError("Missing pelvis IMU")
            age = imu_source_age(
                self.get_clock().now().nanoseconds / 1e9,
                self.imu_stamp,
                wait_for_startup=not self.ready,
            )
            if age is None:
                self.stable_since = None
                self.active_time = stamp
                self.last_stamp = stamp
                self.last_wall = time.monotonic()
                self.imu_startup_waits += 1
                if self.imu_startup_waits == 1:
                    self.get_logger().warning(
                        "[roll:imu-wait] Skipping unsynchronized startup IMU update; restarting full settling. Active measurement freshness bounds unchanged."
                    )
                return
        if self.interactive is not None:
            # Keep the existing planner/guards on one command-driven clock. Sensor
            # freshness still uses actual timestamps, including during a hold.
            elapsed = self.interactive.update(stamp, self.ready)
            if self.ready:
                self.shift_origin = stamp - elapsed
        contact_state = self.contact_state() if self.shift else {}
        contacts_ok = not self.shift or all(
            c["ground"] and -0.02 <= c["age_s"] <= 0.05 for c in contact_state.values()
        )
        load_state = None
        lift_state = None
        if self.unloading or self.lowering:
            from my_robot_description.motion.partial_unload import load_guard

            if self.zero_load:
                from my_robot_description.sensing.contact_state import (
                    contact_state as classify_contact,
                )
                from my_robot_description.motion.zero_load import zero_load_guard

                now = self.get_clock().now().nanoseconds / 1e9
                states = {
                    s: classify_contact(self.zero_contact_rows.get(s), now, s)
                    for s in ["left", "right"]
                }
                elapsed = (
                    0.0 if self.shift_origin is None else stamp - self.shift_origin
                )
                load_state = zero_load_guard(
                    states,
                    now,
                    self.plan["mass_kg"],
                    self.shift.expected_fraction(elapsed),
                    self.shift.sample(elapsed).get(
                        "supporting_foot", self.shift.setup["supporting_foot"]
                    ),
                )
                if self.foot_lift:
                    lift_state = self.shift.guard(
                        states, now, elapsed, self.lift_observation
                    )
                    if self.ready and not lift_state["eligible"]:
                        raise RuntimeError(f"Foot-lift guard: {lift_state}")
                    load_state = lift_state
                contacts_ok = load_state["eligible"]
            else:
                load_state = load_guard(
                    self.loads,
                    self.get_clock().now().nanoseconds / 1e9,
                    self.plan["mass_kg"],
                    minimum_share=getattr(self.shift, "minimum_measured_share", 0.25),
                )
            contacts_ok = contacts_ok and load_state["eligible"]
            if self.ready and not load_state["eligible"]:
                raise RuntimeError(f"Partial-unload force guard: {load_state}")
        shift_sample = None
        load_control_state = None
        if self.shift and self.ready:
            if not contacts_ok:
                raise RuntimeError(
                    f"Lateral shift requires fresh ground contact at both feet: {contact_state}"
                )
            shift_sample = self.shift.sample(stamp - self.shift_origin)
            applied = shift_sample
            if self.load_control is not None:
                if stamp - self.last_recorded >= 0.009:
                    outer_dt = (
                        0.0
                        if self.last_load_update is None
                        else stamp - self.last_load_update
                    )
                    if timing.get("integral_frozen_gap", False):
                        outer_dt = 0.0
                    target_left = shift_sample["predicted_force_n"]["left"] / (
                        self.plan["mass_kg"] * 9.81
                    )
                    self.load_control_state = self.load_control.update(
                        target_left, load_state, outer_dt
                    )
                    self.last_load_update = stamp
                load_control_state = self.load_control_state
                applied = self.shift.command_sample(
                    stamp - self.shift_origin, self.load_control.offset
                )
            self.control.target = applied["target_rad"]
            self.control.ff = applied["feedforward_nm"]
            if shift_sample["phase"] != self.last_phase:
                self.last_phase = shift_sample["phase"]
                self.get_logger().info("[shift:" + self.last_phase + "] " + str(stamp))
        if (
            self.active
            and ACTIVE["name"] == "robotis_p54"
            and any(abs(speed) > limit for speed, limit in zip(v, MOTOR_SPEEDS))
        ):
            raise RuntimeError(
                "P54 output speed exceeded the configured 2.4 rad/s operating envelope"
            )
        derivative = (
            self.shift.derivative_gains(
                0.0
                if self.shift_origin is None
                else max(0.0, stamp - self.shift_origin)
            )
            if self.zero_load
            else None
        )
        proportional = (
            self.shift.proportional_gains(
                0.0
                if self.shift_origin is None
                else max(0.0, stamp - self.shift_origin)
            )
            if self.zero_load
            else None
        )
        terms = self.control.update(
            q, v, dt, derivative_gains=derivative, proportional_gains=proportional
        )
        if (
            (self.unloading or self.lowering)
            and self.active
            and any(abs(t) > 0.8 * c for t, c in zip(terms["command"], MOTOR_CAPS))
        ):
            raise RuntimeError(
                "Partial unloading consumed the reserved 20% motor headroom"
            )
        self.last_stamp = stamp
        self.last_wall = time.monotonic()
        if self.active:
            if max(abs(e) for e in terms["error"]) > 0.2 or max(abs(x) for x in v) > 3:
                raise RuntimeError("Roll posture exceeded angle/speed guard")
            stable = (
                contacts_ok
                and max(abs(e) for e in terms["error"]) < 0.002
                and max(abs(x) for x in v) < 0.002
                and not timing.get("integral_frozen_gap", False)
            )
            if not stable:
                self.stable_since = None
            elif self.stable_since is None:
                self.stable_since = stamp
            if (
                not self.ready
                and stamp - self.active_time >= 15
                and self.stable_since is not None
                and stamp - self.stable_since >= 2
            ):
                self.ready = True
                self.shift_origin = stamp
                self.get_logger().info(
                    "[roll:ready] Ten bounded effort joints settled. Verify pose, contacts and COM separately."
                )
        self.pub.publish(Float64MultiArray(data=terms["command"]))
        if self.stream and stamp - self.last_recorded >= 0.009:
            self.stream.write(
                json.dumps(
                    dict(
                        sim_time=stamp,
                        active=self.active,
                        ready=self.ready,
                        position=q,
                        velocity=v,
                        target_rad=self.control.target.copy(),
                        integral_gains=self.control.ki,
                        shift=shift_sample,
                        load_feedback=load_control_state,
                        shift_contacts=contact_state,
                        load_guard=load_state,
                        lift_guard=lift_state,
                        lift_observation=(
                            self.lift_observation if self.foot_lift else None
                        ),
                        pelvis_tilt_rad=self.tilt,
                        imu_age_s=(
                            None
                            if self.imu_stamp is None
                            else self.get_clock().now().nanoseconds / 1e9
                            - self.imu_stamp
                        ),
                        imu_minus_joint_stamp_s=(
                            None if self.imu_stamp is None else self.imu_stamp - stamp
                        ),
                        **timing,
                        **terms,
                    )
                )
                + "\n"
            )
            self.last_recorded = stamp

    def supervise(self):
        now = time.monotonic()
        if not self.active and now - self.start_wall > 60:
            raise TimeoutError("Roll effort startup exceeded 60 wall seconds")
        for last, label in [(self.last_wall, "Joint"), (self.imu_wall, "IMU")]:
            if last is not None and now - last > 1:
                raise RuntimeError(label + " feedback absent for one wall second")
        if (
            self.active
            or self.last_stamp is None
            or self.imu_stamp is None
            or not self.pub.get_subscription_count()
        ):
            return
        if not startup_timing_ready(
            self.last_stamp, self.get_clock().now().nanoseconds / 1e9
        ):
            return
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
            raise RuntimeError("Command-limit enforcement required")
        if self.switch_future is None:
            if self.switch.service_is_ready():
                request = SwitchController.Request()
                request.activate_controllers = ["roll_effort_controller"]
                request.strictness = SwitchController.Request.STRICT
                request.activate_asap = True
                request.timeout.sec = 5
                self.switch_future = self.switch.call_async(request)
        elif self.switch_future.done():
            if not self.switch_future.result().ok:
                raise RuntimeError("Roll effort activation failed")
            self.active = True
            self.active_time = self.last_stamp
            self.get_logger().info(
                "[roll:activated] Inactive startup hold handed over to bounded efforts."
            )

    def close(self):
        try:
            if rclpy.ok():
                self.pub.publish(Float64MultiArray(data=[0.0] * 10))
        finally:
            if self.lift_transport:
                self.lift_transport.unsubscribe("/biped/com_state")
            if self.stream:
                self.stream.close()


def main():
    node = None
    executor = None
    stopping = [False]
    previous = {}

    def request_stop(signum, frame):
        stopping[0] = True

    # Preserve the context through the final callback and zero-effort publish.
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, request_stop)
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = RollStanding()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        while not stopping[0] and rclpy.ok():
            executor.spin_once(timeout_sec=0.02)
    except Exception:
        import traceback

        details = traceback.format_exc()
        if node is not None:
            # ROS node logs retain this message; launch.log alone omits stderr.
            node.get_logger().error("[controller:fault] " + details)
            from my_robot_description.runtime.fault_report import save_fault

            try:
                report = node.get_parameter("fault_report").value
                save_fault(report, node, details)
                if report:
                    node.get_logger().error("[controller:fault-report] " + report)
            except Exception as report_error:
                node.get_logger().error(
                    "Cannot save fault report: " + str(report_error)
                )
        raise
    finally:
        try:
            if node is not None:
                node.supervision_timer.cancel()
            if executor is not None:
                executor.shutdown()
            try:
                if node is not None:
                    try:
                        node.close()
                    finally:
                        node.destroy_node()
            finally:
                if rclpy.ok():
                    rclpy.shutdown()
        finally:
            for sig, handler in previous.items():
                signal.signal(sig, handler)


if __name__ == "__main__":
    main()
