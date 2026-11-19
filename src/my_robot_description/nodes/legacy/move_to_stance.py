#!/usr/bin/env python3
"""Execute a calculated, slow symmetric crouch through a monitored ROS action."""

import json
import math
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance, JointTrajectoryControllerState
from rclpy.qos import qos_profile_sensor_data
from my_robot_description.validation.tracking import TrackingRecorder
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from my_robot_description.model.stance import JOINT_NAMES, model_from_urdf


def wait(node, future, seconds):
    rclpy.spin_until_future_complete(node, future, timeout_sec=seconds)
    if not future.done():
        raise TimeoutError(
            "Trajectory action timed out; inspect simulation time and controller state"
        )
    return future.result()


def execute(node, recorder):
    model = model_from_urdf(node.declare_parameter("robot_description", "").value)
    target = float(node.declare_parameter("knee_bend", 0.4).value)
    duration = float(node.declare_parameter("transition_duration", 8.0).value)
    if not math.isfinite(duration) or not 4 <= duration <= 60:
        raise ValueError("transition_duration must be between 4 and 60 seconds")
    target_pose = model.pose(target)
    node.create_subscription(
        JointTrajectoryControllerState,
        "/joint_trajectory_controller/controller_state",
        lambda message: recorder.record(
            message, node.get_clock().now().nanoseconds / 1e9
        ),
        qos_profile_sensor_data,
    )
    latest = {}

    def on_state(message):
        latest.update(zip(message.name, message.position))

    subscription = node.create_subscription(JointState, "/joint_states", on_state, 10)
    deadline = time.monotonic() + 15
    while not all(name in latest for name in JOINT_NAMES):
        if time.monotonic() > deadline:
            raise TimeoutError("No complete /joint_states message within 15 seconds")
        rclpy.spin_once(node, timeout_sec=0.1)
    start = [latest[name] for name in JOINT_NAMES]
    node.get_logger().info(f"[stance] Measured starting joints: {start}")
    start_knee = (start[1] + start[4]) / 2
    # Physics roundoff can report a tiny negative angle at the zero knee stop.
    if abs(start_knee) < 1e-6:
        start_knee = 0.0
    planned_start = list(model.pose(start_knee)) * 2
    if any(
        not math.isfinite(a) or abs(a - b) > 0.03 for a, b in zip(start, planned_start)
    ):
        raise ValueError(
            "Current joint pose is not near a supported symmetric stance; reset simulation first"
        )
    client = ActionClient(
        node,
        FollowJointTrajectory,
        "/joint_trajectory_controller/follow_joint_trajectory",
    )
    if not client.wait_for_server(timeout_sec=15):
        raise TimeoutError("Trajectory controller action server unavailable")
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = JOINT_NAMES
    for index in range(81):
        u = index / 80
        # Sample a quintic easing schedule; position waypoints are interpolated by the controller.
        fraction = 10 * u**3 - 15 * u**4 + 6 * u**5
        knee = start_knee + (target - start_knee) * fraction
        point = JointTrajectoryPoint()
        point.positions = list(model.pose(knee)) * 2
        seconds = 0.2 + duration * u
        point.time_from_start = Duration(
            sec=int(seconds), nanosec=round((seconds - int(seconds)) * 1e9)
        )
        goal.trajectory.points.append(point)
    goal.path_tolerance = [
        JointTolerance(name=name, position=0.08) for name in JOINT_NAMES
    ]
    goal.goal_tolerance = [
        JointTolerance(name=name, position=0.02) for name in JOINT_NAMES
    ]
    goal.goal_time_tolerance = Duration(sec=3)
    node.get_logger().info(
        f"[stance] Target hip/knee/ankle: {target_pose}; geometry: {model.geometry(target_pose)}"
    )
    recorder.phase = "goal_requested"
    handle = wait(node, client.send_goal_async(goal), 10)
    if not handle.accepted:
        raise RuntimeError("Controller rejected the stance trajectory")
    recorder.phase = "active"
    try:
        result = wait(node, handle.get_result_async(), duration + 20)
    except Exception:
        wait(node, handle.cancel_goal_async(), 3)
        raise
    if result.status != GoalStatus.STATUS_SUCCEEDED or result.result.error_code != 0:
        raise RuntimeError(
            f"Trajectory failed: {result.result.error_code}: {result.result.error_string}"
        )
    print(
        json.dumps(
            {
                "trajectory_succeeded": True,
                "target_per_leg": target_pose,
                "predicted_geometry": model.geometry(target_pose),
            },
            indent=2,
        ),
        flush=True,
    )
    node.destroy_subscription(subscription)


def main():
    rclpy.init()
    node = Node("move_to_stance")
    recorder = TrackingRecorder(node.declare_parameter("tracking_log", "").value)
    outcome = "failed"
    try:
        execute(node, recorder)
        outcome = "succeeded"
    except Exception as error:
        node.get_logger().error(f"[stance] {error}")
        raise
    finally:
        report = recorder.finish(
            outcome, {"path_tolerance": 0.08, "goal_tolerance": 0.02}
        )
        node.get_logger().info(f"[tracking] {json.dumps(report)}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
