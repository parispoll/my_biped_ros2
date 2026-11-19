#!/usr/bin/env python3
"""Record one small push on a settled crouch; prefer run_push_trials.py for owned simulations."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
from dataclasses import asdict
import json
import math
import threading
import time
import rclpy
from rclpy.node import Node as RosNode
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from ros_gz_interfaces.msg import Contacts
from control_msgs.msg import JointTrajectoryControllerState
from gz.transport13 import Node as GazeboNode
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.pose_v_pb2 import Pose_V
from my_robot_description.sensing.contacts import ground_contact_pairs
from my_robot_description.experiments.disturbance import (
    PushConfig,
    summarize,
    process_sensor_callbacks,
)
from my_robot_description.visualization.force_visual import ForceArrowVisualizer


def stamp(value):
    # ROS uses nanosec; Gazebo protobuf time uses nsec.
    nanoseconds = value.nanosec if hasattr(value, "nanosec") else value.nsec
    return value.sec + nanoseconds / 1e9


def run(
    config,
    folder,
    joint_subscription=None,
    analyzer=summarize,
    wall_timeout=35,
    ready_file=None,
):
    """Record a push; optional adapters supply a different joint controller stream."""
    config.validate()
    if not math.isfinite(wall_timeout) or wall_timeout <= 0:
        raise ValueError("Wall timeout must be finite and positive")
    folder.mkdir(parents=True, exist_ok=False)
    node = RosNode("record_push_experiment")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    transport = GazeboNode()
    push = transport.advertise("/world/standing/wrench/persistent", EntityWrench)
    clear = transport.advertise("/world/standing/wrench/clear", Entity)
    entity = Entity(name="my_biped", type=Entity.MODEL)  # canonical link is base_link
    command = EntityWrench()
    command.entity.CopyFrom(entity)
    command.wrench.force.x = config.force_x
    pose_lock = threading.Lock()
    pose = {}
    latest = {"clock": None, "imu": None, "joints": None, "left": None, "right": None}
    errors = []

    def on_pose(message):
        for item in message.pose:
            if item.name == "my_biped":
                with pose_lock:
                    pose.update(
                        time=stamp(message.header.stamp),
                        height=item.position.z,
                        x=item.position.x,
                        y=item.position.y,
                    )

    transport.subscribe(Pose_V, "/world/standing/pose/info", on_pose)

    def on_imu(message):
        q = message.orientation
        values = (
            q.x,
            q.y,
            q.z,
            q.w,
            message.angular_velocity.y,
            message.linear_acceleration.x,
        )
        if message.header.frame_id != "base_link" or not all(
            math.isfinite(v) for v in values
        ):
            errors.append(ValueError("Unexpected IMU frame or non-finite measurement"))
            return
        if abs(sum(v * v for v in values[:4]) - 1) > 0.002:
            errors.append(ValueError("Invalid IMU orientation quaternion"))
            return
        latest["imu"] = {
            "time": stamp(message.header.stamp),
            "pitch": math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x)))),
            "pitch_rate": message.angular_velocity.y,
            "acceleration_x": message.linear_acceleration.x,
            "tilt": math.acos(max(-1, min(1, 1 - 2 * (q.x * q.x + q.y * q.y)))),
        }

    def on_joints(message):
        latest["joints"] = {
            "time": stamp(message.header.stamp),
            "names": list(message.joint_names),
            "desired": list(message.reference.positions),
            "actual": list(message.feedback.positions),
            "error": list(message.error.positions),
        }

    def on_contacts(message, side):
        try:
            latest[side] = {
                "time": stamp(message.header.stamp),
                "ground": bool(ground_contact_pairs(message, side)),
            }
        except Exception as error:
            errors.append(error)

    def on_clock(message):
        current = stamp(message.clock)
        if latest["clock"] is not None and current < latest["clock"]:
            errors.append(ValueError("Simulation clock moved backwards"))
        latest["clock"] = current

    node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
    node.create_subscription(Imu, "/biped/imu", on_imu, qos_profile_sensor_data)
    if joint_subscription is None:
        node.create_subscription(
            JointTrajectoryControllerState,
            "/joint_trajectory_controller/controller_state",
            on_joints,
            qos_profile_sensor_data,
        )
    else:
        joint_subscription(node, latest, errors, executor)
    for side in ("left", "right"):
        node.create_subscription(
            Contacts,
            f"/biped/{side}_foot/contacts",
            lambda m, side=side: on_contacts(m, side),
            qos_profile_sensor_data,
        )
    samples = []
    events = []
    beginning = None
    ready_since = None
    pushed = None
    released = None
    last_sample = None
    deadline = time.monotonic() + wall_timeout
    stream = (folder / "samples.jsonl").open("w")
    result = {"completed": False, "requested": asdict(config)}
    force_visual = ForceArrowVisualizer()
    connected_wall = time.monotonic()
    if ready_file is not None:
        print(
            "[experiment] recorder connected; waiting for posture readiness signal",
            flush=True,
        )

    def event(name, current):
        value = {"event": name, "sim_time": current, "wall_time": time.monotonic()}
        events.append(value)
        print(
            f'[experiment] {name.replace("_", " ")} at simulation time {current:.3f} s',
            flush=True,
        )
        (folder / "events.json").write_text(json.dumps(events, indent=2) + "\n")

    try:
        while time.monotonic() < deadline:
            process_sensor_callbacks(executor)
            if errors:
                raise errors[0]
            current = latest["clock"]
            with pose_lock:
                body = pose.copy()
            if (
                current is None
                or not body
                or any(latest[k] is None for k in ("imu", "joints", "left", "right"))
            ):
                continue
            if not push.has_connections() or not clear.has_connections():
                continue
            imu = latest["imu"]
            joints = latest["joints"]
            source_ages = {
                "imu": current - imu["time"],
                "joints": current - joints["time"],
                "pose": current - body["time"],
            }
            if "command_time" in joints:
                source_ages["torque_command"] = current - joints["command_time"]
            if beginning is None:
                if ready_file is not None and not ready_file.is_file():
                    ready_since = None
                    continue
                # Subscription discovery can briefly deliver incomplete/stale streams.
                # Require a continuously fresh settled window before the experiment starts.
                ready = (
                    max(source_ages.values()) <= 0.1
                    and abs(imu["pitch"]) <= 0.01
                    and abs(imu["pitch_rate"]) <= 0.02
                    and all(
                        latest[s]["ground"]
                        and current - latest[s]["time"] <= config.contact_max_age
                        for s in ("left", "right")
                    )
                )
                ready_since = (
                    (current if ready_since is None else ready_since) if ready else None
                )
                if ready_since is None or current - ready_since < 0.5:
                    continue
                beginning = current
                event("recording_started", current)
            if max(source_ages.values()) > 0.1:
                raise RuntimeError(
                    f"IMU, joint or body-pose data became stale: {source_ages}"
                )
            if imu["tilt"] > 0.25 or body["height"] < 0.45:
                raise RuntimeError(
                    "Stopping experiment: body exceeded baseline tilt/height bounds"
                )
            if pushed is None and current - beginning >= config.baseline:
                if abs(imu["pitch"]) > 0.01 or abs(imu["pitch_rate"]) > 0.02:
                    raise RuntimeError("Body is not settled immediately before push")
                if not all(
                    latest[s]["ground"]
                    and current - latest[s]["time"] <= config.contact_max_age
                    for s in ("left", "right")
                ):
                    raise RuntimeError(
                        "Both feet need fresh ground-contact evidence immediately before push"
                    )
                if not push.publish(command):
                    raise RuntimeError("Gazebo push publication failed")
                pushed = current
                force_visual.update(config.force_x, body)
                event("push_published", current)
            if (
                pushed is not None
                and released is None
                and current - pushed >= config.duration
            ):
                if not clear.publish(entity):
                    raise RuntimeError("Gazebo force-clear publication failed")
                released = current
                force_visual.hide()
                event("clear_published", current)
            if pushed is not None and released is None:
                force_visual.update(config.force_x, body)
            if last_sample is None or current - last_sample >= 0.0095:
                phase = (
                    "baseline"
                    if pushed is None
                    else ("push" if released is None else "recovery")
                )
                contacts = {}
                for side in ("left", "right"):
                    age = max(0, current - latest[side]["time"])
                    contacts[side] = {
                        "age": age,
                        "ground_contact": (
                            latest[side]["ground"]
                            if age <= config.contact_max_age
                            else None
                        ),
                    }
                sample = {
                    "sim_time": current,
                    "phase": phase,
                    **{k: imu[k] for k in ("pitch", "pitch_rate", "acceleration_x")},
                    "height": body["height"],
                    "body_x": body["x"],
                    "joint_names": joints["names"],
                    "desired_joints": joints["desired"],
                    "actual_joints": joints["actual"],
                    "joint_error": joints["error"],
                    "source_times": {
                        "imu": imu["time"],
                        "joints": joints["time"],
                        "pose": body["time"],
                    },
                    "contacts": contacts,
                }
                if "command" in joints:
                    sample.update(
                        joint_command_nm=joints["command"],
                        command_reception_clock=joints["command_time"],
                    )
                samples.append(sample)
                stream.write(json.dumps(sample) + "\n")
                stream.flush()
                last_sample = current
            if released is not None and current - released >= config.recovery:
                result = {"completed": True, **analyzer(samples, events, config)}
                break
        else:
            raise TimeoutError(
                "Experiment timed out waiting for live data or simulation time"
            )
    except BaseException as error:
        result.update(error=str(error), events=events, recorded_samples=len(samples))
        raise
    finally:
        # Clear even if recording fails or the user interrupts this process.
        clear.publish(entity)
        force_visual.close()
        result["force_visualization"] = force_visual.status()
        result["recorder_connected_wall_time"] = connected_wall
        executor.spin_once(timeout_sec=0.05)
        clear.publish(entity)
        stream.close()
        (folder / "summary.json").write_text(json.dumps(result, indent=2) + "\n")
        transport.unsubscribe("/world/standing/pose/info")
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1)
        node.destroy_node()
    print(json.dumps(result, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force-x", type=float, default=5)
    parser.add_argument("--duration", type=float, default=0.2)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    config = PushConfig(force_x=args.force_x, duration=args.duration).validate()
    rclpy.init()
    try:
        run(config, args.output)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
