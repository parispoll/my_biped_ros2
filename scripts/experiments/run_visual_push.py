#!/usr/bin/env python3
"""One visual force pulse, without a reference trial or recovery assessment."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import os
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from gz.transport13 import Node as GazeboNode
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.pose_v_pb2 import Pose_V
from my_robot_description.experiments.effort_push import EffortPushConfig
from my_robot_description.visualization.force_visual import ForceArrowVisualizer
from my_robot_description.validation.experiment_failure import launch_failure
from scripts.experiments.run_effort_push import launch_command


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force-x", type=float, default=2.0)
    parser.add_argument("--duration", type=float, default=0.2)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--balance-enabled", action="store_true")
    parser.add_argument(
        "--full-settle",
        action="store_true",
        help="Use the full measured posture/balance warmup while retaining visual fall continuation",
    )
    parser.add_argument("--balance-kd", type=float, choices=[4.0, 8.0], default=4.0)
    parser.add_argument(
        "--ankle-balance-limit", type=float, choices=[0.5, 1.5], default=0.5
    )
    parser.add_argument("--ros-domain-id", type=int, choices=range(101), default=58)
    args = parser.parse_args()
    if args.balance_kd != 4.0 and (
        not args.balance_enabled or args.ankle_balance_limit != 1.5
    ):
        parser.error(
            "Higher damping requires active balance with the 1.5 N m correction cap"
        )
    if args.ankle_balance_limit != 0.5 and not args.balance_enabled:
        parser.error("Experimental ankle response requires --balance-enabled")
    config = EffortPushConfig(force_x=args.force_x, duration=args.duration).validate()
    folder = args.output.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    os.environ.update(
        ROS_DOMAIN_ID=str(args.ros_domain_id),
        GZ_PARTITION="visual_push_" + uuid.uuid4().hex,
        ROS_LOG_DIR=str(Path.cwd() / "log/runtime"),
    )
    (folder / "visual-session.json").write_text(
        json.dumps(
            dict(
                ros_domain_id=args.ros_domain_id,
                gz_partition=os.environ["GZ_PARTITION"],
                force_x=config.force_x,
                duration=config.duration,
                ankle_balance_limit_nm=args.ankle_balance_limit,
                full_settling=args.full_settle,
                balance_kd_nm_s_per_rad=args.balance_kd,
            )
        )
        + "\n"
    )
    rclpy.init()
    node = Node("visual_force_pulse")
    clock = [None]
    body = {}

    def on_clock(message):
        clock[0] = message.clock.sec + message.clock.nanosec / 1e9

    node.create_subscription(Clock, "/clock", on_clock, qos_profile_sensor_data)
    transport = GazeboNode()
    push = transport.advertise("/world/standing/wrench/persistent", EntityWrench)
    clear = transport.advertise("/world/standing/wrench/clear", Entity)
    entity = Entity(name="my_biped", type=Entity.MODEL)
    command = EntityWrench()
    command.entity.CopyFrom(entity)
    command.wrench.force.x = config.force_x

    def on_pose(message):
        for pose in message.pose:
            if pose.name == "my_biped":
                body.update(
                    x=pose.position.x, y=pose.position.y, height=pose.position.z
                )

    transport.subscribe(Pose_V, "/world/standing/pose/info", on_pose)
    visual = ForceArrowVisualizer()
    simulation = None
    events = []

    def event(name):
        events.append(dict(event=name, sim_time=clock[0], wall_time=time.monotonic()))
        (folder / "visual-events.json").write_text(json.dumps(events, indent=2) + "\n")
        print("[visual] " + name, flush=True)

    try:
        with (folder / "launch.log").open("w") as log:
            simulation = subprocess.Popen(
                launch_command(
                    folder,
                    True,
                    0.2,
                    False,
                    False,
                    0.25,
                    0.001,
                    "explicit",
                    "pid",
                    args.balance_enabled,
                    args.balance_enabled,
                    ankle_balance_limit=args.ankle_balance_limit,
                    balance_kd=args.balance_kd,
                )
                + [
                    "visual_keep_alive:=true",
                    f"fast_visual_ready:={str(not args.full_settle).lower()}",
                    "com_visual:=true",
                    f'com_recording:={folder / "com.jsonl"}',
                ],
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            deadline = time.monotonic() + (160 if args.full_settle else 110)
            marker = "[balance:ready]" if args.balance_enabled else "[effort:ready]"
            print(
                "[visual] One simulation warming up; no reference trial or recovery assessment.",
                flush=True,
            )
            while (
                marker not in (folder / "launch.log").read_text()
                or clock[0] is None
                or not body
                or not push.has_connections()
            ):
                rclpy.spin_once(node, timeout_sec=0.01)
                if simulation.poll() is not None:
                    raise RuntimeError(launch_failure(folder / "launch.log"))
                if "[visual:controller-stopped]" in (folder / "launch.log").read_text():
                    raise RuntimeError(
                        "Controller stopped before standing readiness; no visual push applied"
                    )
                if time.monotonic() > deadline:
                    raise TimeoutError("Visual simulation did not become ready")
            # Wall-clock delay is independent of Gazebo real-time factor.
            push_at_wall = time.monotonic()
            pulse_deadline = push_at_wall + config.duration / 0.25 + 30
            print("[visual] Standing ready. Applying force now.", flush=True)
            pushed = None
            released = False
            last_clock = clock[0]
            last_tick = time.monotonic()
            controller_stopped = False
            while simulation.poll() is None:
                if (
                    not controller_stopped
                    and "[visual:controller-stopped]"
                    in (folder / "launch.log").read_text()
                ):
                    controller_stopped = True
                    event("controller_stopped_simulation_continues")
                rclpy.spin_once(node, timeout_sec=0.01)
                if clock[0] != last_clock:
                    last_clock = clock[0]
                    last_tick = time.monotonic()
                if time.monotonic() - last_tick > 10:
                    raise RuntimeError(
                        "Simulation clock stopped; clearing visual pulse"
                    )
                if pushed is None and time.monotonic() >= push_at_wall:
                    push.publish(command)
                    pushed = clock[0]
                    event("push_published")
                if pushed is not None and not released:
                    visual.update(config.force_x, body)
                    if clock[0] - pushed >= config.duration:
                        clear.publish(entity)
                        released = True
                        visual.hide()
                        event("clear_published")
                        print(
                            "[visual] Pulse finished. Gazebo stays open; press Stop & clear run to close it. No recovery verdict.",
                            flush=True,
                        )
                if not released and time.monotonic() > pulse_deadline:
                    raise TimeoutError("Visual pulse deadline exceeded")
            raise RuntimeError(
                "Visual simulation stopped: "
                + str(launch_failure(folder / "launch.log"))
            )
    except KeyboardInterrupt:
        print("[visual] Stopped by user.", flush=True)
    finally:
        clear.publish(entity)
        time.sleep(0.1)
        visual.close()
        if simulation is not None:
            if simulation.poll() is None:
                simulation.send_signal(signal.SIGINT)
            try:
                simulation.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(simulation.pid, signal.SIGKILL)
                simulation.wait()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
