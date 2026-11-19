#!/usr/bin/env python3
"""Record a gentle push on effort standing; normally started by run_effort_push.py."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import math
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from rclpy.parameter_client import AsyncParameterClient
from my_robot_description.model.stance import JOINT_NAMES, model_from_urdf
from my_robot_description.control.posture_effort import effort_limits_from_urdf
from my_robot_description.control.posture_setup import load_posture_setup
from functools import partial
from my_robot_description.experiments.effort_push import (
    EffortPushConfig,
    summarize_effort,
)
from scripts.experiments.push_experiment import run, stamp


def attach_effort_stream(node, latest, errors, executor, setup=None):
    if setup is not None:
        target, limits = load_posture_setup(setup)
    else:
        # Read the actual running controller's target; do not assume a fixed knee bend.
        parameters = AsyncParameterClient(node, "/standing_effort")
        if not parameters.wait_for_services(timeout_sec=5):
            raise RuntimeError("Effort posture node is not available")
        future = parameters.get_parameters(["robot_description", "knee_bend"])
        executor.spin_until_future_complete(future, timeout_sec=5)
        if not future.done() or future.result() is None:
            raise RuntimeError("Cannot read the effort posture target")
        xml, knee = future.result().values
        target = list(model_from_urdf(xml.string_value).pose(knee.double_value)) * 2
        limits = effort_limits_from_urdf(xml.string_value)
    latest["command"] = None

    def commands(msg):
        if latest["clock"] is None:
            return
        if len(msg.data) != 6 or not all(math.isfinite(x) for x in msg.data):
            errors.append(ValueError("Invalid effort command vector"))
            return
        if any(abs(value) > limit + 1e-9 for value, limit in zip(msg.data, limits)):
            errors.append(
                ValueError("Observed torque command exceeds its effort-interface cap")
            )
            return
        latest["command"] = dict(time=latest["clock"], values=list(msg.data))

    def joints(msg):
        try:
            if latest["command"] is None:
                return
            indices = [msg.name.index(name) for name in JOINT_NAMES]
            actual = [msg.position[i] for i in indices]
            if not all(math.isfinite(x) for x in actual):
                raise ValueError("Non-finite joint positions")
            latest["joints"] = dict(
                time=stamp(msg.header.stamp),
                names=JOINT_NAMES,
                desired=target,
                actual=actual,
                error=[a - b for a, b in zip(target, actual)],
                command=latest["command"]["values"],
                command_time=latest["command"]["time"],
            )
        except Exception as error:
            errors.append(error)

    node.create_subscription(
        Float64MultiArray, "/standing_effort_controller/commands", commands, 1
    )
    node.create_subscription(
        JointState, "/joint_states", joints, qos_profile_sensor_data
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--force-x", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=0.2)
    parser.add_argument("--recovery", type=float, default=10.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--controller-setup",
        type=Path,
        help="Actual startup targets/caps written by the controller; avoids live metadata RPC",
    )
    parser.add_argument(
        "--ready-file",
        type=Path,
        help="Wait for runner readiness signal before starting the fresh baseline",
    )
    parser.add_argument(
        "--wall-timeout",
        type=float,
        default=35.0,
        help="Wall deadline only; sensor freshness limits are unchanged",
    )
    args = parser.parse_args()
    config = EffortPushConfig(
        force_x=args.force_x, duration=args.duration, recovery=args.recovery
    ).validate()
    rclpy.init()
    try:
        run(
            config,
            args.output,
            joint_subscription=partial(
                attach_effort_stream, setup=args.controller_setup
            ),
            analyzer=summarize_effort,
            wall_timeout=args.wall_timeout,
            ready_file=args.ready_file,
        )
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
