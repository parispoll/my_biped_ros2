#!/usr/bin/env python3
"""Record one effort-controlled pendulum test; use run_torque_bench.py for isolated runs."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse
import json
import math
import statistics
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64MultiArray
from ament_index_python.packages import get_package_share_directory
import xacro
from my_robot_description.model.torque import clamp_torque, pendulum_from_urdf
from my_robot_description.experiments.disturbance import process_sensor_callbacks

CASES = (
    "pulse-positive",
    "pulse-negative",
    "stop-positive",
    "stop-negative",
    "hold",
    "hold-limited",
    "gravity-release",
)


def program(case, t, angle, velocity, model):
    """Return phase, raw request, transmitted ROS command and optional position target."""
    soft_limit = 0.25 if case == "hold-limited" else model.effort_limit
    if t < 2:
        target = 0.3 if case == "gravity-release" else 0.0
        raw = model.requested_torque(angle, velocity, target)
        return "prepare", raw, clamp_torque(raw, soft_limit), target
    if case.startswith("pulse"):
        raw = (4.0 if case == "pulse-positive" else -4.0) if t < 2.04 else 0.0
        return ("pulse" if t < 2.04 else "coast"), raw, raw, None
    if case.startswith("stop"):
        raw = (4.0 if case == "stop-positive" else -4.0) if t < 3 else 0.0
        return ("drive_stop" if t < 3 else "release_stop"), raw, raw, None
    if case == "gravity-release":
        return "gravity_release", 0.0, 0.0, None
    u = max(0, min(1, (t - 2) / 2))
    target = 0.5 * (10 * u**3 - 15 * u**4 + 6 * u**5)
    raw = model.requested_torque(angle, velocity, target)
    return ("move" if t < 4 else "hold"), raw, clamp_torque(raw, soft_limit), target


def analyze(rows, model, case):
    report = {
        "case": case,
        "samples": len(rows),
        "hard_torque_limit_nm": model.effort_limit,
        "position_limits_rad": [model.lower, model.upper],
        "velocity_limit_rad_s": model.velocity_limit,
        "angle_min_rad": min(r["angle"] for r in rows),
        "angle_max_rad": max(r["angle"] for r in rows),
        "peak_speed_rad_s": max(abs(r["velocity"]) for r in rows),
        "max_ros_command_nm": max(abs(r["command"]) for r in rows),
        "reported_effort_note": "joint_states.effort is a transmitted joint-wrench projection, not independently calibrated motor torque.",
    }
    if (
        report["angle_min_rad"] < model.lower - 0.005
        or report["angle_max_rad"] > model.upper + 0.005
    ):
        raise ValueError(f"Joint exceeded its travel limits: {report}")
    if report["peak_speed_rad_s"] > model.velocity_limit + 0.01:
        raise ValueError(f"Joint exceeded its speed limit: {report}")
    if case.startswith("pulse"):
        pulse = [r for r in rows if r["phase"] == "pulse"]
        start = pulse[0]["time"]
        end = next(r["time"] for r in rows if r["phase"] == "coast")
        report["recorded_pulse_command_window_s"] = end - start
        estimates = []
        for i in range(1, len(rows) - 1):
            row = rows[i]
            if (
                0.012 - 1e-8 <= row["time"] - start <= 0.028 + 1e-8
                and row["phase"] == "pulse"
            ):
                previous, following = rows[i - 1], rows[i + 1]
                acceleration = (following["velocity"] - previous["velocity"]) / (
                    following["time"] - previous["time"]
                )
                estimates.append(
                    model.inertia * acceleration
                    - model.gravity_torque(row["angle"])
                    + model.damping * row["velocity"]
                )
        if len(estimates) < 3:
            raise ValueError("Too few pulse samples to estimate applied torque")
        estimate = statistics.median(estimates)
        sign = 1 if case == "pulse-positive" else -1
        report.update(
            requested_pulse_nm=4 * sign,
            dynamics_inferred_torque_nm=estimate,
            pulse_estimate_samples=len(estimates),
            torque_cap_verified=abs(estimate - sign * model.effort_limit) < 0.2,
        )
        if not report["torque_cap_verified"]:
            raise ValueError(
                f"Dynamics do not support the configured torque cap: {report}"
            )
    elif case.startswith("stop"):
        driven = [r for r in rows if 2.7 < r["time"] < 3]
        target = model.upper if case == "stop-positive" else model.lower
        report["mean_angle_at_driven_stop_rad"] = statistics.mean(
            r["angle"] for r in driven
        )
        if abs(report["mean_angle_at_driven_stop_rad"] - target) > 0.02:
            raise ValueError(f"Did not reach the expected mechanical stop: {report}")
    elif case == "gravity-release":
        released = [r for r in rows if r["phase"] == "gravity_release"]
        initial = released[0]
        later = next(r for r in released if r["time"] - initial["time"] >= 0.02)
        residuals = []
        for i in range(1, len(rows) - 1):
            row = rows[i]
            if (
                row["phase"] == "gravity_release"
                and 0.012 <= row["time"] - initial["time"] <= 0.032
            ):
                previous, following = rows[i - 1], rows[i + 1]
                acceleration = (following["velocity"] - previous["velocity"]) / (
                    following["time"] - previous["time"]
                )
                residuals.append(
                    model.inertia * acceleration
                    - model.gravity_torque(row["angle"])
                    + model.damping * row["velocity"]
                )
        residual = statistics.median(residuals) if residuals else None
        report.update(
            release_angle_rad=initial["angle"],
            velocity_after_20ms=later["velocity"],
            predicted_initial_acceleration_rad_s2=model.acceleration(
                initial["angle"], initial["velocity"], 0
            ),
            gravity_release_residual_torque_nm=residual,
        )
        if (
            not 0.28 < initial["angle"] < 0.32
            or later["velocity"] >= 0
            or residual is None
            or abs(residual) > 0.1
        ):
            raise ValueError(
                f"Gravity response disagrees with the unforced model: {report}"
            )
    else:
        tail = [r for r in rows if r["time"] >= rows[-1]["time"] - 1]
        final = statistics.mean(r["angle"] for r in tail)
        report.update(
            target_rad=0.5,
            final_mean_angle_rad=final,
            final_error_rad=0.5 - final,
            final_peak_velocity_rad_s=max(abs(r["velocity"]) for r in tail),
            software_limit_nm=0.25 if case == "hold-limited" else model.effort_limit,
            saturated_fraction=sum(
                abs(r["raw_request"]) > abs(r["command"]) + 1e-9 for r in rows
            )
            / len(rows),
        )
        if case == "hold" and (
            abs(final - 0.5) > 0.02 or report["final_peak_velocity_rad_s"] > 0.03
        ):
            raise ValueError(f"Unsaturated hold did not converge: {report}")
        if case == "hold-limited":
            predicted = math.asin(0.25 / model.gravity_gain)
            report["predicted_saturated_equilibrium_rad"] = predicted
            if abs(final - predicted) > 0.03 or report["max_ros_command_nm"] > 0.250001:
                raise ValueError(
                    f"Limited hold does not match expected saturation: {report}"
                )
    return {"passed": True, **report}


def run(case, folder):
    folder.mkdir(parents=True, exist_ok=False)
    share = Path(get_package_share_directory("my_robot_description"))
    xml = xacro.process_file(str(share / "urdf/torque_bench.urdf.xacro")).toxml()
    model = pendulum_from_urdf(xml)
    node = Node("torque_bench_experiment")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    latest = {}
    clock = [None]
    publisher = node.create_publisher(
        Float64MultiArray, "/bench_effort_controller/commands", 10
    )

    def on_joint(message):
        if "bench_joint" in message.name:
            i = message.name.index("bench_joint")
            latest.update(
                time=message.header.stamp.sec + message.header.stamp.nanosec / 1e9,
                angle=message.position[i],
                velocity=message.velocity[i],
                reported_effort=message.effort[i] if len(message.effort) > i else None,
            )

    node.create_subscription(
        JointState, "/joint_states", on_joint, qos_profile_sensor_data
    )
    node.create_subscription(
        Clock,
        "/clock",
        lambda m: clock.__setitem__(0, m.clock.sec + m.clock.nanosec / 1e9),
        qos_profile_sensor_data,
    )
    rows = []
    start = None
    last = None
    outcome = {"passed": False, "case": case}
    stream = (folder / "samples.jsonl").open("w")
    zero = Float64MultiArray(data=[0.0])
    try:
        parameters = AsyncParameterClient(node, "/controller_manager")
        if not parameters.wait_for_services(timeout_sec=10):
            raise TimeoutError("Controller-manager parameter service unavailable")
        future = parameters.get_parameters(["enforce_command_limits"])
        executor.spin_until_future_complete(future, timeout_sec=5)
        if not future.done() or not future.result().values[0].bool_value:
            raise RuntimeError(
                "Bench requires controller-manager command-limit enforcement"
            )
        end = 10 if case == "hold-limited" else (7 if case == "hold" else 4)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            process_sensor_callbacks(executor)
            if (
                not latest
                or clock[0] is None
                or publisher.get_subscription_count() == 0
            ):
                continue
            if clock[0] - latest["time"] > 0.1:
                raise RuntimeError("Joint feedback is stale")
            if latest["time"] == last:
                continue
            start = latest["time"] if start is None else start
            t = latest["time"] - start
            phase, raw, command, target = program(
                case, t, latest["angle"], latest["velocity"], model
            )
            if not all(
                math.isfinite(v) for v in (latest["angle"], latest["velocity"], command)
            ):
                raise RuntimeError("Non-finite command or measurement")
            if (
                not model.lower - 0.01 <= latest["angle"] <= model.upper + 0.01
                or abs(latest["velocity"]) > 7
            ):
                raise RuntimeError("Bench exceeded joint angle/speed experiment bounds")
            publisher.publish(Float64MultiArray(data=[command]))
            row = {
                "time": t,
                "sim_time": latest["time"],
                "phase": phase,
                "angle": latest["angle"],
                "velocity": latest["velocity"],
                "reported_effort": latest["reported_effort"],
                "raw_request": raw,
                "command": command,
                "target": target,
            }
            rows.append(row)
            stream.write(json.dumps(row) + "\n")
            stream.flush()
            last = latest["time"]
            if t >= end:
                outcome = analyze(rows, model, case)
                break
        else:
            raise TimeoutError("Bench experiment timed out")
    except BaseException as error:
        outcome.update(error=str(error))
        raise
    finally:
        publisher.publish(zero)
        executor.spin_once(timeout_sec=0.05)
        publisher.publish(zero)
        stream.close()
        (folder / "summary.json").write_text(json.dumps(outcome, indent=2) + "\n")
        executor.shutdown(timeout_sec=1)
        node.destroy_node()
    print(json.dumps(outcome, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    try:
        run(args.case, args.output)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
