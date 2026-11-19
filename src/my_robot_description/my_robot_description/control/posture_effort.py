"""Joint posture control math; no ROS and no body-balance feedback."""

import math
import xml.etree.ElementTree as ET
from my_robot_description.model.stance import JOINT_NAMES
from my_robot_description.model.torque import clamp_torque


def quaternion_tilt(x, y, z, w):
    """Angle between pelvis up and world up, for an IMU shutdown check."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or abs(norm - 1) > 0.001:
        raise ValueError("Invalid IMU orientation quaternion")
    return math.acos(max(-1.0, min(1.0, 1 - 2 * ((x / norm) ** 2 + (y / norm) ** 2))))


def effort_limits_from_urdf(xml):
    root = ET.fromstring(xml)
    physical = {
        j.get("name"): float(j.find("limit").get("effort"))
        for j in root.findall("joint")
    }
    limits = []
    for name in JOINT_NAMES:
        joint = root.find(f"ros2_control/joint[@name='{name}']")
        interfaces = joint.findall("command_interface")
        if len(interfaces) != 1 or interfaces[0].get("name") != "effort":
            raise ValueError(f"{name}: expected only an effort command interface")
        values = {p.get("name"): float(p.text) for p in interfaces[0].findall("param")}
        limit = values["max"]
        if (
            not math.isfinite(limit)
            or not 0 < limit <= physical[name]
            or values["min"] != -limit
        ):
            raise ValueError(f"{name}: invalid symmetric effort limits")
        limits.append(limit)
    return limits


def posture_command(target, position, velocity, feedforward, kp, kd, limits):
    """Return individual torque terms and the bounded six-joint command."""
    arrays = (target, position, velocity, feedforward, kp, kd, limits)
    if any(len(values) != 6 for values in arrays):
        raise ValueError("Expected six values for each joint vector")
    if any(not math.isfinite(x) for values in arrays for x in values):
        raise ValueError("Joint vectors must contain finite numbers")
    if any(x < 0 for x in (*kp, *kd)):
        raise ValueError("PD gains must be nonnegative")
    error = [goal - actual for goal, actual in zip(target, position)]
    proportional = [gain * e for gain, e in zip(kp, error)]
    damping = [-gain * speed for gain, speed in zip(kd, velocity)]
    requested = [ff + p + d for ff, p, d in zip(feedforward, proportional, damping)]
    command = [clamp_torque(value, limit) for value, limit in zip(requested, limits)]
    return dict(
        error=error,
        feedforward=list(feedforward),
        proportional=proportional,
        damping=damping,
        requested=requested,
        command=command,
    )
