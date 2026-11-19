"""Pendulum model and bounded joint torque law; no Gazebo/ROS dependencies."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET


def clamp_torque(request, limit):
    if not math.isfinite(request) or not math.isfinite(limit) or limit <= 0:
        raise ValueError("Torque and positive limit must be finite")
    return max(-limit, min(limit, request))


@dataclass(frozen=True)
class Pendulum:
    mass: float
    com_distance: float
    inertia: float
    damping: float
    effort_limit: float
    velocity_limit: float
    lower: float
    upper: float

    @property
    def gravity_gain(self):
        return self.mass * 9.81 * self.com_distance

    def gravity_torque(self, angle):
        return -self.gravity_gain * math.sin(angle)

    def requested_torque(self, angle, velocity, target, kp=12, kd=0.5):
        # Compensate the known gravity model, then add position/velocity feedback.
        return kp * (target - angle) - kd * velocity - self.gravity_torque(angle)

    def acceleration(self, angle, velocity, applied_torque):
        return (
            applied_torque + self.gravity_torque(angle) - self.damping * velocity
        ) / self.inertia


def pendulum_from_urdf(xml):
    root = ET.fromstring(xml)
    joint = root.find("joint[@name='bench_joint']")
    if joint.find("axis").get("xyz") != "0 1 0":
        raise ValueError("Bench model requires +Y joint axis")
    inertial = root.find("link[@name='bench_rod']/inertial")
    origin = [float(v) for v in inertial.find("origin").get("xyz").split()]
    if origin[:2] != [0, 0] or origin[2] >= 0:
        raise ValueError("Bench COM must lie below the hinge")
    mass = float(inertial.find("mass").get("value"))
    distance = -origin[2]
    inertia = float(inertial.find("inertia").get("iyy")) + mass * distance**2
    limits = joint.find("limit")
    return Pendulum(
        mass,
        distance,
        inertia,
        float(joint.find("dynamics").get("damping")),
        float(limits.get("effort")),
        float(limits.get("velocity")),
        float(limits.get("lower")),
        float(limits.get("upper")),
    )
