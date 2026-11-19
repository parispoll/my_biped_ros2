"""Ten-joint symmetric standing geometry and static loads, independent of ROS."""

import math
import xml.etree.ElementTree as ET
from my_robot_description.model.roll_geometry import (
    link_transforms,
    model_com,
    origin,
    multiply,
    vector,
)

JOINT_NAMES = [
    f"{s}_{j}"
    for s in ["left", "right"]
    for j in ["hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"]
]
from my_robot_description.model.roll_actuators import MOTOR_CAPS


def standing_plan(xml, knee=0.4):
    """Center all eleven links' COM over symmetric flat feet at zero roll.

    Static assumption: upright pelvis, equal vertical reactions at sole centers,
    no horizontal ground force, no acceleration, and no ground contact moment.
    """
    if not math.isfinite(knee) or not 0.1 <= knee <= 0.6:
        raise ValueError("Standing knee must be 0.1–0.6 rad")
    root = ET.fromstring(xml)
    joints = root.findall("joint")
    if {j.get("name") for j in joints} != set(JOINT_NAMES):
        raise ValueError("Expected the ten-joint roll model")

    def pose(hip):
        return {
            f"{s}_{j}": q
            for s in ["left", "right"]
            for j, q in zip(
                ["hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"],
                [0.0, hip, knee, -hip - knee, 0.0],
            )
        }

    def offset(hip):
        q = pose(hip)
        return model_com(xml, q)[1][0] - link_transforms(xml, q)["left_foot"][0][3]

    low, high = -0.8, 0.1
    if offset(low) * offset(high) >= 0:
        raise ValueError("No bracketed centered standing pose")
    for _ in range(60):
        mid = (low + high) / 2
        if offset(low) * offset(mid) <= 0:
            high = mid
        else:
            low = mid
    q = pose((low + high) / 2)
    transforms = link_transforms(xml, q)
    mass, com = model_com(xml, q)
    loads = double_support_loads(
        xml, q, {"left": mass * 9.81 / 2, "right": mass * 9.81 / 2}
    )
    thickness = float(
        root.find("link[@name='left_foot']/collision/geometry/box")
        .get("size")
        .split()[2]
    )
    height = -transforms["left_foot"][2][3] + thickness
    for name, cap in zip(JOINT_NAMES, MOTOR_CAPS):
        if abs(loads[name]) >= cap:
            raise ValueError(f"{name}: static load exceeds bounded motor budget")
    return dict(
        joint_names=JOINT_NAMES,
        target_rad=[q[n] for n in JOINT_NAMES],
        feedforward_nm=[loads[n] for n in JOINT_NAMES],
        motor_caps_nm=MOTOR_CAPS,
        mass_kg=mass,
        expected_pelvis_height_m=height,
        com_x_from_foot_m=com[0] - transforms["left_foot"][0][3],
        ground_force_per_foot_n=mass * 9.81 / 2,
        assumptions="Upright symmetric static double support; equal vertical reactions at foot centers. Not a dynamic balance controller.",
    )


def double_support_loads(xml, q, vertical_forces):
    """Static joint loads with specified vertical reactions at both sole centers.

    This is an equilibrium estimate, not measured contact force or a unique
    pressure solution for finite-area feet. q maps explicit joint names to angles.
    """
    root = ET.fromstring(xml)
    joints = root.findall("joint")
    transforms = link_transforms(xml, q)
    mass, _ = model_com(xml, q)
    if set(vertical_forces) != {"left", "right"} or any(
        not math.isfinite(v) or v < 0 for v in vertical_forces.values()
    ):
        raise ValueError("Nonnegative finite foot reactions required")
    if abs(sum(vertical_forces.values()) - mass * 9.81) > 1e-8:
        raise ValueError("Static vertical forces must balance weight")
    children = {l.get("name"): [] for l in root.findall("link")}
    for j in joints:
        children[j.find("parent").get("link")].append(j.find("child").get("link"))

    def distal(link):
        return [link, *[item for c in children[link] for item in distal(c)]]

    bodies = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        name = link.get("name")
        bodies[name] = (
            float(inertial.find("mass").get("value")),
            multiply(transforms[name], origin(inertial.find("origin"))),
        )
    cross = lambda r, f: (
        r[1] * f[2] - r[2] * f[1],
        r[2] * f[0] - r[0] * f[2],
        r[0] * f[1] - r[1] * f[0],
    )
    loads = {}
    for j in joints:
        name = j.get("name")
        side = name.split("_")[0]
        frame = multiply(
            transforms[j.find("parent").get("link")], origin(j.find("origin"))
        )
        axis = vector(j.find("axis"), "xyz")
        world_axis = [sum(frame[i][k] * axis[k] for k in range(3)) for i in range(3)]
        point = [frame[i][3] for i in range(3)]
        moment = [0.0] * 3
        for link in distal(j.find("child").get("link")):
            m, tf = bodies[link]
            r = [tf[i][3] - point[i] for i in range(3)]
            tor = cross(r, (0.0, 0.0, -m * 9.81))
            moment = [a + b for a, b in zip(moment, tor)]
        foot = transforms[f"{side}_foot"]
        r = [foot[i][3] - point[i] for i in range(3)]
        tor = cross(r, (0.0, 0.0, vertical_forces[side]))
        moment = [a + b for a, b in zip(moment, tor)]
        loads[name] = -sum(a * b for a, b in zip(world_axis, moment))
    return loads
