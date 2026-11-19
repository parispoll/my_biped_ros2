"""Offline URDF tree/FK checks for the separate roll-joint prototype; no ROS."""

import math
import xml.etree.ElementTree as ET


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)
    ]


def rotation(axis, angle):
    length = math.sqrt(sum(v * v for v in axis))
    if not math.isfinite(angle) or length <= 0:
        raise ValueError("Finite rotation and nonzero axis required")
    x, y, z = [v / length for v in axis]
    c = math.cos(angle)
    s = math.sin(angle)
    d = 1 - c
    r = identity()
    r[:3] = [
        [c + x * x * d, x * y * d - z * s, x * z * d + y * s, 0],
        [y * x * d + z * s, c + y * y * d, y * z * d - x * s, 0],
        [z * x * d - y * s, z * y * d + x * s, c + z * z * d, 0],
    ]
    return r


def vector(element, key, default="0 0 0"):
    return tuple(
        float(v)
        for v in (element.get(key, default) if element is not None else default).split()
    )


def origin(element):
    xyz = vector(element, "xyz")
    r, p, y = vector(element, "rpy")
    result = multiply(
        rotation((0, 0, 1), y), multiply(rotation((0, 1, 0), p), rotation((1, 0, 0), r))
    )
    for i, v in enumerate(xyz):
        result[i][3] = v
    return result


def link_transforms(xml, angles=None):
    root = ET.fromstring(xml)
    angles = angles or {}
    joints = root.findall("joint")
    names = {j.get("name") for j in joints}
    if set(angles) - names:
        raise ValueError("Unknown joint target")
    child_names = {j.find("child").get("link") for j in joints}
    roots = {l.get("name") for l in root.findall("link")} - child_names
    if len(roots) != 1:
        raise ValueError("Expected one tree root")
    result = {roots.pop(): identity()}
    pending = list(joints)
    while pending:
        progressed = False
        for j in list(pending):
            parent = j.find("parent").get("link")
            child = j.find("child").get("link")
            if parent not in result:
                continue
            if child in result:
                raise ValueError("Repeated child/cyclic tree")
            q = angles.get(j.get("name"), 0.0)
            if not math.isfinite(q):
                raise ValueError("Finite joint target required")
            pose = origin(j.find("origin"))
            if j.get("type") == "revolute":
                limit = j.find("limit")
                if not float(limit.get("lower")) <= q <= float(limit.get("upper")):
                    raise ValueError("Joint target outside limits")
                pose = multiply(
                    pose, rotation(vector(j.find("axis"), "xyz", "1 0 0"), q)
                )
            elif j.get("type") != "fixed":
                raise ValueError("Only fixed/revolute FK supported")
            result[child] = multiply(result[parent], pose)
            pending.remove(j)
            progressed = True
        if not progressed:
            raise ValueError("Disconnected/cyclic tree")
    if len(result) != len(root.findall("link")):
        raise ValueError("Unconnected link")
    return result


def model_com(xml, angles=None):
    transforms = link_transforms(xml, angles)
    total = 0.0
    weighted = [0.0] * 3
    for link in ET.fromstring(xml).findall("link"):
        inertial = link.find("inertial")
        mass = float(inertial.find("mass").get("value"))
        pose = multiply(transforms[link.get("name")], origin(inertial.find("origin")))
        total += mass
        for i in range(3):
            weighted[i] += mass * pose[i][3]
    return total, [v / total for v in weighted]


def validate_roll_model(xml):
    root = ET.fromstring(xml)
    joints = root.findall("joint")
    links = root.findall("link")
    expected = {
        f"{s}_{j}"
        for s in ["left", "right"]
        for j in ["hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch", "ankle_roll"]
    }
    if len(joints) != 10 or {j.get("name") for j in joints} != expected:
        raise ValueError("Expected exactly ten named leg joints")
    if len(links) != 11 or len({l.get("name") for l in links}) != 11:
        raise ValueError("Expected eleven unique links")
    for j in joints:
        if j.get("type") != "revolute":
            raise ValueError("All ten joints must be revolute")
        expected_axis = (
            (1.0, 0.0, 0.0) if j.get("name").endswith("_roll") else (0.0, 1.0, 0.0)
        )
        if vector(j.find("axis"), "xyz") != expected_axis:
            raise ValueError("Incorrect pitch/roll axis")
        limit = j.find("limit")
        values = [float(limit.get(k)) for k in ["lower", "upper", "effort", "velocity"]]
        if (
            not all(math.isfinite(v) for v in values)
            or not values[0] <= 0 <= values[1]
            or values[2] <= 0
            or values[3] <= 0
        ):
            raise ValueError("Invalid joint limits")
    for l in links:
        inertial = l.find("inertial")
        m = float(inertial.find("mass").get("value"))
        e = inertial.find("inertia")
        diag = [float(e.get(k)) for k in ["ixx", "iyy", "izz"]]
        if not all(math.isfinite(v) and v > 0 for v in [m, *diag]):
            raise ValueError("Mass/inertia must be finite and positive")
        if any(float(e.get(k)) != 0 for k in ["ixy", "ixz", "iyz"]):
            raise ValueError("This prototype expects principal-axis inertias")
        if any(v > sum(diag) - v + 1e-12 for v in diag):
            raise ValueError("Unphysical inertia triangle")
    transforms = link_transforms(xml)
    mass, com = model_com(xml)
    if root.find("ros2_control") is not None:
        raise ValueError("Geometry prototype must not attach unverified controllers")
    return dict(
        joints=len(joints),
        links=len(links),
        total_mass_kg=mass,
        neutral_com_from_base_m=com,
        inertia_checks_passed=True,
        tree_connected=True,
        controller_attached=False,
    )
