"""Planar standing geometry, independent of ROS execution.

Assumptions are checked against the expanded URDF: symmetric legs, pitch axes,
centred box feet, upright pelvis, and uniform centred thigh/shin masses.
"""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

JOINT_NAMES = [
    f"{side}_{joint}"
    for side in ("left", "right")
    for joint in ("hip", "knee", "ankle")
]


def vector(element, attribute, default="0 0 0"):
    return tuple(float(x) for x in element.get(attribute, default).split())


def close(actual, expected):
    return len(actual) == len(expected) and all(
        abs(a - b) < 1e-9 for a, b in zip(actual, expected)
    )


@dataclass(frozen=True)
class StandingModel:
    thigh: float
    shin: float
    foot_length: float
    foot_thickness: float
    base_mass: float
    thigh_mass: float
    shin_mass: float
    foot_mass: float
    limits: tuple

    @property
    def mass(self):
        return self.base_mass + 2 * (self.thigh_mass + self.shin_mass + self.foot_mass)

    def pose(self, knee):
        """Find hip/ankle angles giving flat feet and COM over foot centres."""
        if not math.isfinite(knee):
            raise ValueError("Knee angle must be finite")
        # COM_x - foot_x = C*sin(hip) + D*sin(hip+knee).
        a = self.thigh * (self.thigh_mass / 2 + self.shin_mass + self.foot_mass)
        b = self.shin * (self.shin_mass / 2 + self.foot_mass)
        c = self.thigh - 2 * a / self.mass
        d = self.shin - 2 * b / self.mass
        hip = math.atan2(-d * math.sin(knee), c + d * math.cos(knee))
        ankle = -hip - knee
        pose = (hip, knee, ankle)
        for name, angle, (lower, upper) in zip(
            ("hip", "knee", "ankle"), pose, self.limits
        ):
            if not lower <= angle <= upper:
                raise ValueError(f"{name} angle {angle:.4f} outside [{lower}, {upper}]")
        return pose

    def geometry(self, pose):
        """Forward kinematics and mass-weighted COM relative to pelvis origin."""
        hip, knee, ankle = pose
        if not all(math.isfinite(q) for q in pose) or abs(hip + knee + ankle) > 1e-8:
            raise ValueError(
                "Geometry prediction requires finite angles and flat feet with upright pelvis"
            )
        thigh_x = -self.thigh / 2 * math.sin(hip)
        knee_x = -self.thigh * math.sin(hip)
        shin_x = knee_x - self.shin / 2 * math.sin(hip + knee)
        foot_x = knee_x - self.shin * math.sin(hip + knee)
        com_x = (
            2
            * (
                self.thigh_mass * thigh_x
                + self.shin_mass * shin_x
                + self.foot_mass * foot_x
            )
            / self.mass
        )
        return {
            "foot_pitch": hip + knee + ankle,
            "foot_x_from_pelvis": foot_x,
            "com_x_from_pelvis": com_x,
            "com_offset_from_foot_center": com_x - foot_x,
            "expected_pelvis_height": self.thigh * math.cos(hip)
            + self.shin * math.cos(hip + knee)
            + self.foot_thickness,
            "fore_aft_static_margin": self.foot_length / 2 - abs(com_x - foot_x),
        }


def model_from_urdf(xml):
    """Read physical values from the model; reject unsupported geometry changes."""
    root = ET.fromstring(xml)
    links = {e.get("name"): e for e in root.findall("link")}
    joints = {e.get("name"): e for e in root.findall("joint")}

    def require(condition, message):
        if not condition:
            raise ValueError(f"[stance model] {message}")

    def mass(link):
        value = float(links[link].find("inertial/mass").get("value"))
        require(math.isfinite(value) and value > 0, f"{link}: mass must be positive")
        return value

    require(
        set(links)
        == {
            "base_link",
            *[
                f"{side}_{part}"
                for side in ("left", "right")
                for part in ("thigh", "shin", "foot")
            ],
        },
        "Extra or missing links require extending the COM model",
    )
    require(
        set(joints) == set(JOINT_NAMES),
        "Extra joints require extending the standing model",
    )
    base_origin = links["base_link"].find("inertial/origin")
    require(
        close(vector(base_origin, "xyz")[:2], (0, 0)),
        "Base COM must lie on the vertical centreline",
    )
    models = []
    for side in ("left", "right"):
        hip, knee, ankle = [
            joints[f"{side}_{name}"] for name in ("hip", "knee", "ankle")
        ]
        expected = [
            ("base_link", f"{side}_thigh"),
            (f"{side}_thigh", f"{side}_shin"),
            (f"{side}_shin", f"{side}_foot"),
        ]
        for joint, (parent, child) in zip((hip, knee, ankle), expected):
            require(
                joint.get("type") == "revolute",
                "Only revolute leg joints are supported",
            )
            require(
                joint.find("parent").get("link") == parent
                and joint.find("child").get("link") == child,
                "Unexpected leg connection",
            )
            require(
                close(vector(joint.find("axis"), "xyz"), (0, 1, 0)),
                "All leg axes must point along +Y",
            )
            require(
                close(vector(joint.find("origin"), "rpy"), (0, 0, 0)),
                "Rotated joint origins are unsupported",
            )
        require(
            close(vector(hip.find("origin"), "xyz")[::2], (0, 0)),
            "Hip must have zero X/Z offset",
        )
        lengths = []
        for joint in (knee, ankle):
            origin = vector(joint.find("origin"), "xyz")
            require(
                close(origin[:2], (0, 0)) and origin[2] < 0,
                "Leg segments must extend down local Z",
            )
            lengths.append(-origin[2])
        for part, length in zip(("thigh", "shin"), lengths):
            origin = links[f"{side}_{part}"].find("inertial/origin")
            require(
                close(vector(origin, "xyz"), (0, 0, -length / 2)),
                "Leg COM must be at segment midpoint",
            )
        foot = links[f"{side}_foot"]
        dimensions = vector(foot.find("collision/geometry/box"), "size")
        require(
            all(math.isfinite(x) and x > 0 for x in dimensions),
            "Foot dimensions must be positive",
        )
        for origin in (foot.find("collision/origin"), foot.find("inertial/origin")):
            require(
                close(vector(origin, "xyz"), (0, 0, -dimensions[2] / 2)),
                "Foot must be centred below ankle",
            )
            require(
                close(vector(origin, "rpy"), (0, 0, 0)),
                "Foot frames must not be rotated",
            )
        limits = tuple(
            (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
            for j in (hip, knee, ankle)
        )
        models.append(
            StandingModel(
                *lengths,
                dimensions[0],
                dimensions[2],
                mass("base_link"),
                mass(f"{side}_thigh"),
                mass(f"{side}_shin"),
                mass(f"{side}_foot"),
                limits,
            )
        )
    require(
        models[0] == models[1],
        "Both legs must have identical dimensions, masses and limits",
    )
    offsets = [
        vector(joints[f"{s}_hip"].find("origin"), "xyz")[1] for s in ("left", "right")
    ]
    require(
        offsets[0] > 0 and abs(sum(offsets)) < 1e-9, "Hip offsets must be symmetric"
    )
    return models[0]
