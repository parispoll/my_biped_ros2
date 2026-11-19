"""Planar static checks for a symmetric pair of ankle axes, independent of ROS."""

import math
import xml.etree.ElementTree as ET
from my_robot_description.model.stance import model_from_urdf, vector


def ankle_gravity(xml, positions, pelvis_pitch, gravity=9.81):
    """Estimate per-ankle motor torque balancing the bodies ABOVE both ankles.

    Assumes symmetric joint angles, collinear ankle axes, sagittal motion and no
    acceleration/external wrench. Foot contact pressure is not assumed centred:
    feet are excluded by cutting through both ankle joints. This predicts a
    gravity load, not a measured actuator torque or a friction measurement.
    """
    model = model_from_urdf(xml)
    if (
        len(positions) != 6
        or not all(math.isfinite(x) for x in (*positions, pelvis_pitch, gravity))
        or gravity <= 0
    ):
        raise ValueError(
            "Expected six finite joint positions, finite pitch and positive gravity"
        )
    if any(abs(a - b) > 1e-6 for a, b in zip(positions[:3], positions[3:])):
        raise ValueError("Gravity split requires symmetric leg positions")
    hip, knee, ankle = positions[:3]
    if abs(pelvis_pitch + hip + knee + ankle) > 1e-4:
        raise ValueError("Planar diagnosis requires approximately flat feet")
    root = ET.fromstring(xml)
    base_z = vector(root.find("link[@name='base_link']/inertial/origin"), "xyz")[2]
    knee_x = -model.thigh * math.sin(pelvis_pitch + hip)
    ankle_x = knee_x - model.shin * math.sin(pelvis_pitch + hip + knee)
    # Coordinates relative to the common fore-aft ankle location; Z is above it.
    pelvis_z = model.thigh * math.cos(pelvis_pitch + hip) + model.shin * math.cos(
        pelvis_pitch + hip + knee
    )
    bodies = {
        "pelvis": (
            model.base_mass,
            base_z * math.sin(pelvis_pitch) - ankle_x,
            pelvis_z + base_z * math.cos(pelvis_pitch),
        ),
        "both_thighs": (
            2 * model.thigh_mass,
            knee_x / 2 - ankle_x,
            pelvis_z - model.thigh / 2 * math.cos(pelvis_pitch + hip),
        ),
        "both_shins": (
            2 * model.shin_mass,
            (knee_x - ankle_x) / 2,
            model.shin / 2 * math.cos(pelvis_pitch + hip + knee),
        ),
    }
    moments = {name: mass * gravity * x / 2 for name, (mass, x, z) in bodies.items()}
    stiffness = sum(mass * gravity * z / 2 for mass, x, z in bodies.values())
    return {
        "gravity_torque_per_ankle_nm": sum(moments.values()),
        "gravity_terms_per_ankle_nm": moments,
        "local_gravity_slope_per_ankle_nm_per_rad": stiffness,
        "upper_body_com_x_from_ankle_m": sum(m * x for m, x, z in bodies.values())
        / sum(m for m, x, z in bodies.values()),
        "upper_body_potential_energy_j": sum(
            m * gravity * z for m, x, z in bodies.values()
        ),
        "foot_pitch_rad": pelvis_pitch + hip + knee + ankle,
    }
