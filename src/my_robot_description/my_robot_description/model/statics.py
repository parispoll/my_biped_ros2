"""Static joint loads for symmetric, flat-footed, two-foot support.

This is an equilibrium estimate, not a balance controller or motor sizing tool.
All dimensions and masses come from the validated StandingModel.
"""

import math


def standing_loads(model, knee, gravity=9.81):
    """Balance moments on the bodies below each joint, about its +Y axis.

    Assume no acceleration, an upright pelvis, equal vertical ground forces,
    zero horizontal ground force, and pressure at each foot's centre.
    """
    if not math.isfinite(gravity) or gravity <= 0:
        raise ValueError("Gravity must be finite and positive")
    hip, knee, ankle = model.pose(knee)
    geometry = model.geometry((hip, knee, ankle))
    # Horizontal coordinates relative to the pelvis, in metres.
    knee_x = -model.thigh * math.sin(hip)
    ankle_x = geometry["foot_x_from_pelvis"]
    bodies = (
        (model.thigh_mass, knee_x / 2),
        (model.shin_mass, (knee_x + ankle_x) / 2),
        (model.foot_mass, ankle_x),
    )
    ground_force = model.mass * gravity / 2
    joints = {}
    for index, (name, joint_x) in enumerate(
        zip(("hip", "knee", "ankle"), (0.0, knee_x, ankle_x))
    ):
        # Moment_y = -distance_x * force_z for vertical forces.
        ground_moment = -(ankle_x - joint_x) * ground_force
        weight_moment = sum(
            (body_x - joint_x) * mass * gravity for mass, body_x in bodies[index:]
        )
        # Motor + ground + weight moments must add to zero.
        motor_torque = -(ground_moment + weight_moment)
        joints[name] = {
            "joint_x_from_pelvis_m": joint_x,
            "ground_moment_nm": ground_moment,
            "distal_weight_moment_nm": weight_moment,
            "motor_torque_nm": motor_torque,
        }
    return {
        "mass_kg": model.mass,
        "gravity_m_s2": gravity,
        "hip_knee_ankle_rad": [hip, knee, ankle],
        "vertical_ground_force_per_foot_n": ground_force,
        "pressure_center_x_from_pelvis_m": ankle_x,
        "per_leg": joints,
        "joint_torques_nm": {
            f"{side}_{name}": row["motor_torque_nm"]
            for side in ("left", "right")
            for name, row in joints.items()
        },
        "assumptions": [
            "Static symmetric stance; upright pelvis and flat feet",
            "Equal vertical load on both feet; zero horizontal ground force",
            "Ground pressure at each foot centre; no joint friction",
            "Equilibrium estimate only; no motion, push or stability prediction",
        ],
    }
