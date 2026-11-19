#!/usr/bin/env python3
"""Offline educational P54 geometry audit. Does not command ROS or Gazebo."""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import os

os.environ["BIPED_ACTUATOR_PROFILE"] = "robotis_p54"
import sys, json, math, hashlib
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))
import numpy as np
import xacro
from scipy.optimize import minimize
from my_robot_description.studies.support_feasibility import SupportSearch
from my_robot_description.model.roll_geometry import (
    link_transforms,
    model_com,
    multiply,
    origin,
    vector,
)
from my_robot_description.model.roll_stance import JOINT_NAMES, MOTOR_CAPS
from my_robot_description.model.sagittal_step import leg_angles

OUT = ROOT / "output/pdf/geometry-calculations"


def calculate():
    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    root = ET.fromstring(xml)
    search = SupportSearch(xml)
    p = search.planner
    links = {l.get("name"): l for l in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}
    children = {n: [] for n in links}
    for j in joints.values():
        children[j.find("parent").get("link")].append(j.find("child").get("link"))

    def descendants(n):
        return [n, *[d for c in children[n] for d in descendants(c)]]

    distal = {n: descendants(j.find("child").get("link")) for n, j in joints.items()}
    height = p.base["expected_pelvis_height_m"]
    mass = p.base["mass_kg"]
    weight = mass * 9.81

    def single_pose(shift, lower, lift=0.0, advance=0.0):
        pose = search.pose(shift, lower)
        q = dict(pose["q"])
        if lift or advance:
            y = -shift
            z = p.ankle_z + lower + lift
            roll = math.atan2(y, -z)
            pitch = leg_angles(p.model, p.foot_x + advance, -math.hypot(y, z), 0.0)
            q.update(
                {
                    f"right_{n}": v
                    for n, v in zip(
                        [
                            "hip_roll",
                            "hip_pitch",
                            "knee_pitch",
                            "ankle_pitch",
                            "ankle_roll",
                        ],
                        [roll, *pitch, -roll],
                    )
                }
            )
        tf = link_transforms(xml, q)
        xyz = np.array([0.0, shift, height - lower])
        pos = {n: np.array(t)[:3, 3] + xyz for n, t in tf.items()}
        bodies = {
            n: (
                float(l.find("inertial/mass").get("value")),
                np.array(multiply(tf[n], origin(l.find("inertial/origin"))))[:3, 3]
                + xyz,
            )
            for n, l in links.items()
        }
        com = sum(m * b for m, b in bodies.values()) / mass
        foot = pos["left_foot"]
        cop = np.array([com[0], com[1], foot[2] - 0.045])
        offset = cop[:2] - foot[:2]
        reserve = np.array([0.1, 0.06]) - abs(offset)
        tau = {}
        terms = {}
        for n in JOINT_NAMES:
            j = joints[n]
            frame = np.array(
                multiply(tf[j.find("parent").get("link")], origin(j.find("origin")))
            )
            at = frame[:3, 3] + xyz
            axis = frame[:3, :3] @ np.array(vector(j.find("axis"), "xyz"))
            contributions = {
                child: -float(
                    axis
                    @ np.cross(bodies[child][1] - at, [0, 0, -bodies[child][0] * 9.81])
                )
                for child in distal[n]
            }
            ground = (
                -float(axis @ np.cross(cop - at, [0, 0, weight]))
                if "left_foot" in distal[n]
                else 0.0
            )
            tau[n] = sum(contributions.values()) + ground
            terms[n] = {
                "gravity_nm": sum(contributions.values()),
                "ground_nm": ground,
                "bodies_nm": contributions,
                "joint_world_m": at.tolist(),
                "axis_world": axis.tolist(),
            }
        # Cross-leg cylinder separation using conservative enclosing capsules.
        clear = []
        for left in ["left_thigh", "left_shin"]:
            for right in ["right_thigh", "right_shin"]:

                def segment(n):
                    t = np.array(tf[n])
                    end = t[:3, 3] + xyz
                    return end, end + t[:3, :3] @ np.array([0, 0, -0.26])

                a, b = segment(left)
                c, d = segment(right)
                res = minimize(
                    lambda u: float(
                        np.sum((a + u[0] * (b - a) - c - u[1] * (d - c)) ** 2)
                    ),
                    [0.5, 0.5],
                    bounds=[(0, 1), (0, 1)],
                    method="L-BFGS-B",
                )
                if not res.success:
                    raise RuntimeError(res.message)
                radius = (0.035 if left.endswith("thigh") else 0.03325) + (
                    0.035 if right.endswith("thigh") else 0.03325
                )
                clear.append(
                    {
                        "pair": [left, right],
                        "capsule_clearance_m": math.sqrt(max(0, res.fun)) - radius,
                    }
                )
        # Flat feet: interval separation is exact along world Y.
        footgap = abs(pos["left_foot"][1] - pos["right_foot"][1]) - 0.12
        util = max(abs(t) / c for t, c in zip(tau.values(), MOTOR_CAPS))
        return dict(
            shift_m=shift,
            lowering_m=lower,
            lift_m=lift,
            advance_m=advance,
            q=q,
            com_world_m=com.tolist(),
            cop_offset_m=offset.tolist(),
            sole_edge_reserve_m=reserve.tolist(),
            torques_nm=tau,
            terms=terms,
            peak_utilization=util,
            positions={n: v.tolist() for n, v in pos.items()},
            cross_leg_capsules=clear,
            foot_y_gap_m=footgap,
        )

    best = None
    interior = None
    feasible = 0
    for lower in np.linspace(0, 0.06, 13):
        for shift in np.linspace(0.035, 0.12, 35):
            try:
                r = single_pose(float(shift), float(lower))
            except ValueError:
                continue
            if (
                min(r["sole_edge_reserve_m"]) >= 0.01 - 1e-10
                and r["peak_utilization"] <= 0.8
            ):
                feasible += 1
                if best is None or r["peak_utilization"] < best["peak_utilization"]:
                    best = r
                if min(r["sole_edge_reserve_m"]) >= 0.02 and (
                    interior is None
                    or r["peak_utilization"] < interior["peak_utilization"]
                ):
                    interior = r
    if best is None:
        raise RuntimeError("No candidate found in sampled family")
    edge_best = best
    if interior is None:
        raise RuntimeError("No 20 mm interior candidate found")
    best = interior
    lifted = single_pose(best["shift_m"], best["lowering_m"], 0.02, 0.03)
    # A continuous geometric example: first lift vertically, then move forward
    # while raised, then lower. Last segment assumes pelvis/support fixed.
    path = []
    for u in np.linspace(0, 1, 301):
        smooth = lambda t: t**3 * (10 + t * (-15 + 6 * t))
        if u <= 1 / 3:
            lift = 0.02 * smooth(u * 3)
            advance = 0.0
        elif u <= 2 / 3:
            lift = 0.02
            advance = 0.03 * smooth(u * 3 - 1)
        else:
            lift = 0.02 * (1 - smooth(u * 3 - 2))
            advance = 0.03
        r = single_pose(best["shift_m"], best["lowering_m"], lift, advance)
        path.append(
            {
                k: r[k]
                for k in [
                    "q",
                    "sole_edge_reserve_m",
                    "peak_utilization",
                    "cross_leg_capsules",
                    "foot_y_gap_m",
                    "lift_m",
                    "advance_m",
                ]
            }
        )
    qarray = np.array([[r["q"][n] for n in JOINT_NAMES] for r in path])
    speeds = np.max(abs(np.gradient(qarray, 3.0 / 300, axis=0)), axis=0)
    # Fixed-height reach is a circle around a hip, before the straight-leg singularity.
    reach = math.sqrt(0.52**2 - p.ankle_z**2 - p.foot_x**2)
    boundary_com = model_com(
        xml, {n: v for n, v in zip(JOINT_NAMES, p.base["target_rad"])}
    )[1]
    scan = []
    for shift in [0, 0.052916782237778535, 0.07, reach - 1e-7]:
        try:
            r = search.pose(shift, 0)
            com_y = r["com"][1] + shift
            scan.append(
                dict(
                    shift_m=shift,
                    com_y_m=com_y,
                    center_pressure_right_share=0.5 - com_y / 0.17,
                    knee_rad=r["q"]["left_knee_pitch"],
                )
            )
        except ValueError as e:
            scan.append(dict(shift_m=shift, error=str(e)))
    result = dict(
        profile="robotis_p54",
        mass_kg=mass,
        weight_n=weight,
        base=p.base,
        ankle_z_m=p.ankle_z,
        foot_x_m=p.foot_x,
        baseline_com_world_m=[
            boundary_com[0],
            boundary_com[1],
            boundary_com[2] + height,
        ],
        links=[
            dict(
                name=n,
                mass_kg=float(l.find("inertial/mass").get("value")),
                com_local_m=list(vector(l.find("inertial/origin"), "xyz")),
            )
            for n, l in links.items()
        ],
        limits={
            n: {
                k: float(j.find("limit").get(k))
                for k in ["lower", "upper", "effort", "velocity"]
            }
            for n, j in joints.items()
        },
        fixed_height_extension_reserve_m=0.52 - math.hypot(p.ankle_z, p.foot_x),
        fixed_height_lateral_reach_m=reach,
        fixed_height_scan=scan,
        search={
            "shift_range_m": [0.035, 0.12],
            "shift_samples": 35,
            "lowering_range_m": [0, 0.06],
            "lowering_samples": 13,
            "accepted_static_candidates": feasible,
            "sole_margin_m": 0.01,
            "torque_utilization_limit": 0.8,
        },
        minimum_torque_10mm_candidate=edge_best,
        single_support_candidate=best,
        lifted_candidate=lifted,
        path={
            "samples": len(path),
            "assumed_duration_s": 3.0,
            "max_joint_speeds_rad_s": dict(zip(JOINT_NAMES, speeds.tolist())),
            "max_static_utilization": max(r["peak_utilization"] for r in path),
            "minimum_edge_reserve_m": min(min(r["sole_edge_reserve_m"]) for r in path),
            "minimum_cross_leg_capsule_clearance_m": min(
                c["capsule_clearance_m"] for r in path for c in r["cross_leg_capsules"]
            ),
            "minimum_foot_y_gap_m": min(r["foot_y_gap_m"] for r in path),
            "minimum_joint_limit_margin_rad": min(
                min(
                    r["q"][n] - float(joints[n].find("limit").get("lower")),
                    float(joints[n].find("limit").get("upper")) - r["q"][n],
                )
                for r in path
                for n in JOINT_NAMES
            ),
        },
        assumptions=[
            "Static gravity and vertical supporting reaction; no acceleration or push",
            "Pressure is allowed at COM projection inside supporting sole with 10 mm reserve",
            "No live single-support validation; zero-load foot remains touching ground in search",
            "Lift path uses fixed pelvis and flat soles; geometric samples, not commanded gait",
            "Capsule check covers cross-leg thigh/shin pairs only; not complete collision certification",
            "Motor cases add mass/inertia but have no faithful collision packaging",
        ],
        source_hashes={
            str(f.relative_to(ROOT)): hashlib.sha256(f.read_bytes()).hexdigest()
            for f in list((ROOT / "src/my_robot_description/urdf").rglob("*.xacro"))
            + [
                ROOT / "src/my_robot_description/config/roll_actuators.yaml",
                Path(__file__),
            ]
        },
    )
    # Independent numerical checks: force/moment balance and baseline solver agreement.
    reference = search.solve(search.pose(best["shift_m"], best["lowering_m"]), 0.0)
    assert reference is not None
    assert (
        np.max(
            abs(
                np.array(list(best["torques_nm"].values()))
                - np.array(list(reference["torques_nm"].values()))
            )
        )
        < 1e-8
    )
    assert min(lifted["sole_edge_reserve_m"]) >= 0.01
    assert result["path"]["max_static_utilization"] < 0.8
    assert result["path"]["minimum_cross_leg_capsule_clearance_m"] > 0
    assert result["path"]["minimum_joint_limit_margin_rad"] > 0

    # Independent gravity check via central differences of potential energy.
    def potential(q):
        trans = link_transforms(xml, q)
        return sum(
            float(l.find("inertial/mass").get("value"))
            * 9.81
            * multiply(trans[n], origin(l.find("inertial/origin")))[2][3]
            for n, l in links.items()
        )

    gravity_error = []
    for n in JOINT_NAMES:
        plus = dict(best["q"])
        minus = dict(best["q"])
        plus[n] += 1e-6
        minus[n] -= 1e-6
        estimate = (potential(plus) - potential(minus)) / 2e-6
        gravity_error.append(abs(estimate - best["terms"][n]["gravity_nm"]))
    assert max(gravity_error) < 1e-6
    result["verification"] = {
        "gravity_potential_difference_max_error_nm": max(gravity_error),
        "single_support_optimizer_torque_agreement_nm": float(
            np.max(
                abs(
                    np.array(list(best["torques_nm"].values()))
                    - np.array(list(reference["torques_nm"].values()))
                )
            )
        ),
        "sampled_path_checks_passed": True,
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "calculations.json").write_text(json.dumps(result, indent=2) + "\n")
    (OUT / "expanded-p54.urdf").write_text(xml)
    print(
        json.dumps(
            {
                k: result[k]
                for k in [
                    "mass_kg",
                    "fixed_height_extension_reserve_m",
                    "fixed_height_lateral_reach_m",
                    "fixed_height_scan",
                    "search",
                    "path",
                ]
            },
            indent=2,
        )
    )
    print(
        "Candidate",
        json.dumps(
            {
                k: best[k]
                for k in [
                    "shift_m",
                    "lowering_m",
                    "com_world_m",
                    "sole_edge_reserve_m",
                    "torques_nm",
                    "peak_utilization",
                ]
            },
            indent=2,
        ),
    )
    return result


if __name__ == "__main__":
    calculate()
