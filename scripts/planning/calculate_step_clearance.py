#!/usr/bin/env python3
"""Design and check a flat-ground 50 mm swing; never commands a robot.

30 mm is an engineering clearance choice, not a unique geometric requirement
for a perfectly flat plane. Reserve 20 mm at mid-swing, 5 deg pitch uncertainty
at a sole half-length, and the unchanged 0.5 mm vertical tracking tolerance.
"""

# Support both `python -m scripts.<folder>.<tool>` and direct file execution.
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

import argparse, json, math, os, sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/my_robot_description"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    os.environ["BIPED_ACTUATOR_PROFILE"] = "robotis_p54"
    import xacro
    from my_robot_description.motion.zero_load import ZeroLoadProgram
    from my_robot_description.model.roll_geometry import link_transforms, model_com
    from my_robot_description.model.roll_stance import JOINT_NAMES, MOTOR_CAPS
    from my_robot_description.model.sagittal_step import leg_angles
    from my_robot_description.motion.foot_lift import single_support_loads, sole_corners
    from my_robot_description.motion.live_foot_lift import collision_check
    import xml.etree.ElementTree as ET

    xml = xacro.process_file(
        str(ROOT / "src/my_robot_description/urdf/biped_roll.urdf.xacro")
    ).toxml()
    root = ET.fromstring(xml)
    length = float(
        root.find("link[@name='left_foot']/collision/geometry/box")
        .get("size")
        .split()[0]
    )
    reserve = 0.020
    pitch = math.radians(5)
    tolerance = 0.0005
    required = reserve + length / 2 * math.sin(pitch) + tolerance
    clearance = math.ceil(required / 0.005) * 0.005
    results = []
    for support in ["left", "right"]:
        zero = ZeroLoadProgram(xml, support)
        end = zero.setup["endpoint"]
        swing = "right" if support == "left" else "left"
        base = dict(zip(JOINT_NAMES, end["target_rad"]))
        tf0 = link_transforms(xml, base)
        foot = tf0[swing + "_foot"]
        hip_y = float(
            root.find(f"joint[@name='{swing}_hip_roll']/origin").get("xyz").split()[1]
        )
        poses = []
        for i in range(101):
            s = i / 100
            u = s**3 * (10 + s * (-15 + 6 * s))
            advance = 0.05 * u
            h = clearance * math.sin(math.pi * u) ** 2
            dy = foot[1][3] - hip_y
            z = foot[2][3] + h
            roll = math.atan2(dy, -z)
            pitchangles = leg_angles(
                zero.planner.model, foot[0][3] + advance, -math.hypot(dy, z), 0.0
            )
            q = dict(base)
            q.update(
                {
                    swing + "_" + n: v
                    for n, v in zip(
                        [
                            "hip_roll",
                            "hip_pitch",
                            "knee_pitch",
                            "ankle_pitch",
                            "ankle_roll",
                        ],
                        [roll, *pitchangles, -roll],
                    )
                }
            )
            tf = link_transforms(xml, q)
            loads = single_support_loads(xml, q, support)
            util = max(abs(t) / c for t, c in zip(loads["feedforward_nm"], MOTOR_CAPS))
            collision = collision_check(xml, q, end["pelvis_height_m"])
            error = max(
                abs(tf[swing + "_foot"][k][3] - foot[k][3] - ([advance, 0, h][k]))
                for k in range(3)
            )
            if util > 0.8 or error > 1e-9:
                raise ValueError("Swing exceeds existing geometry/motor envelope")
            poses.append(
                dict(
                    phase=s,
                    advance_m=advance,
                    height_m=h,
                    target_rad=[q[n] for n in JOINT_NAMES],
                    static_utilization=util,
                    support_margin_m=loads["support_margin_m"],
                    collision_gap_m=collision["minimum_separated_envelope_gap_m"],
                    fk_error_m=error,
                )
            )
        results.append(
            dict(
                support=support,
                passed=True,
                samples=poses,
                peak_static_utilization=max(r["static_utilization"] for r in poses),
                minimum_support_margin_m=min(r["support_margin_m"] for r in poses),
                minimum_collision_envelope_gap_m=min(
                    r["collision_gap_m"] for r in poses
                ),
            )
        )
    result = dict(
        passed=True,
        step_advance_m=0.05,
        mid_swing_clearance_reserve_m=reserve,
        assumed_pitch_uncertainty_deg=5,
        vertical_tracking_tolerance_m=tolerance,
        pitch_corner_allowance_m=length / 2 * math.sin(pitch),
        calculated_mid_swing_height_m=required,
        selected_vertical_test_height_m=clearance,
        scope="Offline fixed-pelvis 50 mm swing feasibility; no landing/body-transfer controller, no executed step. Pitch uncertainty is a design reserve, not a relaxed live guard.",
        results=results,
    )
    a.output.parent.mkdir(parents=True, exist_ok=True)
    with a.output.open("x") as f:
        json.dump(result, f, indent=2)
    print(json.dumps({k: v for k, v in result.items() if k != "results"}, indent=2))
    print(
        json.dumps(
            [{k: v for k, v in r.items() if k != "samples"} for r in results], indent=2
        )
    )


if __name__ == "__main__":
    main()
