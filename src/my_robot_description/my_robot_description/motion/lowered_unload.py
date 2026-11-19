"""Separate positive-load targets at a lowered posture; never a foot-lift mode."""

import math
import xml.etree.ElementTree as ET
from scipy.optimize import root
from my_robot_description.motion.bilateral_lowering import LoweringProgram
from my_robot_description.motion.lowered_shift import LoweredShiftProgram
from my_robot_description.model.roll_geometry import link_transforms, model_com
from my_robot_description.model.roll_stance import JOINT_NAMES


class LoweredUnloadProgram(LoweredShiftProgram):
    lateral_duration = (
        64.0  # Same duration as verified 40 mm; checked reference speeds.
    )

    def validate_displacement(self, d):
        if not math.isfinite(d) or not 0 < abs(d) <= 0.13:
            raise ValueError("Unloading IK exceeded 130 mm diagnostic envelope")

    def validate_goal(self, target_fraction, direction):
        if target_fraction not in (0.2, 0.1, 0.05) or direction not in (
            "left",
            "right",
        ):
            raise ValueError(
                "Choose 20%, 10% or 5% remaining load and left/right support"
            )

    def __init__(self, xml, target_fraction=0.2, direction="left"):
        self.validate_goal(target_fraction, direction)
        self.minimum_predicted_share = target_fraction
        self.minimum_measured_share = round(target_fraction - 0.02, 6)
        sign = 1 if direction == "left" else -1
        lower = LoweringProgram(xml, 0.04)
        h = lower.table[-1]["pelvis_height_m"]
        goal = sign * (0.5 - target_fraction) * 0.17

        def angles(v):
            """Solve both flat-foot legs from world foot positions and pelvis X/Y in metres."""
            roll = math.atan2(-v[2], h - 0.045)
            return dict(zip(JOINT_NAMES, [roll, v[0], v[1], -v[0] - v[1], -roll] * 2))

        def residual(v):
            q = angles(v)
            tf = link_transforms(xml, q)
            _, com = model_com(xml, q)
            return [
                com[0] - tf["left_foot"][0][3],
                tf["left_foot"][2][3] + h - 0.045,
                com[1] + v[2] - goal,
            ]

        sol = root(residual, [-0.48, 0.8, goal / 0.645])
        if not sol.success or max(map(abs, residual(sol.x))) > 1e-9:
            raise ValueError("Unloading endpoint IK did not converge")
        super().__init__(xml, float(sol.x[2]))
        limits = {
            j.get("name"): (
                float(j.find("limit").get("lower")),
                float(j.find("limit").get("upper")),
            )
            for j in ET.fromstring(xml).findall("joint")
            if j.get("name") in JOINT_NAMES
        }
        for pose in self.table + self.lateral:
            if any(
                not limits[n][0] <= q <= limits[n][1]
                for n, q in zip(JOINT_NAMES, pose["target_rad"])
            ):
                raise ValueError("Unloading joint limit exceeded")
        foot_y = self.planner.feet[direction + "_foot"][1][3]
        margin = 0.06 - abs(self.lateral[-1]["predicted_com_y_m"] - foot_y)
        if margin < 0.01:
            raise ValueError("Endpoint COM lacks 10 mm lateral support-foot margin")
        self.setup.update(
            experiment_kind="lowered_unload",
            target_unloaded_fraction=target_fraction,
            unloaded_foot="right" if direction == "left" else "left",
            supporting_foot=direction,
            minimum_measured_share=self.minimum_measured_share,
            predicted_support_foot_lateral_margin_m=margin,
            note="Positive bilateral unloading; target-specific measured guard, no foot lift. Static COM margin is not dynamic stability proof.",
        )


def audit_unloading_goal(raw, com, start, program):
    from my_robot_description.sensing.foot_loads import paired_loads

    lo, hi = program.setup["analysis_windows"]["hold"]
    pairs = paired_loads(raw, start + lo, start + hi)
    side = program.setup["unloaded_foot"]
    fraction = sum(
        r[side]["force_world_n"][2] / r["total_vertical_n"] for r in pairs
    ) / len(pairs)
    held = [r for r in com if start + lo <= r["sim_time"] < start + hi]
    support = program.setup["supporting_foot"]
    margins = []
    for r in held:
        corners = r["feet"][support]
        x, y = r["com"][:2]
        # Both existing audits independently enforce flat, stationary soles.
        margins.append(
            min(
                x - min(c[0] for c in corners),
                max(c[0] for c in corners) - x,
                y - min(c[1] for c in corners),
                max(c[1] for c in corners) - y,
            )
        )
    margin = min(margins) if margins else None
    target = program.setup["target_unloaded_fraction"]
    return dict(
        passed=len(pairs) >= 20
        and len(held) >= 20
        and abs(fraction - target) <= 0.01
        and margin >= 0.01,
        target_fraction=target,
        measured_fraction=fraction,
        foot=side,
        force_samples=len(pairs),
        com_samples=len(held),
        minimum_held_support_foot_com_margin_m=margin,
        foot_lifted=False,
        note="Positive bilateral load and geometric COM projection; not demonstrated single support.",
    )
