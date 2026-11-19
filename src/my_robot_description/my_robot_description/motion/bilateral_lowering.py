"""First lowered-posture experiment: centered COM, flat planted feet, no lift."""

import math
from my_robot_description.motion.lateral_shift import LateralPlanner
from my_robot_description.model.roll_stance import (
    double_support_loads,
    JOINT_NAMES,
    MOTOR_CAPS,
)
from my_robot_description.model.roll_geometry import link_transforms, model_com
from my_robot_description.model.roll_actuators import ACTIVE


class LoweringProgram:
    def __init__(self, xml, lowering=0.01):
        if ACTIVE["name"] != "robotis_p54":
            raise ValueError("Lowering experiment requires P54 profile")
        if not math.isfinite(lowering) or not 0 < lowering <= 0.04:
            raise ValueError("Bilateral lowering supports 0 < lowering <= 40 mm")
        self.planner = LateralPlanner(xml)
        self.displacement = 0.0
        self.lowering = lowering
        move_time = max(10.0, 1000.0 * lowering)
        self.schedule = [
            ("baseline", 3.0),
            ("move_out", move_time),
            ("hold", 6.0),
            ("move_back", move_time),
            ("recovery", 8.0),
        ]
        self.table = []
        from scipy.optimize import root

        seed = [self.planner.base["target_rad"][1], 0.4]
        intervals = max(40, math.ceil(lowering / 0.00025))
        for i in range(intervals + 1):
            target = (
                self.planner.base["expected_pelvis_height_m"] - lowering * i / intervals
            )

            def angles(v):
                """Solve both flat-foot legs from world foot positions and pelvis X/Y in metres."""
                return dict(zip(JOINT_NAMES, [0.0, v[0], v[1], -sum(v), 0.0] * 2))

            def residual(v):
                q = angles(v)
                tf = link_transforms(xml, q)
                _, com = model_com(xml, q)
                return [
                    com[0] - tf["left_foot"][0][3],
                    -tf["left_foot"][2][3] + 0.045 - target,
                ]

            solution = root(residual, seed)
            if not solution.success or max(abs(v) for v in residual(solution.x)) > 1e-9:
                raise ValueError("Centered lowering IK did not converge")
            seed = solution.x
            q = angles(seed)
            tf = link_transforms(xml, q)
            mass, com = model_com(xml, q)
            loads = double_support_loads(
                xml, q, dict(left=mass * 9.81 / 2, right=mass * 9.81 / 2)
            )
            plan = dict(
                target_rad=[q[n] for n in JOINT_NAMES],
                feedforward_nm=[loads[n] for n in JOINT_NAMES],
                expected_pelvis_height_m=-tf["left_foot"][2][3] + 0.045,
            )
            pelvis_x = self.planner.foot_x - tf["left_foot"][0][3]
            assert abs(com[0] - tf["left_foot"][0][3]) < 1e-9
            assert abs(plan["expected_pelvis_height_m"] - target) < 1e-8
            if any(
                abs(t) > 0.8 * c for t, c in zip(plan["feedforward_nm"], MOTOR_CAPS)
            ):
                raise ValueError("Lowering consumes reserved motor headroom")
            self.table.append(
                dict(
                    target_rad=plan["target_rad"],
                    feedforward_nm=plan["feedforward_nm"],
                    pelvis_y_m=0.0,
                    pelvis_x_m=pelvis_x,
                    pelvis_height_m=plan["expected_pelvis_height_m"],
                    predicted_com_y_m=0.0,
                    predicted_com_height_m=com[2] + plan["expected_pelvis_height_m"],
                    predicted_force_n=dict(left=mass * 9.81 / 2, right=mass * 9.81 / 2),
                )
            )
        total = sum(t for _, t in self.schedule)
        hold_end = 9.0 + move_time
        self.setup = dict(
            program_kind="bilateral_lowering",
            lowering_m=lowering,
            endpoint=self.table[-1],
            schedule=self.schedule,
            total_time_s=total,
            analysis_windows=dict(
                baseline=(1.0, 2.5),
                hold=(hold_end - 2.0, hold_end),
                recovery=(total - 3.0, total + 0.00001),
            ),
            note="Symmetric centered-COM crouch; small fore/aft pelvis translation keeps both feet fixed. No lateral shift, unloading or lift.",
        )

    def sample(self, elapsed):
        """Return targets for elapsed simulated seconds; angles are radians and torques N m.

        This samples a prechecked plan. It does not read sensors or publish commands."""
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("Nonnegative finite time required")
        start = 0.0
        u = 0.0
        phase = "done"
        for name, duration in self.schedule:
            if elapsed < start + duration:
                phase = name
                s = (elapsed - start) / duration
                v = s**3 * (10 + s * (-15 + 6 * s))
                u = (
                    v
                    if name == "move_out"
                    else (
                        1 - v if name == "move_back" else 1.0 if name == "hold" else 0.0
                    )
                )
                break
            start += duration
        intervals = len(self.table) - 1
        point = u * intervals
        i = min(intervals - 1, int(point))
        f = point - i
        a, b = self.table[i], self.table[i + 1]
        result = dict(phase=phase, elapsed_s=elapsed)
        for key in a:
            if isinstance(a[key], list):
                result[key] = [x + (y - x) * f for x, y in zip(a[key], b[key])]
            elif isinstance(a[key], dict):
                result[key] = {
                    s: a[key][s] + (b[key][s] - a[key][s]) * f for s in a[key]
                }
            else:
                result[key] = a[key] + (b[key] - a[key]) * f
        return result
