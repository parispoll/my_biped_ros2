"""40 mm lowered posture plus a small fixed-foot lateral translation.

Separate from single support: both feet remain planted and carry positive load.
"""

import math
from scipy.optimize import root
from my_robot_description.motion.bilateral_lowering import LoweringProgram
from my_robot_description.model.roll_geometry import link_transforms, model_com
from my_robot_description.model.roll_stance import (
    JOINT_NAMES,
    MOTOR_CAPS,
    double_support_loads,
)


class LoweredShiftProgram:
    minimum_predicted_share = 0.3
    lateral_duration = None

    def validate_displacement(self, displacement):
        if not math.isfinite(displacement) or not any(
            abs(abs(displacement) - d) < 1e-12 for d in (0.005, 0.01, 0.02, 0.03, 0.04)
        ):
            raise ValueError(
                "Lowered transfer supports exactly ±5, ±10, ±20, ±30 or ±40 mm"
            )

    def __init__(self, xml, displacement=0.005):
        self.validate_displacement(displacement)
        self.lower_program = LoweringProgram(xml, 0.04)
        self.planner = self.lower_program.planner
        self.lowering = 0.04
        self.displacement = displacement
        self.table = self.lower_program.table
        self.lateral = []
        height = self.table[-1]["pelvis_height_m"]
        seed = [self.table[-1]["target_rad"][1], self.table[-1]["target_rad"][2]]
        intervals = round(abs(displacement) / 0.000125)
        for i in range(intervals + 1):
            shift = displacement * (i / intervals)
            roll = math.atan2(-shift, height - 0.045)

            def angles(v):
                """Solve both flat-foot legs from world foot positions and pelvis X/Y in metres."""
                return dict(zip(JOINT_NAMES, [roll, v[0], v[1], -sum(v), -roll] * 2))

            def residual(v):
                q = angles(v)
                tf = link_transforms(xml, q)
                _, com = model_com(xml, q)
                return [
                    com[0] - tf["left_foot"][0][3],
                    tf["left_foot"][2][3] + height - 0.045,
                ]

            solution = root(residual, seed)
            if not solution.success or max(abs(v) for v in residual(solution.x)) > 1e-9:
                raise ValueError("Lowered lateral IK did not converge")
            seed = solution.x
            q = angles(seed)
            tf = link_transforms(xml, q)
            mass, com = model_com(xml, q)
            pelvis_x = self.planner.foot_x - tf["left_foot"][0][3]
            com_y = com[1] + shift
            forces = dict(
                left=mass * 9.81 * (0.5 + com_y / 0.17),
                right=mass * 9.81 * (0.5 - com_y / 0.17),
            )
            if (
                min(forces.values()) / (mass * 9.81)
                < self.minimum_predicted_share - 1e-10
            ):
                raise ValueError("Lowered plan exceeds bilateral load envelope")
            if self.minimum_predicted_share == 0 and min(forces.values()) < 0:
                # Solver roundoff at the exact zero endpoint only.
                if min(forces.values()) < -1e-8:
                    raise ValueError("Negative planned contact force")
                forces = {side: max(0.0, force) for side, force in forces.items()}
                total = sum(forces.values())
                forces = {
                    side: force * mass * 9.81 / total for side, force in forces.items()
                }
            loads = double_support_loads(xml, q, forces)
            if any(
                abs(loads[n]) > 0.8 * cap for n, cap in zip(JOINT_NAMES, MOTOR_CAPS)
            ):
                raise ValueError("Insufficient static motor headroom")
            for side in ["left", "right"]:
                foot = tf[side + "_foot"]
                old = self.planner.feet[side + "_foot"]
                if (
                    max(
                        abs(
                            foot[k][3]
                            + [
                                pelvis_x,
                                shift,
                                height - self.planner.base["expected_pelvis_height_m"],
                            ][k]
                            - old[k][3]
                        )
                        for k in range(3)
                    )
                    > 1e-9
                ):
                    raise ValueError("Foot fixation mismatch")
            self.lateral.append(
                dict(
                    target_rad=[q[n] for n in JOINT_NAMES],
                    feedforward_nm=[loads[n] for n in JOINT_NAMES],
                    pelvis_x_m=pelvis_x,
                    pelvis_y_m=shift,
                    pelvis_height_m=height,
                    predicted_com_y_m=com_y,
                    predicted_com_height_m=com[2] + height,
                    predicted_force_n=forces,
                )
            )
        lateral_time = self.lateral_duration or 8.0 * abs(displacement) / 0.005
        self.schedule = [
            ("baseline", 3.0),
            ("lower", 40.0),
            ("lowered_settle", 4.0),
            ("move_out", lateral_time),
            ("hold", 6.0),
            ("move_back", lateral_time),
            ("center_settle", 4.0),
            ("rise", 40.0),
            ("recovery", 8.0),
        ]
        total = sum(duration for _, duration in self.schedule)
        hold_end = 53.0 + lateral_time
        self.setup = dict(
            program_kind="lowered_shift",
            lowering_m=0.04,
            displacement_m=displacement,
            endpoint=self.lateral[-1],
            schedule=self.schedule,
            total_time_s=total,
            analysis_windows=dict(
                baseline=(1.0, 2.5),
                hold=(hold_end - 2.0, hold_end),
                recovery=(total - 3.0, total + 0.00001),
            ),
            note="Sequential lowering, selected bilateral lateral transfer, centering and rising; no force pulse or foot lift.",
        )

    def sample(self, elapsed):
        """Return targets for elapsed simulated seconds; angles are radians and torques N m.

        This samples a prechecked plan. It does not read sensors or publish commands."""
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("Nonnegative finite program time required")
        start = 0.0
        phase = "done"
        local = 0.0
        duration = 1.0
        for name, duration in self.schedule:
            if elapsed < start + duration:
                phase = name
                local = elapsed - start
                break
            start += duration
        if phase == "lower":
            result = self.lower_program.sample(3.0 + local)
        elif phase == "rise":
            result = self.lower_program.sample(49.0 + local)
        elif phase in ["baseline", "recovery", "done"]:
            result = dict(self.table[0])
        else:
            s = local / duration
            v = s**3 * (10 + s * (-15 + 6 * s))
            u = (
                v
                if phase == "move_out"
                else 1 - v if phase == "move_back" else 1.0 if phase == "hold" else 0.0
            )
            intervals = len(self.lateral) - 1
            point = intervals * u
            i = min(intervals - 1, int(point))
            f = point - i
            a, b = self.lateral[i], self.lateral[i + 1]
            result = {}
            for key in a:
                if isinstance(a[key], list):
                    result[key] = [x + (y - x) * f for x, y in zip(a[key], b[key])]
                elif isinstance(a[key], dict):
                    result[key] = {
                        s: a[key][s] + (b[key][s] - a[key][s]) * f for s in a[key]
                    }
                else:
                    result[key] = a[key] + (b[key] - a[key]) * f
        return dict(result, phase=phase, elapsed_s=elapsed)
