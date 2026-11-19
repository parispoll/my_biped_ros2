"""Finite slow alternating gait: right/left/right placements and a stable stop.

Reuses the verified first step, then transfers from each actual new stance.
No resetting feet, teleportation, attachment, or changes to motor/physics limits.
"""

import math
import copy
from scipy.optimize import root
import numpy as np
from my_robot_description.motion.first_step import FirstStepProgram, smooth, interpolate
from my_robot_description.motion.zero_load import zero_load_guard
from my_robot_description.model.roll_geometry import (
    link_transforms,
    model_com,
    multiply,
    origin,
    vector,
)
from my_robot_description.model.roll_stance import (
    JOINT_NAMES,
    MOTOR_CAPS,
    double_support_loads,
)
from my_robot_description.control.roll_posture import KP, KD
from my_robot_description.motion.live_foot_lift import collision_check


class AlternatingWalkProgram(FirstStepProgram):
    def __init__(self, xml, direction="left", step_count=3, step_length=0.05):
        if type(step_count) is not int or not 3 <= step_count <= 20:
            raise ValueError("Choose 3 to 20 walking steps")
        self.step_count = step_count
        self.step_length = step_length
        super().__init__(xml, direction, advance=step_length)
        self.segments = []
        first = dict(
            index=1,
            support=self.support,
            swing=self.swing,
            lift_start=117.0,
            advance_start=132.0,
            lower_start=152.0,
            recontact_start=167.0,
            reload_start=169.0,
            hold_start=233.0,
            end=241.0,
            start_feet={s: list(p) for s, p in self.feet.items()},
            end_feet={
                s: [p[0] + (self.step_length if s == self.swing else 0), p[1], p[2]]
                for s, p in self.feet.items()
            },
        )
        self.steps = [first]
        feet = {s: list(p) for s, p in first["end_feet"].items()}
        previous = self.paths["reload"][-1]
        clock = 241.0
        peaks = [self.setup["sampled_peak_reference_speed_rad_s"]]
        # The trailing foot starts 50 mm behind support and lands 50 mm ahead:
        # it therefore travels 100 mm, while nominal step length stays 50 mm.
        templates = {}
        self.reused_cycles = 0
        for index in range(2, step_count + 1):
            # The foot that just landed becomes the next supporting foot.
            support = self.steps[-1]["swing"]
            swing = "left" if support == "right" else "right"
            start_feet = {s: list(p) for s, p in feet.items()}
            target_x = feet[support][0] + self.step_length
            initial_xy = [previous["pelvis_x_m"], previous["pelvis_y_m"]]
            _, com = model_com(xml, self.angles(feet, initial_xy))
            initial_com = [com[k] + initial_xy[k] for k in (0, 1)]
            step = dict(
                index=index,
                support=support,
                swing=swing,
                transfer_start=clock,
                lift_start=clock + 70,
                advance_start=clock + 85,
                lower_start=clock + 125,
                recontact_start=clock + 140,
                reload_start=clock + 142,
                hold_start=clock + 206,
                end=clock + 214,
                start_feet=start_feet,
            )
            phases = [
                ("transfer", 64.0),
                ("confirm_unloaded", 6.0),
                ("lift", 15.0),
                ("advance", 40.0),
                ("lower_foot", 15.0),
                ("recontact", 2.0),
                ("reload", 64.0),
                ("staggered_hold", 8.0),
            ]
            # Flat-ground dynamics are invariant to a common world-X translation.
            # Reuse a checked cycle only for the same support and relative stance.
            key = (support, round(feet[swing][0] - feet[support][0], 9))
            if key in templates:
                anchor, cached = templates[key]
                dx = feet[support][0] - anchor
                for old in cached:
                    table = []
                    for point in old["table"]:
                        point = copy.deepcopy(point)
                        for field in ("pelvis_x_m", "predicted_com_x_m"):
                            point[field] += dx
                        for side in ("left", "right"):
                            point[side + "_foot_xyz_m"][0] += dx
                        table.append(point)
                    duration = old["end"] - old["start"]
                    self.segments.append(
                        dict(
                            old,
                            table=table,
                            step=index,
                            start=clock,
                            end=clock + duration,
                        )
                    )
                    clock += duration
                previous = self.segments[-1]["table"][-1]
                feet = {
                    side: list(previous[side + "_foot_xyz_m"])
                    for side in ("left", "right")
                }
                step["end_feet"] = copy.deepcopy(feet)
                self.steps.append(step)
                self.reused_cycles += 1
                continue
            cycle_start = len(self.segments)
            lift_xy = None
            landing_com = None
            for name, duration in phases:
                table = []
                n = (
                    200
                    if name in ("lift", "advance", "lower_foot")
                    else 100 if name in ("transfer", "reload") else 1
                )
                for i in range(n + 1):
                    u = i / n
                    pose_feet = {s: list(p) for s, p in feet.items()}
                    if name == "transfer":
                        fraction = 0.5 * (1 - u)
                        target = [
                            initial_com[k] + (feet[support][k] - initial_com[k]) * u
                            for k in (0, 1)
                        ]
                        xy = None
                    elif name == "reload":
                        fraction = 0.5 * u
                        target = [
                            landing_com[k]
                            + (
                                (feet["left"][k] + feet["right"][k]) / 2
                                - landing_com[k]
                            )
                            * u
                            for k in (0, 1)
                        ]
                        xy = None
                    else:
                        fraction = 0.0 if name not in ("staggered_hold",) else 0.5
                        target = None
                        xy = (
                            lift_xy
                            if name != "staggered_hold"
                            else [previous["pelvis_x_m"], previous["pelvis_y_m"]]
                        )
                        if name == "lift":
                            pose_feet[swing][2] += 0.03 * u
                        elif name == "advance":
                            pose_feet[swing][0] += u * (target_x - pose_feet[swing][0])
                            pose_feet[swing][2] += 0.03
                        elif name == "lower_foot":
                            pose_feet[swing][0] = target_x
                            pose_feet[swing][2] += 0.03 * (1 - u)
                    table.append(
                        self.walk_pose(
                            pose_feet, support, fraction, target, xy, initial_xy
                        )
                    )
                if name == "transfer":
                    lift_xy = [table[-1]["pelvis_x_m"], table[-1]["pelvis_y_m"]]
                if name == "lower_foot":
                    feet[swing][0] = target_x
                    landing_com = [
                        table[-1]["predicted_com_x_m"],
                        table[-1]["predicted_com_y_m"],
                    ]
                if name == "reload":
                    previous = table[-1]
                speed = max(
                    abs(b["target_rad"][j] - a["target_rad"][j]) * n * 1.875 / duration
                    for a, b in zip(table, table[1:])
                    for j in range(10)
                )
                peaks.append(speed)
                if speed > 0.05:
                    raise ValueError(
                        f"Walking reference exceeds slow joint-speed bound: step {index} {name}: {speed}"
                    )
                self.segments.append(
                    dict(
                        step=index,
                        phase=name,
                        start=clock,
                        end=clock + duration,
                        table=table,
                        support=support,
                        swing=swing,
                    )
                )
                clock += duration
            step["end_feet"] = {s: list(p) for s, p in feet.items()}
            self.steps.append(step)
            templates[key] = (start_feet[support][0], self.segments[cycle_start:])
        self.total = clock
        self.setup.update(
            experiment_kind="alternating_walk",
            total_time_s=clock,
            steps=self.steps,
            step_count=step_count,
            clearance_m=0.03,
            nominal_step_length_m=self.step_length,
            subsequent_foot_travel_m=2 * self.step_length,
            translated_cycles_reused=self.reused_cycles,
            sampled_peak_reference_speed_rad_s=max(peaks),
            final_stance=previous,
            note=f"{step_count} alternating placements at {self.step_length*1000:g} mm nominal length; measured support exchange, landing and final stance required.",
        )
        # Reject joint/torque discontinuities at every new phase boundary.
        for segment in self.segments:
            a = self.sample(segment["start"] - 1e-7)
            b = self.sample(segment["start"])
            if (
                max(
                    abs(x - y)
                    for key in ("target_rad", "feedforward_nm")
                    for x, y in zip(a[key], b[key])
                )
                > 1e-6
            ):
                raise ValueError("Discontinuous alternating-walk boundary")

    def walk_pose(self, feet, support, fraction, target_com, xy, seed):
        """Solve pelvis/COM geometry and static torques for one proposed walking pose.

        The force split is a planning assumption; the live force sensors must confirm it.
        """
        if target_com is not None:

            def residual(v):
                _, c = model_com(self.xml, self.angles(feet, v))
                return [c[k] + v[k] - target_com[k] for k in (0, 1)]

            def jacobian(v):
                h = 1e-6
                out = np.empty((2, 2))
                for k in (0, 1):
                    a = np.array(v, dtype=float)
                    b = np.array(v, dtype=float)
                    a[k] += h
                    b[k] -= h
                    out[:, k] = (np.array(residual(a)) - np.array(residual(b))) / (
                        2 * h
                    )
                return out

            sol = root(residual, seed, jac=jacobian)
            if not sol.success or max(map(abs, residual(sol.x))) > 1e-9:
                raise ValueError("Walking COM/stance inverse kinematics failed")
            xy = list(sol.x)
        q = self.angles(feet, xy)
        tf = link_transforms(self.xml, q)
        mass, c = model_com(self.xml, q)
        swing = "left" if support == "right" else "right"
        forces = {support: mass * 9.81 * (1 - fraction), swing: mass * 9.81 * fraction}
        offset = [
            c[k] - sum(forces[s] * tf[s + "_foot"][k][3] for s in feet) / (mass * 9.81)
            for k in (0, 1)
        ]
        if abs(offset[0]) > 0.09 or abs(offset[1]) > 0.05:
            raise ValueError("Walking pressure projection lacks sole margin")
        loads = double_support_loads(self.xml, q, forces)
        for j in self.root.findall("joint"):
            name = j.get("name")
            side = name.split("_")[0]
            f = forces[side]
            frame = multiply(tf[j.find("parent").get("link")], origin(j.find("origin")))
            axis = vector(j.find("axis"), "xyz")
            delta = [offset[1] * f, -offset[0] * f, 0.0]
            loads[name] -= sum(
                sum(frame[i][k] * axis[k] for k in range(3)) * delta[i]
                for i in range(3)
            )
        if max(abs(loads[n]) / cap for n, cap in zip(JOINT_NAMES, MOTOR_CAPS)) > 0.8:
            raise ValueError("Walking static torque exceeds motor reserve")
        for s in feet:
            if (
                max(
                    abs(
                        tf[s + "_foot"][k][3]
                        + [xy[0], xy[1], self.height][k]
                        - feet[s][k]
                    )
                    for k in range(3)
                )
                > 1e-9
            ):
                raise ValueError("Walking world-foot fixation mismatch")
        collision_check(self.xml, q, self.height)
        return dict(
            target_rad=[q[n] for n in JOINT_NAMES],
            feedforward_nm=[loads[n] for n in JOINT_NAMES],
            pelvis_x_m=xy[0],
            pelvis_y_m=xy[1],
            pelvis_height_m=self.height,
            predicted_com_x_m=c[0] + xy[0],
            predicted_com_y_m=c[1] + xy[1],
            predicted_com_height_m=c[2] + self.height,
            predicted_force_n=forces,
            left_foot_xyz_m=list(feet["left"]),
            right_foot_xyz_m=list(feet["right"]),
        )

    def sample(self, elapsed):
        """Return targets for elapsed simulated seconds; angles are radians and torques N m.

        This samples a prechecked plan. It does not read sensors or publish commands."""
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("Finite nonnegative walking time required")
        if elapsed < 241.0 or not hasattr(self, "segments"):
            p = super().sample(elapsed)
            feet = {s: list(v) for s, v in self.feet.items()}
            feet[self.swing][0] += p["swing_advance_m"]
            feet[self.swing][2] += p["swing_clearance_m"]
            return dict(
                p,
                step_index=1,
                supporting_foot=self.support,
                swing_foot=self.swing,
                left_foot_xyz_m=feet["left"],
                right_foot_xyz_m=feet["right"],
            )
        for segment in self.segments:
            if elapsed < segment["end"]:
                p = interpolate(
                    segment["table"],
                    smooth(
                        (elapsed - segment["start"])
                        / (segment["end"] - segment["start"])
                    ),
                )
                return dict(
                    p,
                    phase=segment["phase"],
                    elapsed_s=elapsed,
                    step_index=segment["step"],
                    supporting_foot=segment["support"],
                    swing_foot=segment["swing"],
                )
        last = self.segments[-1]
        return dict(
            last["table"][-1],
            phase="done",
            elapsed_s=elapsed,
            step_index=self.step_count,
            supporting_foot=last["support"],
            swing_foot=last["swing"],
        )

    def expected_fraction(self, elapsed):
        """Return the planned unloaded/swing-foot vertical load fraction (0 to 0.5)."""
        p = self.sample(max(0.0, elapsed))
        return p["predicted_force_n"][p["swing_foot"]] / sum(
            p["predicted_force_n"].values()
        )

    def proportional_gains(self, elapsed):
        """Select joint stiffness gains for the supporting-foot role at this time."""
        p = self.sample(max(0.0, elapsed))
        g = list(KP)
        blend = self.gain_blend(elapsed)
        offset = 0 if p["supporting_foot"] == "left" else 5
        for i in (offset + 3, offset + 4):
            g[i] += (120.0 - KP[i]) * blend
        return g

    def derivative_gains(self, elapsed):
        """Select velocity damping gains for the current supporting and free ankles."""
        p = self.sample(max(0.0, elapsed))
        g = list(KD)
        blend = self.gain_blend(elapsed)
        offset = 0 if p["supporting_foot"] == "left" else 5
        free = 4 if p["swing_foot"] == "left" else 9
        g[free] += (0.3 - KD[free]) * blend
        for i in (offset + 3, offset + 4):
            g[i] += (8.0 - KD[i]) * blend
        return g

    def guard(self, states, now, elapsed, observation, *, touchdown=False):
        """Validate fresh measured contacts and sole geometry against the planned phase.

        A rejected guard stops motion; missing observations cannot be treated as success.
        """
        if elapsed < 241.0:
            return super().guard(states, now, elapsed, observation, touchdown=touchdown)
        p = self.sample(elapsed)
        support = p["supporting_foot"]
        swing = p["swing_foot"]
        fraction = self.expected_fraction(elapsed)
        result = zero_load_guard(
            states, now, self.planner.base["mass_kg"], fraction, support
        )
        if not result["eligible"]:
            return result

        def fail(reason):
            return dict(result, eligible=False, reason=reason)

        if observation is None or not -0.02 <= now - observation["sim_time"] <= 0.1:
            return fail("Missing/stale walking sole geometry")
        if abs(observation["mass_kg"] - self.planner.base["mass_kg"]) > 1e-8:
            return fail("Walking probe mass mismatch")
        if not all(
            math.isfinite(v)
            for v in [
                *observation["com"],
                *[v for cs in observation["feet"].values() for c in cs for v in c],
            ]
        ):
            return fail("Nonfinite walking geometry")
        for side in ("left", "right"):
            corners = observation["feet"][side]
            goal = p[side + "_foot_xyz_m"]
            if max(abs(c[2] - (goal[2] - 0.045)) for c in corners) > 0.0005:
                return fail("Walking sole height differs by over 0.5 mm")
            if (
                max(abs(sum(c[k] for c in corners) / 4 - goal[k]) for k in (0, 1))
                > 0.001
            ):
                return fail("Walking foot XY differs by over 1 mm")
        step = self.steps[p["step_index"] - 1]
        from .touchdown import contact_force_limit

        force_limit = contact_force_limit(
            p["phase"],
            p[swing + "_foot_xyz_m"][2] - 0.045,
            self.planner.base["mass_kg"],
            touchdown,
        )
        if step["lift_start"] - 2 <= elapsed < step["reload_start"]:
            if states[swing]["force_world_n"][2] > force_limit:
                return fail(
                    f"Walking swing/landing force exceeds phase limit {force_limit:g} N"
                )
            cs = observation["feet"][support]
            com = observation["com"]
            if (
                min(
                    *[com[k] - min(c[k] for c in cs) for k in (0, 1)],
                    *[max(c[k] for c in cs) - com[k] for k in (0, 1)],
                )
                < 0.01
            ):
                return fail("Walking supporting sole COM margin below 10 mm")
        if (
            p[swing + "_foot_xyz_m"][2] - 0.045 >= 0.0005
            and states[swing]["state"] != "no_contact"
        ):
            return fail("Walking raised foot lacks fresh no-contact evidence")
        if step["reload_start"] - 1 <= elapsed and states[swing]["state"] not in (
            "touching_unloaded",
            "loaded_contact",
        ):
            return fail("Walking landing contact missing before reload")
        return dict(
            result,
            reason="ok",
            geometry_stamp=observation["sim_time"],
            contact_force_limit_n=force_limit,
        )

    def sensor_contacts_ok(self, row, origin):
        """Check independent contact freshness using the planned support phase."""
        p = self.sample(max(0.0, row["sim_time"] - origin))
        values = row["contacts"]
        support = p["supporting_foot"]
        swing = p["swing_foot"]
        return (
            set(values) == {"left", "right"}
            and all(
                -0.02 <= row["sim_time"] - c["sim_time"] <= 0.05
                for c in values.values()
            )
            and values[support]["ground"]
            and (
                values[swing]["ground"]
                or self.expected_fraction(row["sim_time"] - origin) <= 0.01
            )
        )
