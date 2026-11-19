"""One slow forward placement and reload into a new staggered stance.

Separate from the verified vertical-lift/return experiment. No alternating gait.
Coordinates are relative to the original upright pelvis translation.
"""

import math
import xml.etree.ElementTree as ET
from scipy.optimize import root
from my_robot_description.motion.zero_load import ZeroLoadProgram, zero_load_guard
from my_robot_description.model.roll_geometry import (
    link_transforms,
    model_com,
    origin,
    multiply,
    vector,
)
from my_robot_description.model.roll_stance import (
    JOINT_NAMES,
    MOTOR_CAPS,
    double_support_loads,
)
from my_robot_description.model.sagittal_step import leg_angles
from my_robot_description.motion.live_foot_lift import collision_check


def smooth(s):
    s = max(0.0, min(1.0, s))
    return s**3 * (10 + s * (-15 + 6 * s))


def interpolate(table, u):
    p = max(0.0, min(1.0, u)) * (len(table) - 1)
    i = min(len(table) - 2, int(p))
    f = p - i
    a, b = table[i : i + 2]
    out = {}
    for k, x in a.items():
        if isinstance(x, list):
            out[k] = [v + (w - v) * f for v, w in zip(x, b[k])]
        elif isinstance(x, dict):
            out[k] = {s: v + (b[k][s] - v) * f for s, v in x.items()}
        else:
            out[k] = x + (b[k] - x) * f
    return out


class FirstStepProgram(ZeroLoadProgram):
    advance_m = 0.05
    clearance_m = 0.03
    insert_end = 169.0

    # Verified preparation is retained; the old fixed-feet return is replaced.
    def __init__(self, xml, direction="left", advance=0.05):
        if advance not in (0.05, 0.075, 0.1):
            raise ValueError("Step length must be 50, 75 or 100 mm")
        self.advance_m = advance
        super().__init__(xml, direction)
        self.xml = xml
        self.root = ET.fromstring(xml)
        self.support = direction
        self.swing = self.setup["unloaded_foot"]
        self.initial = dict(self.setup["endpoint"])
        self.initial_xy = [self.initial["pelvis_x_m"], self.displacement]
        self.height = self.initial["pelvis_height_m"]
        self.feet = {
            s: [
                self.planner.feet[s + "_foot"][0][3],
                self.planner.feet[s + "_foot"][1][3],
                0.045,
            ]
            for s in ("left", "right")
        }
        self.paths = {}
        for name in ("lift", "advance", "lower_foot", "reload"):
            points = []
            for i in range(201):
                u = i / 200
                x = (
                    self.advance_m * u
                    if name == "advance"
                    else self.advance_m if name in ("lower_foot", "reload") else 0.0
                )
                z = (
                    self.clearance_m * u
                    if name == "lift"
                    else (
                        self.clearance_m
                        if name == "advance"
                        else self.clearance_m * (1 - u) if name == "lower_foot" else 0.0
                    )
                )
                points.append(self.pose(x, z, u if name == "reload" else 0.0))
            self.paths[name] = points
        continuity = max(
            abs(a - b)
            for a, b in zip(
                self.paths["lift"][0]["feedforward_nm"], self.initial["feedforward_nm"]
            )
        )
        if continuity > 1e-7:
            raise ValueError("Discontinuous zero-load to first-step feedforward")
        peak_speed = max(
            max(abs(b["target_rad"][j] - a["target_rad"][j]) for j in range(10))
            * 200
            * 1.875
            / duration
            for name, duration in [
                ("lift", 15),
                ("advance", 20),
                ("lower_foot", 15),
                ("reload", 64),
            ]
            for a, b in zip(self.paths[name], self.paths[name][1:])
        )
        if peak_speed > 0.05:
            raise ValueError(
                "First-step reference exceeds 0.05 rad/s slow-motion bound"
            )
        self.setup.update(
            experiment_kind="first_step",
            total_time_s=241.0,
            step_advance_m=self.advance_m,
            clearance_m=0.03,
            schedule=[
                *self.schedule[:4],
                ("confirm_unloaded", 6.0),
                ("lift", 15.0),
                ("advance", 20.0),
                ("lower_foot", 15.0),
                ("recontact", 2.0),
                ("reload", 64.0),
                ("staggered_hold", 8.0),
            ],
            sampled_peak_reference_speed_rad_s=peak_speed,
            analysis_windows=dict(
                baseline=(1.0, 2.5), hold=(133.0, 151.0), recovery=(238.0, 241.00001)
            ),
            final_stance=self.paths["reload"][-1],
            note=f"One {self.advance_m*1000:g} mm forward placement, 30 mm clearance, landing and 50/50 reload; stays crouched in staggered stance. No old-stance return or alternating walking.",
        )

    def angles(self, feet, xy):
        """Solve both flat-foot legs from world foot positions and pelvis X/Y in metres."""
        q = {}
        for side in ("left", "right"):
            hip = vector(
                self.root.find(f"joint[@name='{side}_hip_roll']/origin"), "xyz"
            )
            x = feet[side][0] - xy[0]
            dy = feet[side][1] - xy[1] - hip[1]
            z = feet[side][2] - self.height
            roll = math.atan2(dy, -z)
            pitches = leg_angles(self.planner.model, x, -math.hypot(dy, z), 0.0)
            q.update(
                {
                    side + "_" + n: v
                    for n, v in zip(
                        (
                            "hip_roll",
                            "hip_pitch",
                            "knee_pitch",
                            "ankle_pitch",
                            "ankle_roll",
                        ),
                        [roll, *pitches, -roll],
                    )
                }
            )
        return q

    def pose(self, advance, height, reload):
        feet = {s: list(v) for s, v in self.feet.items()}
        feet[self.swing][0] += advance
        feet[self.swing][2] += height
        xy = list(self.initial_xy)
        if reload:
            landed = self.angles(feet, self.initial_xy)
            _, c = model_com(self.xml, landed)
            c0 = [c[k] + self.initial_xy[k] for k in (0, 1)]
            target = [
                (1 - reload) * c0[k] + reload * (feet["left"][k] + feet["right"][k]) / 2
                for k in (0, 1)
            ]

            def residual(v):
                _, c = model_com(self.xml, self.angles(feet, v))
                return [c[k] + v[k] - target[k] for k in (0, 1)]

            sol = root(residual, self.initial_xy)
            if not sol.success or max(map(abs, residual(sol.x))) > 1e-9:
                raise ValueError("Staggered-stance COM solve did not converge")
            xy = list(sol.x)
        q = self.angles(feet, xy)
        tf = link_transforms(self.xml, q)
        mass, c = model_com(self.xml, q)
        # Both CoPs share a small offset from the sole centers; their weighted
        # sum equals COM xy, satisfying whole-body static moment equilibrium.
        forces = {
            self.support: mass * 9.81 * (1 - 0.5 * reload),
            self.swing: mass * 9.81 * 0.5 * reload,
        }
        offset = [
            c[k] - sum(forces[s] * tf[s + "_foot"][k][3] for s in feet) / (mass * 9.81)
            for k in (0, 1)
        ]
        if abs(offset[0]) > 0.09 or abs(offset[1]) > 0.05:
            raise ValueError("Static pressure point lacks 10 mm sole margin")
        loads = double_support_loads(self.xml, q, forces)
        for j in self.root.findall("joint"):
            name = j.get("name")
            side = name.split("_")[0]
            force = forces[side]
            frame = multiply(tf[j.find("parent").get("link")], origin(j.find("origin")))
            axis = vector(j.find("axis"), "xyz")
            delta = [offset[1] * force, -offset[0] * force, 0.0]
            loads[name] -= sum(
                sum(frame[i][k] * axis[k] for k in range(3)) * delta[i]
                for i in range(3)
            )
        if max(abs(loads[n]) / cap for n, cap in zip(JOINT_NAMES, MOTOR_CAPS)) > 0.8:
            raise ValueError("First-step static motor reserve exceeded")
        for side in feet:
            if (
                max(
                    abs(
                        tf[side + "_foot"][k][3]
                        + [xy[0], xy[1], self.height][k]
                        - feet[side][k]
                    )
                    for k in range(3)
                )
                > 1e-9
            ):
                raise ValueError("First-step fixed-foot IK mismatch")
        collision_check(self.xml, q, self.height)
        return dict(
            target_rad=[q[n] for n in JOINT_NAMES],
            feedforward_nm=[loads[n] for n in JOINT_NAMES],
            pelvis_x_m=xy[0],
            pelvis_y_m=xy[1],
            pelvis_height_m=self.height,
            predicted_com_y_m=c[1] + xy[1],
            predicted_com_height_m=c[2] + self.height,
            predicted_force_n=forces,
            swing_advance_m=advance,
            swing_clearance_m=height,
        )

    def sample(self, elapsed):
        """Return targets for elapsed simulated seconds; angles are radians and torques N m.

        This samples a prechecked plan. It does not read sensors or publish commands."""
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("Finite nonnegative step time required")
        if elapsed < 117.0:
            return dict(
                super().sample(elapsed), swing_advance_m=0.0, swing_clearance_m=0.0
            )
        for phase, start, duration in [
            ("lift", 117.0, 15.0),
            ("advance", 132.0, 20.0),
            ("lower_foot", 152.0, 15.0),
            ("recontact", 167.0, 2.0),
            ("reload", 169.0, 64.0),
        ]:
            if elapsed < start + duration:
                p = interpolate(
                    self.paths["lower_foot" if phase == "recontact" else phase],
                    (
                        1.0
                        if phase == "recontact"
                        else smooth((elapsed - start) / duration)
                    ),
                )
                return dict(p, phase=phase, elapsed_s=elapsed)
        return dict(
            self.paths["reload"][-1],
            phase="staggered_hold" if elapsed < 241.0 else "done",
            elapsed_s=elapsed,
        )

    def guard(self, states, now, elapsed, observation, *, touchdown=False):
        """Validate fresh measured contacts and sole geometry against the planned phase.

        A rejected guard stops motion; missing observations cannot be treated as success.
        """
        r = zero_load_guard(
            states,
            now,
            self.planner.base["mass_kg"],
            self.expected_fraction(elapsed),
            self.support,
        )
        if not r["eligible"]:
            return r

        def fail(reason):
            return dict(r, eligible=False, reason=reason)

        if observation is None or not -0.02 <= now - observation["sim_time"] <= 0.1:
            return fail("Missing/stale first-step sole geometry")
        if abs(observation["mass_kg"] - self.planner.base["mass_kg"]) > 1e-8:
            return fail("First-step probe mass mismatch")
        if not all(
            math.isfinite(v)
            for v in [
                *observation["com"],
                *[
                    v
                    for corners in observation["feet"].values()
                    for c in corners
                    for v in c
                ],
            ]
        ):
            return fail("Nonfinite first-step geometry")
        p = self.sample(elapsed)
        for side in ("left", "right"):
            corners = observation["feet"][side]
            h = p["swing_clearance_m"] if side == self.swing else 0.0
            if max(abs(c[2] - h) for c in corners) > 0.0005:
                return fail("Step sole height tracking exceeded 0.5 mm")
            for k in (0, 1):
                expected = self.feet[side][k] + (
                    p["swing_advance_m"] if side == self.swing and k == 0 else 0.0
                )
                if abs(sum(c[k] for c in corners) / 4 - expected) > 0.001:
                    return fail("Step foot placement tracking exceeded 1 mm")
        from .touchdown import contact_force_limit

        force_limit = contact_force_limit(
            p["phase"], p["swing_clearance_m"], self.planner.base["mass_kg"], touchdown
        )
        if 115.0 <= elapsed < 169.0:
            if states[self.swing]["force_world_n"][2] > force_limit:
                return fail(
                    f"Swing/landing force exceeds phase limit {force_limit:g} N"
                )
            corners = observation["feet"][self.support]
            c = observation["com"]
            if (
                min(c[k] - min(v[k] for v in corners) for k in (0, 1)) < 0.01
                or min(max(v[k] for v in corners) - c[k] for k in (0, 1)) < 0.01
            ):
                return fail("Step support COM margin below 10 mm")
        if (
            p["swing_clearance_m"] >= 0.0005
            and states[self.swing]["state"] != "no_contact"
        ):
            return fail("Raised step foot lacks fresh empty contact evidence")
        if elapsed >= 168.0 and states[self.swing]["state"] not in (
            "touching_unloaded",
            "loaded_contact",
        ):
            return fail("New forward landing contact missing")
        return dict(
            r,
            reason="ok",
            geometry_stamp=observation["sim_time"],
            contact_force_limit_n=force_limit,
        )

    def controller_contacts_ok(self, row):
        """Replay the recorded live contact/geometry guard for this controller row."""
        return (
            bool(row.get("lift_guard", {}).get("eligible"))
            and self.guard(
                (row.get("load_guard") or {}).get("states", {}),
                row["sim_time"],
                row["shift"]["elapsed_s"],
                row.get("lift_observation"),
            )["eligible"]
        )
