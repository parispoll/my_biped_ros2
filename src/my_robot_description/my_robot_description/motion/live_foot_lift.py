"""Guarded vertical lift at 2, 10 or 30 mm, with landing and return.

This program moves the foot vertically. Forward placement belongs in
first_step.py; alternating support belongs in alternating_walk.py."""

import math
import xml.etree.ElementTree as ET
from my_robot_description.motion.zero_load import (
    ZeroLoadProgram,
    zero_load_guard,
    paired_contact_states,
)
from my_robot_description.motion.foot_lift import FootLiftPreview
from my_robot_description.model.roll_geometry import link_transforms, multiply, origin


def collision_check(xml, q, pelvis_height=None):
    """Conservative sampled oriented-box separation of URDF collision primitives.

    Cylinders are enclosed by oriented boxes. Connected collision bodies with
    at most two tree edges between them are mounting interfaces, excluded from
    self-collision checks (carriers have no collision). No collision geometry is
    disabled or modified. This is a static sample check, not swept collision.
    """
    root = ET.fromstring(xml)
    tf = link_transforms(xml, q)
    boxes = {}
    graph = {}
    for j in root.findall("joint"):
        a, b = j.find("parent").get("link"), j.find("child").get("link")
        graph.setdefault(a, set()).add(b)
        graph.setdefault(b, set()).add(a)
    for link in root.findall("link"):
        for c in link.findall("collision"):
            name = link.get("name")
            geometry = c.find("geometry")
            if geometry.find("box") is not None:
                half = [float(v) / 2 for v in geometry.find("box").get("size").split()]
            elif geometry.find("cylinder") is not None:
                cylinder = geometry.find("cylinder")
                r = float(cylinder.get("radius"))
                half = [r, r, float(cylinder.get("length")) / 2]
            else:
                raise ValueError("Unsupported collision geometry in first-lift check")
            frame = multiply(tf[name], origin(c.find("origin")))
            if name in boxes:
                raise ValueError("First-lift check expects one collision per link")
            if pelvis_height is not None and not name.endswith("_foot"):
                bottom = (
                    frame[2][3]
                    - sum(abs(frame[2][k]) * half[k] for k in range(3))
                    + pelvis_height
                )
                if bottom < 0:
                    raise ValueError(
                        "Non-foot collision envelope intersects the floor: " + name
                    )
            boxes[name] = (
                [frame[i][3] for i in range(3)],
                [[frame[i][k] for i in range(3)] for k in range(3)],
                half,
            )
    minimum = float("inf")
    pairs = 0
    for a, box in boxes.items():
        connected = {a, *graph[a]}
        for neighbor in graph[a]:
            connected.update(graph[neighbor])
        for b, other in boxes.items():
            if b <= a or b in connected:
                continue
            center, axes, half = box
            other_center, other_axes, other_half = other
            cross = lambda a, b: [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
            dot = lambda a, b: sum(x * y for x, y in zip(a, b))
            candidates = (
                axes + other_axes + [cross(a, b) for a in axes for b in other_axes]
            )
            distance = 0.0
            for axis in candidates:
                length = math.sqrt(dot(axis, axis))
                if length < 1e-10:
                    continue
                axis = [x / length for x in axis]
                separation = (
                    abs(dot([b - a for a, b in zip(center, other_center)], axis))
                    - sum(h * abs(dot(a, axis)) for h, a in zip(half, axes))
                    - sum(h * abs(dot(a, axis)) for h, a in zip(other_half, other_axes))
                )
                distance = max(distance, separation)
            if distance <= 1e-5:
                raise ValueError(f"Conservative collision envelopes overlap: {a}, {b}")
            minimum = min(minimum, distance)
            pairs += 1
    return dict(
        minimum_separated_envelope_gap_m=minimum,
        checked_pairs=pairs,
        scope="Sampled conservative envelopes; directly mounted bodies excluded, no swept/impact claim",
    )


class LiveFootLiftProgram(ZeroLoadProgram):
    insert_start = 117.0
    insert_duration = 24.0

    def __init__(self, xml, direction="left", clearance=0.002):
        super().__init__(xml, direction)
        self.preview = FootLiftPreview(xml, direction, clearance, zero_program=self)
        self.lift_end = 117.0 + self.preview.move_time_s
        self.hold_end = self.lift_end + self.preview.hold_time_s
        self.lower_end = self.hold_end + self.preview.move_time_s
        self.insert_end = self.lower_end + 2.0
        self.insert_duration = self.insert_end - 117.0
        total = 233.0 + self.insert_duration
        self.collision_checks = [
            collision_check(
                xml,
                dict(zip(self.planner.base["joint_names"], r["target_rad"])),
                self.setup["endpoint"]["pelvis_height_m"],
            )
            for r in self.preview.table
        ]
        self.setup = dict(
            self.setup,
            experiment_kind="live_foot_lift",
            total_time_s=total,
            foot_lift_commanded=True,
            clearance_m=clearance,
            analysis_windows=dict(
                baseline=(1.0, 2.5),
                hold=(self.hold_end - 1.5, self.hold_end),
                recovery=(total - 3.0, total + 0.00001),
            ),
            schedule=[
                *self.schedule[:5],
                ("lift", self.preview.move_time_s),
                ("lifted_hold", self.preview.hold_time_s),
                ("lower_foot", self.preview.move_time_s),
                ("recontact", 2.0),
                *self.schedule[5:],
            ],
            collision_check=dict(
                samples=len(self.collision_checks),
                minimum_gap_m=min(
                    r["minimum_separated_envelope_gap_m"] for r in self.collision_checks
                ),
                scope=self.collision_checks[0]["scope"],
            ),
            note=f"Guarded {clearance*1000:g} mm vertical foot lift; measured clearance/contact/support/return audits required.",
        )

    def base_time(self, elapsed):
        """Map the inserted lift interval onto the earlier zero-load preparation clock."""
        return (
            elapsed
            if elapsed < 117.0
            else (
                117.0 - 1e-9
                if elapsed < self.insert_end
                else elapsed - self.insert_duration
            )
        )

    def sample(self, elapsed):
        """Return targets for elapsed simulated seconds; angles are radians and torques N m.

        This samples a prechecked plan. It does not read sensors or publish commands."""
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("Nonnegative finite lift time required")
        point = super().sample(self.base_time(elapsed))
        if 117.0 <= elapsed < self.insert_end:
            lift = self.preview.sample(
                min(
                    2 * self.preview.move_time_s + self.preview.hold_time_s,
                    elapsed - 117.0,
                )
            )
            phase = (
                "recontact"
                if elapsed >= self.lower_end
                else {"lower": "lower_foot", "done": "recontact"}.get(
                    lift["phase"], lift["phase"]
                )
            )
            u = max(
                0.0,
                min(
                    float(self.preview.intervals),
                    lift["requested_clearance_m"]
                    / self.preview.clearance_m
                    * self.preview.intervals,
                ),
            )
            i = min(self.preview.intervals - 1, int(u))
            f = u - i
            a, b = self.preview.table[i : i + 2]
            point.update(
                target_rad=lift["target_rad"],
                feedforward_nm=[
                    x + (y - x) * f
                    for x, y in zip(a["feedforward_nm"], b["feedforward_nm"])
                ],
                predicted_com_y_m=a["predicted_com_local_m"][1]
                + (b["predicted_com_local_m"][1] - a["predicted_com_local_m"][1]) * f
                + self.displacement,
                predicted_force_n=a["predicted_force_n"],
                phase=phase,
            )
        return dict(point, elapsed_s=elapsed, swing_clearance_m=self.clearance(elapsed))

    def clearance(self, elapsed):
        return (
            self.preview.sample(elapsed - 117.0)["requested_clearance_m"]
            if 117.0 <= elapsed < self.lower_end
            else 0.0
        )

    def foot_offset(self, elapsed, side):
        return self.clearance(elapsed) if side == self.setup["unloaded_foot"] else 0.0

    def guard(self, states, now, elapsed, observation):
        """Validate fresh measured contacts and sole geometry against the planned phase.

        A rejected guard stops motion; missing observations cannot be treated as success.
        """
        result = zero_load_guard(
            states,
            now,
            self.planner.base["mass_kg"],
            self.expected_fraction(elapsed),
            self.setup["supporting_foot"],
        )
        if not result["eligible"]:
            return result

        def fail(reason):
            return dict(result, eligible=False, reason=reason)

        if observation is None or not -0.02 <= now - observation["sim_time"] <= 0.1:
            return fail("Missing/stale independent sole geometry")
        if abs(observation["mass_kg"] - self.planner.base["mass_kg"]) > 1e-8:
            return fail("Sole probe model mass mismatch")
        support = self.setup["supporting_foot"]
        swing = self.setup["unloaded_foot"]
        for side in (support, swing):
            corners = observation["feet"][side]
            target = self.foot_offset(elapsed, side)
            if max(abs(c[2] - target) for c in corners) > 0.0005:
                return fail("Measured sole clearance differs from plan by over 0.5 mm")
            for k in (0, 1):
                expected = self.planner.feet[side + "_foot"][k][3]
                if max(abs(sum(c[k] for c in corners) / 4 - expected), 0.0) > 0.001:
                    return fail("Foot drift exceeded 1 mm")
        if 115.0 <= elapsed < self.insert_end:
            if states[swing]["force_world_n"][2] > 0.5:
                return fail("Swing foot not unloaded below 0.5 N")
            corners = observation["feet"][support]
            x, y = observation["com"][:2]
            margin = min(
                x - min(c[0] for c in corners),
                max(c[0] for c in corners) - x,
                y - min(c[1] for c in corners),
                max(c[1] for c in corners) - y,
            )
            if margin < 0.01:
                return fail("Supporting-sole COM margin below 10 mm")
        if self.clearance(elapsed) >= 0.0005 and states[swing]["state"] != "no_contact":
            return fail("Raised swing sole must have fresh no-contact evidence")
        if self.insert_end - 1.0 <= elapsed < self.insert_end and states[swing][
            "state"
        ] not in ("touching_unloaded", "loaded_contact"):
            return fail("Landing contact not confirmed before reload")
        return dict(result, reason="ok", geometry_stamp=observation["sim_time"])

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


def audit_lift(raw, com, rows, start, end, program):
    """Replay raw contact states with independent physics sole/COM observations."""
    from bisect import bisect_right

    pairs = paired_contact_states(raw, start, end)
    observations = [r for r in com if start - 0.1 <= r["sim_time"] <= end]
    times = [r["sim_time"] for r in observations]
    valid = True
    clearances = []
    held = []
    landed = []
    for p in pairs:
        i = bisect_right(times, p["sim_time"]) - 1
        if i < 0:
            raise ValueError("Missing independent sole observation")
        observation = observations[i]
        t = p["sim_time"] - start
        valid &= program.guard(
            {s: p[s] for s in ("left", "right")}, p["sim_time"], t, observation
        )["eligible"]
        if (
            program.setup["analysis_windows"]["hold"][0]
            <= t
            < program.setup["analysis_windows"]["hold"][1]
        ):
            held.append(p)
            clearances.append(
                min(c[2] for c in observation["feet"][program.setup["unloaded_foot"]])
            )
        if program.insert_end - 1.0 <= t < program.insert_end:
            landed.append(p)
    live = [
        r for r in rows if r.get("shift") is not None and start <= r["sim_time"] <= end
    ]
    valid &= bool(live) and all(program.controller_contacts_ok(r) for r in live)
    swing = program.setup["unloaded_foot"]
    support = program.setup["supporting_foot"]
    gaps = [b["sim_time"] - a["sim_time"] for a, b in zip(pairs, pairs[1:])]
    coverage = (
        bool(gaps)
        and 0 < min(gaps)
        and max(gaps) <= 0.05
        and pairs[0]["sim_time"] - start <= 0.05
        and end - pairs[-1]["sim_time"] <= 0.05
    )
    absence = bool(held) and all(p[swing]["state"] == "no_contact" for p in held)
    landing = bool(landed) and all(
        p[swing]["state"] in ("touching_unloaded", "loaded_contact") for p in landed
    )
    passed = (
        coverage
        and valid
        and len(held) >= 20
        and len(landed) >= 20
        and absence
        and landing
        and min(clearances) >= program.preview.clearance_m - 0.0005
    )
    return dict(
        passed=bool(passed),
        supporting_foot=support,
        swing_foot=swing,
        held_minimum_clearance_m=min(clearances) if clearances else None,
        held_samples=len(held),
        requested_clearance_m=program.preview.clearance_m,
        required_minimum_clearance_m=program.preview.clearance_m - 0.0005,
        held_fresh_no_contact=absence,
        landing_contact_confirmed=landing,
        valid_live_and_independent_guards=bool(valid),
        force_coverage_passed=coverage,
        maximum_force_gap_s=max(gaps),
        foot_lift_commanded=True,
        note="Measured small lift/contact loss/landing in a slow no-push trial; not walking or disturbance recovery.",
    )
