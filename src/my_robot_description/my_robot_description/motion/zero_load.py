"""Explicit zero-load transition. A fixed-sole reference is not a foot lift."""

import math
from bisect import bisect_right
from my_robot_description.motion.lowered_unload import LoweredUnloadProgram
from my_robot_description.sensing.contact_state import contact_state


class ZeroLoadProgram(LoweredUnloadProgram):
    def validate_goal(self, fraction, direction):
        if fraction != 0 or direction not in ("left", "right"):
            raise ValueError(
                "Zero-load mode requires zero target and left/right support"
            )

    def validate_displacement(self, d):
        if not math.isfinite(d) or not 0 < abs(d) <= 0.14:
            raise ValueError("Zero-load IK exceeded checked 140 mm envelope")

    def __init__(self, xml, direction="left"):
        super().__init__(xml, 0.0, direction)
        self.minimum_measured_share = None
        self.setup.update(
            support_ankle_gains_schedule=dict(
                kp=120.0,
                kd=8.0,
                start_fraction=0.05,
                end_fraction=0.01,
                blend="quintic",
            ),
            unloaded_ankle_roll_kd_schedule=dict(
                loaded=2.0,
                unloaded=0.3,
                start_fraction=0.05,
                end_fraction=0.01,
                blend="quintic",
            ),
            experiment_kind="zero_load_transition",
            minimum_measured_share=None,
            empty_contact_allowed_below_planned_fraction=0.01,
            held_unloaded_force_limit_n=0.5,
            note="Physics-stamped contact states; fixed flat-sole reference, no lift commanded. Zero measured force does not establish dynamic single support.",
        )

    def gain_blend(self, elapsed):
        """Blend the established support gains smoothly near zero swing-foot load."""
        u = max(0.0, min(1.0, (0.05 - self.expected_fraction(elapsed)) / 0.04))
        return u * u * u * (10 + u * (-15 + 6 * u))

    def proportional_gains(self, elapsed):
        """Select joint stiffness gains for the supporting-foot role at this time."""
        from my_robot_description.control.roll_posture import KP

        gains = list(KP)
        blend = self.gain_blend(elapsed)
        offset = 0 if self.setup["supporting_foot"] == "left" else 5
        for i in [offset + 3, offset + 4]:
            gains[i] = KP[i] + (120.0 - KP[i]) * blend
        return gains

    def derivative_gains(self, elapsed):
        """Select velocity damping gains for the current supporting and free ankles."""
        from my_robot_description.control.roll_posture import KD

        gains = list(KD)
        blend = self.gain_blend(elapsed)
        offset = 0 if self.setup["supporting_foot"] == "left" else 5
        index = 9 if self.setup["unloaded_foot"] == "right" else 4
        gains[index] = KD[index] + (0.3 - KD[index]) * blend
        for i in [offset + 3, offset + 4]:
            gains[i] = KD[i] + (8.0 - KD[i]) * blend
        return gains

    def expected_fraction(self, elapsed):
        """Return the planned unloaded/swing-foot vertical load fraction (0 to 0.5)."""
        p = self.sample(max(0.0, elapsed))
        forces = p["predicted_force_n"]
        return forces[self.setup["unloaded_foot"]] / sum(forces.values())

    def controller_contacts_ok(self, row):
        """Replay the recorded live contact/geometry guard for this controller row."""
        g = row.get("load_guard") or {}
        states = g.get("states", {})
        checked = zero_load_guard(
            states,
            row["sim_time"],
            self.planner.base["mass_kg"],
            self.expected_fraction(row["shift"]["elapsed_s"]),
            self.setup["supporting_foot"],
        )
        return g.get("eligible") is True and checked["eligible"]

    def sensor_contacts_ok(self, row, origin):
        """Check independent contact freshness using the planned support phase."""
        values = row["contacts"]
        support = self.setup["supporting_foot"]
        other = self.setup["unloaded_foot"]
        return (
            set(values) == {"left", "right"}
            and all(
                -0.02 <= row["sim_time"] - c["sim_time"] <= 0.05
                for c in values.values()
            )
            and values[support]["ground"]
            and (
                values[other]["ground"]
                or self.expected_fraction(row["sim_time"] - origin) <= 0.01
            )
        )


def zero_load_guard(states, now, mass, expected_fraction, support):
    if (
        support not in ("left", "right")
        or not all(math.isfinite(v) for v in [now, mass, expected_fraction])
        or mass <= 0
        or not 0 <= expected_fraction <= 0.5 + 1e-9
    ):
        raise ValueError("Invalid zero-load guard configuration")

    def result(ok, reason, **kwargs):
        return dict(
            eligible=ok,
            reason=reason,
            states=states,
            expected_unloaded_fraction=expected_fraction,
            supporting_foot=support,
            **kwargs,
        )

    if set(states) != {"left", "right"}:
        return result(False, "Missing contact-state data")
    for side, s in states.items():
        if (
            s.get("side") != side
            or not s.get("measurement_valid")
            or s.get("state")
            not in ("loaded_contact", "touching_unloaded", "no_contact")
        ):
            return result(False, "Invalid/unknown contact-state data")
        if (
            not math.isfinite(s.get("sim_time", float("nan")))
            or not -0.02 <= now - s["sim_time"] <= 0.05
        ):
            return result(False, "Stale contact state")
        f = s.get("force_world_n")
        if (
            not isinstance(f, list)
            or len(f) != 3
            or not all(math.isfinite(v) for v in f)
            or f[2] < 0
        ):
            return result(False, "Invalid measured force")
    other = "right" if support == "left" else "left"
    weight = mass * 9.81
    if (
        states[support]["state"] != "loaded_contact"
        or states[support]["force_world_n"][2] <= 1.0
    ):
        return result(False, "Supporting contact lost or unloaded")
    if states[other]["state"] == "no_contact" and expected_fraction > 0.01:
        return result(False, "Other foot lost contact before zero-load window")
    forces = {s: states[s]["force_world_n"][2] for s in states}
    total = sum(forces.values())
    if abs(total - weight) > 0.05 * weight:
        return result(False, "Vertical support outside 5% weight band")
    fraction = forces[other] / total
    if abs(fraction - expected_fraction) > 0.02:
        return result(
            False,
            "Measured load differs from planned transfer by over two percentage points",
        )
    return result(
        True, "ok", forces_n=forces, total_n=total, measured_unloaded_fraction=fraction
    )


def paired_contact_states(raw, start, end):
    feet = {s: [] for s in ["left", "right"]}
    for row in raw:
        if start - 0.05 <= row["sim_time"] <= end:
            side = row["side"]
            s = contact_state(row, row["sim_time"], side)
            if not s["measurement_valid"]:
                raise ValueError(
                    "Invalid physics contact-state evidence: " + s["reason"]
                )
            feet[side].append(s)
    for side, values in feet.items():
        if len(values) < 2 or any(
            b["sim_time"] <= a["sim_time"] for a, b in zip(values, values[1:])
        ):
            raise ValueError("Missing or reordered contact states for " + side)
    times = [r["sim_time"] for r in feet["right"]]
    pairs = []
    for left in feet["left"]:
        t = left["sim_time"]
        if t < start:
            continue
        i = bisect_right(times, t) - 1
        if i < 0:
            continue
        right = feet["right"][i]
        # ROS stamps have integer-nanosecond resolution. Floating subtraction
        # can turn exactly 30 ms into 0.030000000000001 s. Preserve the
        # same inclusive 30 ms boundary without accepting even 1 ns more.
        if round(t * 1e9) - round(right["sim_time"] * 1e9) > 30_000_000:
            raise ValueError("Stale paired contact state")
        total = left["force_world_n"][2] + right["force_world_n"][2]
        if total <= 0:
            raise ValueError("Both contacts unloaded")
        cop = [0.0, 0.0]
        for s in [left, right]:
            if s["force_world_n"][2] > 0:
                if s["cop_world_xy_m"] is None:
                    raise ValueError("Positive force missing pressure point")
                for k in range(2):
                    cop[k] += s["force_world_n"][2] * s["cop_world_xy_m"][k] / total
        pairs.append(
            dict(
                sim_time=t,
                left=left,
                right=right,
                total_vertical_n=total,
                cop_world_xy_m=cop,
            )
        )
    if len(pairs) < 2:
        raise ValueError("Missing paired contact-state samples")
    return pairs


def audit_zero_load(raw, com, rows, start, end, program):
    pairs = paired_contact_states(raw, start, end)
    mass = program.planner.base["mass_kg"]
    support = program.setup["supporting_foot"]
    other = program.setup["unloaded_foot"]
    gaps = [b["sim_time"] - a["sim_time"] for a, b in zip(pairs, pairs[1:])]
    valid = all(
        zero_load_guard(
            {s: p[s] for s in ["left", "right"]},
            p["sim_time"],
            mass,
            program.expected_fraction(p["sim_time"] - start),
            support,
        )["eligible"]
        for p in pairs
    )
    live = [
        r for r in rows if r.get("shift") is not None and start <= r["sim_time"] <= end
    ]
    valid = (
        valid and bool(live) and all(program.controller_contacts_ok(r) for r in live)
    )
    lo, hi = program.setup["analysis_windows"]["hold"]
    held = [p for p in pairs if lo <= p["sim_time"] - start < hi]
    observations = [r for r in com if lo <= r["sim_time"] - start < hi]
    margins = []
    for r in observations:
        corners = r["feet"][support]
        x, y = r["com"][:2]
        margins.append(
            min(
                x - min(c[0] for c in corners),
                max(c[0] for c in corners) - x,
                y - min(c[1] for c in corners),
                max(c[1] for c in corners) - y,
            )
        )
    force = (
        sum(p[other]["force_world_n"][2] for p in held) / len(held) if held else None
    )
    margin = min(margins) if margins else None
    coverage = (
        bool(gaps)
        and 0 < min(gaps)
        and max(gaps) <= 0.05
        and pairs[0]["sim_time"] - start <= 0.05
        and end - pairs[-1]["sim_time"] <= 0.05
    )
    passed = (
        coverage
        and valid
        and len(held) >= 20
        and len(observations) >= 20
        and force <= 0.5
        and margin >= 0.01
    )
    return dict(
        passed=bool(passed),
        target_fraction=0.0,
        measured_held_unloaded_force_n=force,
        minimum_held_support_com_margin_m=margin,
        supporting_foot=support,
        unloaded_foot=other,
        force_samples=len(pairs),
        max_force_gap_s=max(gaps),
        held_no_contact_samples=sum(p[other]["state"] == "no_contact" for p in held),
        held_touching_unloaded_samples=sum(
            p[other]["state"] == "touching_unloaded" for p in held
        ),
        valid_live_and_independent_guards=valid,
        coverage_passed=coverage,
        foot_lift_commanded=False,
        note="Measured unloaded force <= 0.5 N, with fresh support and geometric COM margin; no lifted-foot or disturbance-recovery claim.",
    )
