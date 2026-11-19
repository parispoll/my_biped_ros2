"""Derive a conservative bilateral unloading plan within the existing model.

No foot is lifted. The same fixed-foot IK and joint PID serve this separate
experiment; measured forces supply a safety guard, not a force servo.
"""

import math
from my_robot_description.motion.lateral_shift import LateralPlanner, ShiftProgram
from my_robot_description.model.roll_stance import MOTOR_CAPS
from my_robot_description.model.roll_actuators import ACTIVE

SCHEDULE = [
    ("baseline", 3.0),
    ("move_out", 8.0),
    ("hold", 8.0),
    ("move_back", 8.0),
    ("recovery", 10.0),
]
WINDOWS = dict(baseline=(1.0, 2.5), hold=(17.0, 19.0), recovery=(34.0, 37.00001))
STATIC_UTILIZATION_LIMIT = 0.8
MIN_SHARE = 0.25


def derive_unloading(planner, target_fraction=0.35, direction="left"):
    if target_fraction not in (0.4, 0.35, 0.3) or direction not in ("left", "right"):
        raise ValueError(
            "Supported goals: 40%, 35% or P54-only 30% lighter-foot share, direction left/right"
        )
    if target_fraction == 0.3 and ACTIVE["name"] != "robotis_p54":
        raise ValueError("The 30% target requires the robotis_p54 actuator profile")
    weight = planner.base["mass_kg"] * 9.81
    sign = 1 if direction == "left" else -1
    unloaded = "right" if sign > 0 else "left"
    low = 0.0
    bracket = None
    scan = []
    for i in range(101):
        d = i * 0.001
        try:
            point = planner.fixed_foot_pose(sign * d)
            utilization = max(
                abs(t) / c for t, c in zip(point["feedforward_nm"], MOTOR_CAPS)
            )
            fraction = point["predicted_force_n"][unloaded] / weight
            scan.append(
                dict(
                    displacement_m=sign * d,
                    unloaded_fraction=fraction,
                    static_motor_utilization=utilization,
                )
            )
        except ValueError as error:
            scan.append(
                dict(displacement_m=sign * d, feasible=False, reason=str(error))
            )
            break
        if utilization > STATIC_UTILIZATION_LIMIT:
            scan[-1]["reason"] = "20% static torque headroom exhausted"
            break
        if fraction <= target_fraction:
            bracket = (low, d)
            break
        low = d
    if bracket is None:
        raise ValueError("Requested unloading is outside joint/torque headroom limits")
    low, high = bracket
    for _ in range(35):
        mid = (low + high) / 2
        p = planner.fixed_foot_pose(sign * mid)
        if p["predicted_force_n"][unloaded] / weight > target_fraction:
            low = mid
        else:
            high = mid
    displacement = sign * (low + high) / 2
    endpoint = planner.fixed_foot_pose(displacement)
    return dict(
        displacement_m=displacement,
        target_unloaded_fraction=target_fraction,
        unloaded_foot=unloaded,
        direction=direction,
        endpoint=endpoint,
        static_motor_utilization=max(
            abs(v) / cap for v, cap in zip(endpoint["feedforward_nm"], MOTOR_CAPS)
        ),
        scan=scan,
        static_utilization_limit=STATIC_UTILIZATION_LIMIT,
        min_allowed_measured_share=MIN_SHARE,
    )


class PartialUnloadProgram(ShiftProgram):
    def __init__(self, xml, target_fraction=0.35, direction="left"):
        self.planner = LateralPlanner(xml)
        derived = derive_unloading(self.planner, target_fraction, direction)
        self.displacement = derived["displacement_m"]
        self.schedule = SCHEDULE
        self.table = [
            self.planner.fixed_foot_pose(self.displacement * i / 40) for i in range(41)
        ]
        if any(
            abs(t) > STATIC_UTILIZATION_LIMIT * c
            for p in self.table
            for t, c in zip(p["feedforward_nm"], MOTOR_CAPS)
        ):
            raise ValueError("Trajectory consumes reserved static torque headroom")
        self.setup = dict(
            program_kind="partial_unload",
            **derived,
            schedule=SCHEDULE,
            total_time_s=sum(d for _, d in SCHEDULE),
            analysis_windows=WINDOWS,
            interpolation="41 fixed-foot poses with quintic time phase; no foot lift"
        )


def load_guard(loads, now, mass, minimum_share=MIN_SHARE):
    """Require fresh bilateral measured forces and conservative support shares."""
    if minimum_share not in (0.25, 0.18, 0.08, 0.03):
        raise ValueError("Unsupported measured share floor")
    weight = mass * 9.81
    if not math.isfinite(now) or not math.isfinite(weight) or weight <= 0:
        raise ValueError("Finite clock and positive weight required")
    if set(loads) != {"left", "right"}:
        return dict(eligible=False, reason="Missing decoded foot loads")
    ages = {s: now - r["sim_time"] for s, r in loads.items()}
    if any(not math.isfinite(a) or not -0.02 <= a <= 0.05 for a in ages.values()):
        return dict(eligible=False, reason="Stale foot-force data", ages_s=ages)
    forces = {s: r["force_world_n"][2] for s, r in loads.items()}
    if any(not math.isfinite(f) or f < 0 for f in forces.values()):
        return dict(eligible=False, reason="Invalid upward foot force")
    total = sum(forces.values())
    if abs(total - weight) > 0.05 * weight:
        return dict(
            eligible=False,
            reason="Vertical force sum outside 5% weight band",
            total_n=total,
        )
    shares = {s: f / total for s, f in forces.items()}
    return dict(
        eligible=min(shares.values()) >= minimum_share,
        reason=(
            "ok"
            if min(shares.values()) >= minimum_share
            else "Foot share below configured guard"
        ),
        forces_n=forces,
        total_n=total,
        shares=shares,
        ages_s=ages,
        min_share=minimum_share,
        used_for_control="safety guard only",
    )


def audit_load_guards(rows, start, end, mass, minimum_share=MIN_SHARE):
    selected = [
        r for r in rows if start <= r["sim_time"] <= end and r.get("shift") is not None
    ]
    if not selected:
        raise ValueError("No recorded force-guard evidence")
    minimum = 1.0
    max_age = 0.0
    peak_utilization = 0.0
    for r in selected:
        g = r.get("load_guard")
        if not g or not g["eligible"] or g.get("min_share") != minimum_share:
            raise ValueError("Missing/failed live force guard")
        total = sum(g["forces_n"].values())
        if (
            abs(total - g["total_n"]) > 1e-9
            or abs(total - mass * 9.81) > 0.05 * mass * 9.81
        ):
            raise ValueError("Invalid guard force sum")
        for side in ["left", "right"]:
            if not -0.02 <= g["ages_s"][side] <= 0.05:
                raise ValueError("Stale live guard sample")
            if abs(g["forces_n"][side] / total - g["shares"][side]) > 1e-12:
                raise ValueError("Invalid force share")
        minimum = min(minimum, min(g["shares"].values()))
        max_age = max(max_age, max(g["ages_s"].values()))
        peak_utilization = max(
            peak_utilization, max(abs(t) / c for t, c in zip(r["command"], MOTOR_CAPS))
        )
    return dict(
        passed=minimum >= minimum_share
        and peak_utilization <= STATIC_UTILIZATION_LIMIT,
        samples=len(selected),
        minimum_measured_share=minimum,
        share_floor=minimum_share,
        max_force_source_age_s=max_age,
        peak_measured_motor_utilization=peak_utilization,
        motor_utilization_limit=STATIC_UTILIZATION_LIMIT,
        force_feedback_used="safety guard, not a force servo",
    )
