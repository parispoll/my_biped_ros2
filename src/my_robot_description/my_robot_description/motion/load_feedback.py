"""Bounded bilateral load-share admittance; independent of ROS.

Positive left-share error moves the pelvis toward world +Y (robot left).
This is an outer posture correction, not direct force/torque control.
"""

import math
from my_robot_description.motion.partial_unload import (
    PartialUnloadProgram,
    STATIC_UTILIZATION_LIMIT,
    MIN_SHARE,
)
from my_robot_description.model.roll_stance import MOTOR_CAPS
from my_robot_description.model.roll_actuators import ACTIVE

GAIN_M_S_PER_SHARE = 0.04
FILTER_TIME_S = 1.0
OFFSET_CAP_M = 0.004
RATE_CAP_M_S = 0.002


class LoadShareFeedback:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.filtered = 0.0
        self.offset = 0.0

    def update(self, target_left, guard, dt):
        if (
            not all(math.isfinite(v) for v in (target_left, dt))
            or not 0.25 <= target_left <= 0.75
            or not 0 <= dt <= 0.05
        ):
            raise ValueError(
                "Valid bilateral target and 0–0.05 s outer timestep required"
            )
        if not guard or not guard.get("eligible"):
            raise ValueError("Fresh eligible bilateral force guard required")
        forces = guard["forces_n"]
        total = sum(forces.values())
        if (
            not all(
                math.isfinite(forces[s]) and forces[s] > 0 for s in ["left", "right"]
            )
            or total <= 0
        ):
            raise ValueError("Finite positive bilateral forces required")
        measured = forces["left"] / total
        error = target_left - measured
        if min(measured, 1 - measured) < MIN_SHARE:
            raise ValueError("Measured force share below support floor")
        if any(
            not math.isfinite(a) or not -0.02 <= a <= 0.05
            for a in guard["ages_s"].values()
        ):
            raise ValueError("Stale force input")
        before = self.offset
        rate = 0.0
        clipped = False
        if self.enabled and dt > 0:
            self.filtered += (dt / (FILTER_TIME_S + dt)) * (error - self.filtered)
            rate = max(
                -RATE_CAP_M_S, min(RATE_CAP_M_S, GAIN_M_S_PER_SHARE * self.filtered)
            )
            proposed = self.offset + rate * dt
            self.offset = max(-OFFSET_CAP_M, min(OFFSET_CAP_M, proposed))
            clipped = abs(proposed - self.offset) > 1e-15
        return dict(
            enabled=self.enabled,
            target_left_fraction=target_left,
            measured_left_fraction=measured,
            error_fraction=error,
            filtered_error_fraction=self.filtered,
            outer_dt_s=dt,
            correction_m=self.offset,
            previous_correction_m=before,
            requested_rate_m_s=rate,
            offset_clipped=clipped,
            offset_cap_m=OFFSET_CAP_M,
            rate_cap_m_s=RATE_CAP_M_S,
        )


class LoadTransferProgram(PartialUnloadProgram):
    """Nominal load reference plus explicit test bias and bounded correction."""

    def __init__(
        self,
        xml,
        target_fraction=0.3,
        direction="left",
        feedback_enabled=False,
        reference_bias=0.0,
    ):
        if ACTIVE["name"] != "robotis_p54":
            raise ValueError("Load feedback experiment requires robotis_p54")
        if not math.isfinite(reference_bias) or abs(reference_bias) > 0.003:
            raise ValueError("Reference test bias must be within ±3 mm")
        super().__init__(xml, target_fraction, direction)
        self.feedback_enabled = feedback_enabled
        self.bias = (1 if direction == "left" else -1) * reference_bias
        nominal_endpoint = self.setup["endpoint"]
        span = [
            -OFFSET_CAP_M,
            OFFSET_CAP_M,
            self.displacement + self.bias - OFFSET_CAP_M,
            self.displacement + self.bias + OFFSET_CAP_M,
        ]
        self.low = min(span)
        self.high = max(span)
        self.command_table = [
            self.planner.fixed_foot_pose(self.low + (self.high - self.low) * i / 200)
            for i in range(201)
        ]
        if any(
            abs(t) > 0.8 * c
            for p in self.command_table
            for t, c in zip(p["feedforward_nm"], MOTOR_CAPS)
        ):
            raise ValueError("Correction workspace consumes reserved motor budget")
        self.audit_displacement = (
            self.displacement if feedback_enabled else self.displacement + self.bias
        )
        self.setup.update(
            program_kind="load_feedback",
            nominal_endpoint=nominal_endpoint,
            endpoint=self.planner.fixed_foot_pose(self.audit_displacement),
            feedback_enabled=feedback_enabled,
            reference_bias_world_m=self.bias,
            nominal_displacement_m=self.displacement,
            audit_displacement_m=self.audit_displacement,
            load_feedback=dict(
                gain_m_s_per_share=GAIN_M_S_PER_SHARE,
                filter_time_s=FILTER_TIME_S,
                offset_cap_m=OFFSET_CAP_M,
                rate_cap_m_s=RATE_CAP_M_S,
            ),
        )

    def command_sample(self, elapsed, correction):
        if not math.isfinite(correction) or abs(correction) > OFFSET_CAP_M + 1e-12:
            raise ValueError("Invalid bounded load correction")
        nominal = super().sample(elapsed)
        u = nominal["pelvis_y_m"] / self.displacement
        d = nominal["pelvis_y_m"] + self.bias * u + correction
        if not self.low - 1e-12 <= d <= self.high + 1e-12:
            raise ValueError("Command outside checked correction workspace")
        point = max(0.0, min(200.0, 200 * (d - self.low) / (self.high - self.low)))
        i = min(199, int(point))
        f = point - i
        a, b = self.command_table[i : i + 2]
        blend = lambda key: [x + (y - x) * f for x, y in zip(a[key], b[key])]
        return dict(
            **{k: nominal[k] for k in ["phase", "elapsed_s"]},
            target_rad=blend("target_rad"),
            feedforward_nm=blend("feedforward_nm"),
            pelvis_y_m=d,
            predicted_com_y_m=a["predicted_com_y_m"]
            + (b["predicted_com_y_m"] - a["predicted_com_y_m"]) * f,
            predicted_force_n={
                s: a["predicted_force_n"][s]
                + (b["predicted_force_n"][s] - a["predicted_force_n"][s]) * f
                for s in ["left", "right"]
            }
        )


def audit_load_feedback(rows, program, start, end):
    selected = [
        r for r in rows if start <= r["sim_time"] <= end and r.get("shift") is not None
    ]
    if not selected:
        raise ValueError("Missing recorded outer-loop updates")
    replay = LoadShareFeedback(program.feedback_enabled)
    previous = None
    peak = rate_peak = 0.0
    residual = 0.0
    clipped = 0
    for r in selected:
        state = r.get("load_feedback")
        if state is None:
            raise ValueError("Missing load-feedback state")
        dt = 0.0 if previous is None else r["sim_time"] - previous
        nominal = program.sample(r["sim_time"] - start)
        target = nominal["predicted_force_n"]["left"] / (
            program.planner.base["mass_kg"] * 9.81
        )
        expected = replay.update(target, r["load_guard"], dt)
        for key, value in expected.items():
            if isinstance(value, bool):
                if state[key] != value:
                    raise ValueError("Incorrect recorded feedback enable/clipping flag")
            else:
                residual = max(residual, abs(state[key] - value))
        if not all(
            math.isfinite(v) for v in [state["correction_m"], state["error_fraction"]]
        ):
            raise ValueError("Nonfinite feedback state")
        peak = max(peak, abs(state["correction_m"]))
        clipped += int(state["offset_clipped"])
        if previous is not None:
            rate_peak = max(
                rate_peak,
                abs(state["correction_m"] - state["previous_correction_m"]) / dt,
            )
        previous = r["sim_time"]
    if (
        residual > 1e-10
        or peak > OFFSET_CAP_M + 1e-12
        or rate_peak > RATE_CAP_M_S + 1e-12
    ):
        raise ValueError("Feedback replay/bound check failed")
    return dict(
        passed=True,
        enabled=program.feedback_enabled,
        samples=len(selected),
        peak_correction_m=peak,
        peak_correction_rate_m_s=rate_peak,
        clipped_samples=clipped,
        max_replay_error=residual,
        force_feedback_used=(
            "bounded pelvis correction"
            if program.feedback_enabled
            else "record-only comparator"
        ),
    )
