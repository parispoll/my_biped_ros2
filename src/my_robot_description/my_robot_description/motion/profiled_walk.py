"""Time-map a geometric alternating gait, preserving measured contact holds.

Faster motion uses shorter simulated phases, not a higher real-time factor.
A piecewise clock keeps baseline, landing confirmation and final hold long
 enough to collect independent measurements with the original thresholds.
"""

import copy
from pathlib import Path
from my_robot_description.model.planning_identity import planning_fingerprint
from bisect import bisect_right
from my_robot_description.model.physical_damping import damping_coefficients
from my_robot_description.model.roll_stance import JOINT_NAMES, double_support_loads
from .alternating_walk import AlternatingWalkProgram
from .walking_profile import WalkingProfile


class ProfiledWalkProgram:
    def __init__(self, xml, direction="left", profile=None):
        self.profile = (profile or WalkingProfile()).validate()
        self.base = AlternatingWalkProgram(
            xml, direction, self.profile.step_count, self.profile.step_length_m
        )
        self.viscous = damping_coefficients(xml)
        self.knots = [(0.0, 0.0)]
        # The geometry stays on its original clock. Only the sampling clock changes.
        phases = [(name, duration) for name, duration in self.base.setup["schedule"]]
        phases += [(s["phase"], s["end"] - s["start"]) for s in self.base.segments]
        for name, duration in phases:
            minimum = {
                "baseline": 3.0,
                "lowered_settle": 2.0,
                "confirm_unloaded": 2.0,
                "recontact": 4.0 if self.profile.velocity_feedforward else 2.0,
                "staggered_hold": 4.0,
            }.get(name, 0.0)
            # Weight transfer and airborne motion have different constraints.
            # Preserve the tested transfer speed while independently shortening swing.
            scale = (
                self.profile.effective_transfer_scale
                if name in ("move_out", "transfer", "reload")
                else self.profile.time_scale
            )
            actual = max(minimum, duration * scale)
            old, new = self.knots[-1]
            self.knots.append((old + duration, new + actual))
        self.old_times = [a for a, b in self.knots]
        self.new_times = [b for a, b in self.knots]
        self.total = self.new_times[-1]
        self.steps = copy.deepcopy(self.base.steps)
        # Audits and GUI consume actual phase times, never the unscaled clock.
        for step in self.steps:
            for key in list(step):
                if key.endswith("_start") or key == "end":
                    step[key] = self.forward_time(step[key])
        self.touchdown_torques = {}
        if self.profile.velocity_feedforward:
            # A light, bounded force seed closes a microscopic floor gap. It is
            # not evidence of contact: the raw contact guard must still confirm.
            for step in self.steps:
                p = self.base.sample(self.base_time(step["recontact_start"]))
                q = dict(zip(JOINT_NAMES, p["target_rad"]))
                support, swing = step["support"], step["swing"]
                weight = self.base.planner.base["mass_kg"] * 9.81
                before = double_support_loads(xml, q, {support: weight, swing: 0.0})
                after = double_support_loads(
                    xml, q, {support: weight - 0.1, swing: 0.1}
                )
                self.touchdown_torques[step["index"]] = [
                    after[n] - before[n] for n in JOINT_NAMES
                ]
        self.setup = copy.deepcopy(self.base.setup)
        peak = self.base.setup["sampled_peak_reference_speed_rad_s"] / min(
            self.profile.time_scale, self.profile.effective_transfer_scale
        )
        if peak > self.profile.joint_speed_limit_rad_s:
            raise ValueError(
                "Scaled reference exceeds the explicit walking speed envelope"
            )
        self.setup.update(
            steps=self.steps,
            total_time_s=self.total,
            time_scale=self.profile.time_scale,
            transfer_time_scale=self.profile.effective_transfer_scale,
            velocity_feedforward=self.profile.velocity_feedforward,
            touchdown_preload_n=0.1 if self.profile.velocity_feedforward else 0.0,
            touchdown_force_limit_n=(
                0.02 * self.base.planner.base["mass_kg"] * 9.81
                if self.profile.velocity_feedforward
                else 0.5
            ),
            touchdown_region_m=0.0005,
            planning_fingerprint=planning_fingerprint(
                Path(__file__).resolve().parents[2]
            ),
            physical_damping_coefficients=self.viscous,
            joint_speed_audit_limit_rad_s=self.profile.joint_speed_limit_rad_s,
            sampled_peak_reference_speed_rad_s=peak,
            schedule=[
                (name, self.new_times[i + 1] - self.new_times[i])
                for i, (name, d) in enumerate(phases)
            ],
            analysis_windows=dict(
                baseline=(1.0, 2.5), recovery=(self.total - 3, self.total)
            ),
            note=f"{self.profile.step_count} finite alternating steps; {self.profile.step_length_m*1000:g} mm step length; timing scale {self.profile.time_scale:g}. Motor limits, freshness, geometry and IMU thresholds retained; touchdown policy is explicit in metadata.",
        )

    def __getattr__(self, name):
        return getattr(self.base, name)

    def map_time(self, t, source, dest):
        i = max(0, min(len(source) - 2, bisect_right(source, t) - 1))
        return dest[i] + (t - source[i]) * (dest[i + 1] - dest[i]) / (
            source[i + 1] - source[i]
        )

    def forward_time(self, t):
        return self.map_time(t, self.old_times, self.new_times)

    def base_time(self, t):
        return self.map_time(t, self.new_times, self.old_times)

    def sample(self, t):
        result = dict(self.base.sample(self.base_time(t)), elapsed_s=t)
        if self.profile.velocity_feedforward:
            # Joint PID damps measured velocity. Supply the corresponding desired
            # motion torque plus the physical damper load, within the same caps.
            # This is velocity/damping feedforward, not full inverse dynamics.
            lo, hi = max(0.0, t - 0.001), min(self.total, t + 0.001)
            if hi <= lo:
                velocity = [0.0] * 10
            else:
                a = self.base.sample(self.base_time(lo))["target_rad"]
                b = self.base.sample(self.base_time(hi))["target_rad"]
                velocity = [(y - x) / (hi - lo) for x, y in zip(a, b)]
            kd = self.derivative_gains(t)
            velocity_extra = [
                (d + k) * v for d, k, v in zip(self.viscous, kd, velocity)
            ]
            step = self.steps[result["step_index"] - 1]
            # Add the light seed only after nominal set-down, not during flight.
            blend = max(0.0, min(1.0, t - step["recontact_start"]))
            if t >= step["reload_start"]:
                blend = max(0.0, 1.0 - (t - step["reload_start"]))
            blend = blend**3 * (10.0 - 15.0 * blend + 6.0 * blend**2)
            preload = [blend * v for v in self.touchdown_torques[result["step_index"]]]
            extra = [a + b for a, b in zip(velocity_extra, preload)]
            result.update(
                gravity_feedforward_nm=result["feedforward_nm"].copy(),
                reference_velocity_rad_s=velocity,
                velocity_feedforward_nm=velocity_extra,
                touchdown_preload_torque_nm=preload,
                feedforward_nm=[g + f for g, f in zip(result["feedforward_nm"], extra)],
            )
        return result

    def expected_fraction(self, t):
        return self.base.expected_fraction(self.base_time(t))

    def proportional_gains(self, t):
        return self.base.proportional_gains(self.base_time(t))

    def derivative_gains(self, t):
        return self.base.derivative_gains(self.base_time(t))

    def guard(self, states, now, t, observation):
        return self.base.guard(
            states,
            now,
            self.base_time(t),
            observation,
            touchdown=self.profile.velocity_feedforward,
        )

    def controller_contacts_ok(self, row):
        return (
            bool(row.get("lift_guard", {}).get("eligible"))
            and self.guard(
                row["load_guard"]["states"],
                row["sim_time"],
                row["shift"]["elapsed_s"],
                row.get("lift_observation"),
            )["eligible"]
        )

    def sensor_contacts_ok(self, row, origin):
        # Keep source freshness in real simulated seconds; only the plan clock is mapped.
        p = self.sample(max(0.0, row["sim_time"] - origin))
        values = row["contacts"]
        s = p["supporting_foot"]
        other = p["swing_foot"]
        return (
            set(values) == {"left", "right"}
            and all(
                -0.02 <= row["sim_time"] - v["sim_time"] <= 0.05
                for v in values.values()
            )
            and values[s]["ground"]
            and (
                values[other]["ground"]
                or self.expected_fraction(row["sim_time"] - origin) <= 0.01
            )
        )
