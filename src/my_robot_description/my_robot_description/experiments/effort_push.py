"""Configuration and quantitative checks for effort-mode disturbance limit experiments."""

from dataclasses import dataclass
import math
from my_robot_description.experiments.disturbance import PushConfig, summarize


@dataclass(frozen=True)
class EffortPushConfig(PushConfig):
    force_x: float = 1.0

    def validate(self):
        # Input envelope only: this does not certify recovery at these settings.
        super().validate()
        return self


def summarize_effort(samples, events, config):
    """Compare with this trial's settled baseline, without claiming active balance."""
    report = summarize(samples, events, config)
    baseline = [s for s in samples if s["phase"] == "baseline"]
    response = [s for s in samples if s["phase"] != "baseline"]
    clear = next(e["sim_time"] for e in events if e["event"] == "clear_published")
    recovery = [s for s in response if s["sim_time"] >= clear]
    mean = lambda key: sum(s[key] for s in baseline) / len(baseline)
    joints0 = [
        sum(s["actual_joints"][i] for s in baseline) / len(baseline) for i in range(6)
    ]
    commands0 = [
        sum(s["joint_command_nm"][i] for s in baseline) / len(baseline)
        for i in range(6)
    ]
    pitch0, height0, acceleration0 = (
        mean("pitch"),
        mean("height"),
        mean("acceleration_x"),
    )

    def joint_change(sample):
        return max(abs(a - b) for a, b in zip(sample["actual_joints"], joints0))

    def inside(sample):
        return (
            abs(sample["pitch"] - pitch0) <= math.radians(0.1)
            and abs(sample["pitch_rate"]) <= 0.01
            and abs(sample["height"] - height0) <= 0.002
            and joint_change(sample) <= 0.002
        )

    tail = [s for s in recovery if s["sim_time"] >= recovery[-1]["sim_time"] - 1]
    final_return = (
        len(tail) > 1
        and tail[-1]["sim_time"] - tail[0]["sim_time"] >= 0.95
        and all(b["sim_time"] - a["sim_time"] <= 0.1 for a, b in zip(tail, tail[1:]))
        and all(inside(s) for s in tail)
    )
    outside = [s["sim_time"] - clear for s in recovery if not inside(s)]
    acceleration_change = max(
        abs(s["acceleration_x"] - acceleration0) for s in response
    )
    detected = report["peak_pitch_change_rad"] > 1e-5 or acceleration_change > 0.01
    contacts_ok = all(
        not value["unknown_samples"] and not value["fresh_non_ground_samples"]
        for value in report["contacts"].values()
    )
    report.update(
        controller_mode="bounded_joint_effort",
        baseline_joint_positions_rad=joints0,
        baseline_commands_nm=commands0,
        baseline_acceleration_x_m_s2=acceleration0,
        peak_acceleration_change_m_s2=acceleration_change,
        peak_joint_change_from_baseline_rad=max(joint_change(s) for s in response),
        peak_command_change_nm=[
            max(abs(s["joint_command_nm"][i] - commands0[i]) for s in response)
            for i in range(6)
        ],
        peak_abs_commands_nm=[
            max(abs(s["joint_command_nm"][i]) for s in samples) for i in range(6)
        ],
        final_body_x_change_m=tail[-1]["body_x"] - mean("body_x"),
        final_pitch_change_rad=tail[-1]["pitch"] - pitch0,
        final_joint_change_from_baseline_rad=joint_change(tail[-1]),
        final_second_within_return_band=final_return,
        return_band={
            "pitch_rad": math.radians(0.1),
            "pitch_rate_rad_s": 0.01,
            "height_m": 0.002,
            "joint_change_rad": 0.002,
        },
        last_outside_band_after_clear_s=max(outside) if outside else None,
        response_detected=detected,
        continuous_contact_evidence=contacts_ok,
        passed=final_return and contacts_ok and (config.force_x == 0 or detected),
        measurement_note="Torque messages have no stamp: command_reception_clock is receipt-time ROS clock, not actuation time. Controller terms are separately logged in controller.jsonl.",
        interpretation="Small-disturbance response under joint posture control; no IMU-based balance feedback. No independently measured force or motor torque.",
    )
    return report
