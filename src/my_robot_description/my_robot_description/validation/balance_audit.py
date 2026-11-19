"""Check observation coverage, candidate math and unchanged motor composition."""

import math
from my_robot_description.sensing.balance import BalanceConfig, balance_proposal
from my_robot_description.model.torque import clamp_torque
from my_robot_description.control.balance_control import active_correction


def audit_balance(rows, controller, start, end, active=False):
    if not all(math.isfinite(r["sim_time"]) for r in rows):
        raise ValueError("Invalid observer timestamp")
    selected = [r for r in rows if start <= r["sim_time"] <= end]
    if len(selected) < 2 or not controller:
        raise ValueError("Missing balance/controller evidence")
    times = [r["sim_time"] for r in selected]
    gaps = [b - a for a, b in zip(times, times[1:])]
    if min(gaps) <= 0:
        raise ValueError("Observer timestamps must increase")
    residual = 0.0
    for r in selected:
        terms = balance_proposal(
            r["pitch_rad"], r["pitch_rate_rad_s"], BalanceConfig(**r["settings"])
        )
        for key in (
            "pitch_error_rad",
            "proportional_total_nm",
            "derivative_total_nm",
            "raw_per_ankle_nm",
            "proposal_per_ankle_nm",
        ):
            if not math.isfinite(r[key]):
                raise ValueError("Nonfinite balance proposal")
            residual = max(residual, abs(r[key] - terms[key]))
        if (
            len(r["joint_proposal_nm"]) != 6
            or r["proposal_clipped"] != terms["proposal_clipped"]
        ):
            raise ValueError("Invalid proposal vector or clipping flag")
        for a, b in zip(r["joint_proposal_nm"], terms["joint_proposal_nm"]):
            if not math.isfinite(a):
                raise ValueError("Nonfinite proposal vector")
            residual = max(residual, abs(a - b))
        if r["applied"] or r["applied_balance_nm"] != 0:
            raise ValueError("Observer claims an applied correction")
    motor_residual = 0.0
    balance_residual = 0.0
    gated_samples = 0
    peak_applied = 0.0
    caps = [
        6.0,
        10.0,
        6.0,
    ] * 2  # Existing effort-standing profile, not proposal limits.
    for r in controller:
        extra = [0.0] * 6
        if active:
            if not r.get("balance_enabled"):
                raise ValueError("Active recording missing balance mode")
            check = active_correction(
                r["balance_pitch_rad"],
                r["balance_rate_rad_s"],
                r["balance_imu_age"],
                r["balance_contacts"],
                r["balance_elapsed"],
                BalanceConfig(**r["balance_settings"]),
                r["balance_ramp_seconds"],
            )
            extra = check["balance_nm"]
            gated_samples += not check["balance_gate"]
            balance_residual = max(
                balance_residual,
                max(abs(a - b) for a, b in zip(extra, r["balance_nm"])),
            )
            peak_applied = max(peak_applied, max(abs(x) for x in extra))
        for i in range(6):
            expected = (
                r["feedforward"][i]
                + r["proportional"][i]
                + r["damping"][i]
                + r["integral"][i]
                + extra[i]
            )
            if not all(
                math.isfinite(x) for x in (expected, r["requested"][i], r["command"][i])
            ):
                raise ValueError("Invalid controller evidence")
            motor_residual = max(
                motor_residual,
                abs(expected - r["requested"][i]),
                abs(clamp_torque(expected, caps[i]) - r["command"][i]),
            )
    coverage = (
        times[0] - start <= 0.05 and end - times[-1] <= 0.05 and max(gaps) <= 0.05
    )
    return dict(
        passed=coverage
        and residual < 1e-12
        and motor_residual < 1e-9
        and balance_residual < 1e-12
        and gated_samples == 0,
        samples=len(selected),
        simulated_span_s=times[-1] - times[0],
        max_imu_stamp_gap_s=max(gaps),
        max_proposal_formula_error_nm=residual,
        max_motor_composition_error_nm=motor_residual,
        peak_proposal_per_ankle_nm=max(
            abs(r["proposal_per_ankle_nm"]) for r in selected
        ),
        clipped_samples=sum(r["proposal_clipped"] for r in selected),
        applied_balance_nm=None if active else 0.0,
        active_balance=active,
        peak_requested_balance_nm=peak_applied,
        gated_controller_samples=gated_samples,
        max_balance_composition_error_nm=balance_residual,
        bounded_motor_command_checked=True,
        motor_caps_nm=caps,
        note="Request and total motor clamp audited; active mode includes the gated IMU ankle correction, observer itself remains read-only.",
    )
