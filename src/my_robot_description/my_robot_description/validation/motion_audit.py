"""Shared command and body-sensor checks for single steps and walking.

These functions only read evidence. They do not change sensor samples or send
motor commands. Thresholds are the same as before the folder reorganization.
"""

from dataclasses import dataclass
import json
import math
from my_robot_description.model.roll_stance import MOTOR_CAPS
from my_robot_description.control.roll_posture import LATERAL_KI, INTEGRAL_CAPS


@dataclass(frozen=True)
class JointEvidence:
    composition_error_nm: float
    motor_ok: bool
    live_ok: bool
    peak_speed_rad_s: float
    max_tracking_error_rad: float


@dataclass(frozen=True)
class BodyEvidence:
    passed: bool
    max_pose_error_m: float


def coverage(rows, start, end, gap):
    times = [r["sim_time"] for r in rows]
    return (
        len(times) > 2
        and 0 < min(b - a for a, b in zip(times, times[1:]))
        and max(b - a for a, b in zip(times, times[1:])) <= gap
        and times[0] - start <= gap
        and end - times[-1] <= gap
    )


def audit_joint_feedback(rows, program, start):
    """Recalculate PID + feedforward from logged positions, speeds and gains.

    Comparing the recomputed torque with the requested and clipped commands
    catches wrong target order, wrong gains and silent command saturation.
    """
    residual = 0.0
    motor_ok = True
    live_ok = True
    for r in rows:
        t = r["shift"]["elapsed_s"]
        p = program.sample(t)
        kp = program.proportional_gains(t)
        kd = program.derivative_gains(t)
        live_ok &= (
            r["active"]
            and r["ready"]
            and not r.get("integral_frozen_gap", False)
            and program.controller_contacts_ok(r)
        )
        live_ok &= (
            r.get("integral_gains") == LATERAL_KI
            and r.get("proportional_gains") == kp
            and r.get("derivative_gains") == kd
            and r["shift"]["phase"] == p["phase"]
        )
        if abs(r["sim_time"] - start - t) > 1e-9:
            raise ValueError("First-step clock mismatch")
        for j, cap in enumerate(MOTOR_CAPS):
            e = p["target_rad"][j] - r["position"][j]
            expected = (
                p["feedforward_nm"][j]
                + kp[j] * e
                - kd[j] * r["velocity"][j]
                + r["integral"][j]
            )
            values = [e, expected, r["requested"][j], r["command"][j], r["integral"][j]]
            if not all(math.isfinite(v) for v in values):
                raise ValueError("Nonfinite first-step motor data")
            residual = max(
                residual,
                abs(e - r["error"][j]),
                abs(expected - r["requested"][j]),
                abs(p["target_rad"][j] - r["target_rad"][j]),
                abs(p["feedforward_nm"][j] - r["feedforward"][j]),
                abs(max(-cap, min(cap, expected)) - r["command"][j]),
            )
            motor_ok &= (
                abs(r["command"][j]) <= 0.8 * cap + 1e-9
                and abs(r["integral"][j]) <= INTEGRAL_CAPS[j] + 1e-9
            )
    speed = max(abs(v) for r in rows for v in r["velocity"])
    error = max(abs(e) for r in rows for e in r["error"])
    return JointEvidence(residual, bool(motor_ok), bool(live_ok), speed, error)


def audit_body_measurements(folder, program, start, end):
    """Check raw IMU, pose tracking, source freshness and contact policy.

    All times are simulated seconds. Positions are metres, angles radians,
    angular speeds rad/s and accelerations m/s². Missing coverage fails.
    """

    def read(name):
        return [json.loads(line) for line in (folder / name).read_text().splitlines()]

    sensors = [r for r in read("sensors.jsonl") if start <= r["sim_time"] <= end]
    imu = [r for r in read("imu-raw.jsonl") if start <= r["sim_time"] <= end]
    sensor_ok = coverage(sensors, start, end, 0.1) and coverage(imu, start, end, 0.05)
    pose_error = 0.0
    for r in sensors:
        pose = r["pose"]
        p = program.sample(max(0.0, pose["sim_time"] - start))
        sensor_ok &= all(
            -0.02 <= r["sim_time"] - r[k]["sim_time"] <= (0.1 if k == "pose" else 0.05)
            for k in ("pose", "imu")
        ) and program.sensor_contacts_ok(r, start)
        expected = [p["pelvis_x_m"], p.get("pelvis_y_m", 0.0), p["pelvis_height_m"]]
        pose_error = max(
            pose_error, *(abs(x - y) for x, y in zip(pose["position"], expected))
        )
        sensor_ok &= (
            all(math.isfinite(v) for v in [*pose["position"], pose["tilt_radians"]])
            and pose["tilt_radians"] <= 0.05
        )
    imu_ok = all(
        all(
            math.isfinite(v)
            for v in [r["tilt_radians"], r["angular_speed"], *r["acceleration"]]
        )
        and r["tilt_radians"] <= 0.05
        and r["angular_speed"] <= 0.05
        and max(
            abs(r["acceleration"][0]),
            abs(r["acceleration"][1]),
            abs(r["acceleration"][2] - 9.81),
        )
        <= 0.25
        for r in imu
    )
    sensor_ok &= pose_error <= 0.002 and imu_ok
    return BodyEvidence(bool(sensor_ok), pose_error)
