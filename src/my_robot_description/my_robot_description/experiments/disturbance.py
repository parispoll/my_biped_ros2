"""Configuration and analysis for bounded sagittal push experiments."""

from dataclasses import dataclass, asdict
import math


@dataclass(frozen=True)
class PushConfig:
    force_x: float = 5.0
    duration: float = 0.2
    baseline: float = 2.0
    recovery: float = 10.0
    contact_max_age: float = 0.05

    def validate(self):
        if not all(math.isfinite(value) for value in asdict(self).values()):
            raise ValueError("Experiment values must be finite")
        if abs(self.force_x) > 100 or not 0.02 <= self.duration <= 0.5:
            raise ValueError(
                "This baseline experiment supports |force_x| <= 100 N and duration 0.02–0.5 s"
            )
        if (
            not 1 <= self.baseline <= 100
            or not 2 <= self.recovery <= 20
            or not 0 < self.contact_max_age <= 0.1
        ):
            raise ValueError("Invalid baseline, recovery or contact freshness interval")
        return self


def summarize(samples, events, config):
    """Compare response with its own pre-push baseline, without claiming active balance."""
    baseline = [s for s in samples if s["phase"] == "baseline"]
    response = [s for s in samples if s["phase"] in ("push", "recovery")]
    if not baseline or not response:
        raise ValueError("Missing baseline or response samples")
    pitch0 = sum(s["pitch"] for s in baseline) / len(baseline)
    height0 = sum(s["height"] for s in baseline) / len(baseline)
    released = next(e["sim_time"] for e in events if e["event"] == "clear_published")
    recovery = [s for s in response if s["sim_time"] >= released]
    if not recovery:
        raise ValueError("Missing post-clear response samples")
    band = math.radians(0.5)

    def in_band(sample):
        return (
            abs(sample["pitch"] - pitch0) <= band
            and abs(sample["pitch_rate"]) <= 0.02
            and abs(sample["height"] - height0) <= 0.005
        )

    outside = [s["sim_time"] for s in recovery if not in_band(s)]
    tail = [s for s in recovery if s["sim_time"] >= recovery[-1]["sim_time"] - 1]
    final_within = bool(tail) and all(in_band(s) for s in tail)
    start = next(e["sim_time"] for e in events if e["event"] == "push_published")
    return {
        "requested": asdict(config),
        "events": events,
        "recorded_samples": len(samples),
        "command_window_s": released - start,
        "nominal_command_impulse_n_s": config.force_x * (released - start),
        "timing_note": "ROS/Gazebo transport adds unmeasured actuation delay; command window is not measured force duration.",
        "baseline_pitch_rad": pitch0,
        "baseline_height_m": height0,
        "peak_pitch_change_rad": max(abs(s["pitch"] - pitch0) for s in response),
        "peak_pitch_rate_rad_s": max(abs(s["pitch_rate"]) for s in response),
        "min_height_m": min(s["height"] for s in response),
        "max_joint_tracking_error_rad": max(
            max(abs(e) for e in s["joint_error"]) for s in samples
        ),
        "peak_horizontal_acceleration_m_s2": max(
            abs(s["acceleration_x"]) for s in response
        ),
        "final_second_within_return_band": final_within,
        "return_band": {"pitch_rad": band, "pitch_rate_rad_s": 0.02, "height_m": 0.005},
        "last_outside_band_after_clear_s": max(outside) - released if outside else None,
        "contacts": {
            side: {
                "unknown_samples": sum(
                    s["contacts"][side]["ground_contact"] is None for s in samples
                ),
                "fresh_non_ground_samples": sum(
                    s["contacts"][side]["ground_contact"] is False for s in samples
                ),
                "max_age_s": max(s["contacts"][side]["age"] for s in samples),
            }
            for side in ("left", "right")
        },
    }


def process_sensor_callbacks(executor, max_ready=32):
    """Take a bounded batch before comparing timestamps from separate topics.

    A clock callback can run before queued sensor updates. Processing only that
    callback can falsely classify a pending fresh measurement as stale. The
    zero-timeout calls process ready work without waiting for future messages.
    The cap keeps the force-clearing/timeout logic from being starved by traffic.
    """
    executor.spin_once(timeout_sec=0.002)
    for _ in range(max_ready):
        executor.spin_once(timeout_sec=0.0)
