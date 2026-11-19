"""Bounds and interpretation of recorded push responses."""

import math
import unittest
from my_robot_description.experiments.disturbance import (
    PushConfig,
    summarize,
    process_sensor_callbacks,
)


def sample(time, phase, pitch=0, contact=True):
    return {
        "sim_time": time,
        "phase": phase,
        "pitch": pitch,
        "pitch_rate": 0,
        "height": 0.55449,
        "joint_error": [0.001] * 6,
        "acceleration_x": 0,
        "contacts": {
            s: {"ground_contact": contact, "age": 0.01} for s in ("left", "right")
        },
    }


class PushExperimentTests(unittest.TestCase):
    def test_clock_callback_does_not_hide_pending_fresh_sensor_data(self):
        state = {"clock": 1.0, "imu": 1.0, "joints": 1.0}
        callbacks = [
            lambda: state.update(clock=1.105),
            lambda: state.update(imu=1.10),
            lambda: state.update(joints=1.104),
        ]

        class Executor:
            def spin_once(self, timeout_sec):
                if callbacks:
                    callbacks.pop(0)()

        process_sensor_callbacks(Executor())
        self.assertLess(state["clock"] - state["imu"], 0.1)
        self.assertLess(state["clock"] - state["joints"], 0.1)

    def test_ready_processing_is_bounded_under_continuous_traffic(self):
        class Executor:
            count = 0

            def spin_once(self, timeout_sec):
                self.count += 1

        executor = Executor()
        process_sensor_callbacks(executor, max_ready=8)
        self.assertEqual(executor.count, 9)

    def test_ros_and_gazebo_timestamp_fields(self):
        from types import SimpleNamespace
        from scripts.experiments.push_experiment import stamp

        self.assertEqual(stamp(SimpleNamespace(sec=2, nanosec=500000000)), 2.5)
        self.assertEqual(stamp(SimpleNamespace(sec=2, nsec=500000000)), 2.5)

    def test_invalid_pushes_are_rejected(self):
        for config in (
            PushConfig(force_x=10000),
            PushConfig(duration=0),
            PushConfig(force_x=float("nan")),
        ):
            with self.assertRaises(ValueError):
                config.validate()

    def test_peak_is_relative_to_pre_push_orientation(self):
        samples = [
            sample(0, "baseline", 0.01),
            sample(2, "push", 0.03),
            sample(3, "recovery", 0.01),
            sample(5, "recovery", 0.01),
        ]
        events = [
            {"event": "push_published", "sim_time": 2},
            {"event": "clear_published", "sim_time": 2.2},
        ]
        report = summarize(samples, events, PushConfig())
        self.assertAlmostEqual(report["peak_pitch_change_rad"], 0.02)
        self.assertAlmostEqual(report["command_window_s"], 0.2)
        self.assertTrue(report["final_second_within_return_band"])

    def test_unknown_contact_is_not_reported_as_confirmed_contact_loss(self):
        samples = [
            sample(0, "baseline"),
            sample(2, "push", contact=None),
            sample(4, "recovery"),
        ]
        events = [
            {"event": "push_published", "sim_time": 2},
            {"event": "clear_published", "sim_time": 2.2},
        ]
        report = summarize(samples, events, PushConfig())
        self.assertEqual(report["contacts"]["left"]["unknown_samples"], 1)
        self.assertEqual(report["contacts"]["left"]["fresh_non_ground_samples"], 0)

    def test_missing_post_clear_samples_are_not_a_completed_experiment(self):
        events = [
            {"event": "push_published", "sim_time": 2},
            {"event": "clear_published", "sim_time": 2.2},
        ]
        with self.assertRaisesRegex(ValueError, "post-clear"):
            summarize([sample(0, "baseline"), sample(2, "push")], events, PushConfig())

    def test_response_outside_final_band_is_not_called_returned(self):
        samples = [
            sample(0, "baseline"),
            sample(2, "push", 0.02),
            sample(5, "recovery", 0.02),
        ]
        events = [
            {"event": "push_published", "sim_time": 2},
            {"event": "clear_published", "sim_time": 2.2},
        ]
        self.assertFalse(
            summarize(samples, events, PushConfig())["final_second_within_return_band"]
        )
