"""Record controller references and measurements without changing commands."""

import json
import time
from pathlib import Path


def seconds(duration):
    return duration.sec + duration.nanosec / 1e9


class TrackingRecorder:
    def __init__(self, filename=""):
        self.path = Path(filename) if filename else None
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("w") if self.path else None
        self.phase = "waiting"
        self.count = 0
        self.worst = None
        self.previous = None
        self.max_sim_gap = 0.0
        self.max_wall_gap = 0.0
        self.first = None

    def record(self, message, clock_time):
        sample = {
            "wall_time": time.monotonic(),
            "controller_time": seconds(message.header.stamp),
            "client_clock": clock_time,
            "phase": self.phase,
            "joint_names": list(message.joint_names),
            "desired": list(message.reference.positions),
            "actual": list(message.feedback.positions),
            "error": list(message.error.positions),
            "output": list(message.output.positions),
            "desired_velocity": list(message.reference.velocities),
            "actual_velocity": list(message.feedback.velocities),
            "trajectory_time": seconds(message.reference.time_from_start),
        }
        if self.first is None:
            self.first = sample
        if self.previous:
            self.max_sim_gap = max(
                self.max_sim_gap,
                sample["controller_time"] - self.previous["controller_time"],
            )
            self.max_wall_gap = max(
                self.max_wall_gap, sample["wall_time"] - self.previous["wall_time"]
            )
        self.previous = sample
        self.count += 1
        if sample["error"]:
            index = max(
                range(len(sample["error"])), key=lambda i: abs(sample["error"][i])
            )
            if self.worst is None or abs(sample["error"][index]) > abs(
                self.worst["error"]
            ):
                self.worst = {
                    "joint": sample["joint_names"][index],
                    "error": sample["error"][index],
                    "desired": sample["desired"][index],
                    "actual": sample["actual"][index],
                    "controller_time": sample["controller_time"],
                    "client_clock": clock_time,
                }
        if self.stream:
            self.stream.write(json.dumps(sample) + "\n")
            self.stream.flush()

    def finish(self, result, metadata):
        report = {
            "result": result,
            "samples": self.count,
            "worst_tracking": self.worst,
            "max_controller_timestamp_gap": self.max_sim_gap,
            "max_callback_wall_gap": self.max_wall_gap,
            "metadata": metadata,
        }
        if self.stream:
            self.stream.close()
        if self.path:
            self.path.with_suffix(".summary.json").write_text(
                json.dumps(report, indent=2) + "\n"
            )
        return report
