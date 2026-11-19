"""Small scheduled Gazebo lateral wrench with guaranteed cleanup attempt.

The existing force plugin handles the command. Debug logs confirm queue/clear,
not calibrated force magnitude or exact actuation timing.
"""

import json
import math
import time


def latest_record(path):
    try:
        with path.open("rb") as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - 16000))
            data = f.read()
        for line in reversed(
            data.splitlines()[:-1] if not data.endswith(b"\n") else data.splitlines()
        ):
            try:
                return json.loads(line)
            except (ValueError, UnicodeDecodeError):
                continue
    except OSError:
        pass
    return None


class LateralPulse:
    def __init__(self, folder, force_y, duration, gui=False):
        if (
            not math.isfinite(force_y)
            or not 0 < abs(force_y) <= 1
            or not math.isfinite(duration)
            or not 0.02 <= duration <= 0.2
        ):
            raise ValueError(
                "Lateral test pulse requires 0<|force_y|≤1 N, duration 0.02–0.2 s"
            )
        from gz.transport13 import Node
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.entity_wrench_pb2 import EntityWrench

        self.folder = folder
        self.force = force_y
        self.duration = duration
        self.events = []
        self.pushed = None
        self.cleared = False
        self.observed = {}
        self.started_wall = None
        self.node = Node()
        self.push = self.node.advertise(
            "/world/standing/wrench/persistent", EntityWrench
        )
        self.clear = self.node.advertise("/world/standing/wrench/clear", Entity)
        self.entity = Entity(name="my_biped", type=Entity.MODEL)
        self.command = EntityWrench()
        self.command.entity.CopyFrom(self.entity)
        self.command.wrench.force.y = force_y
        from my_robot_description.visualization.force_visual import ForceArrowVisualizer

        self.visual = ForceArrowVisualizer(axis="y") if gui else None

    def event(self, name, t):
        self.events.append(dict(event=name, sim_time=t, wall_time=time.monotonic()))
        (self.folder / "pulse-events.json").write_text(
            json.dumps(self.events, indent=2) + "\n"
        )
        print("[lateral-pulse] " + name + " " + str(t), flush=True)

    def tick(self, messages):
        for name, marker in [
            ("queued", "Queueing persistent wrench"),
            ("cleared", "Clearing persistent wrench for entity"),
        ]:
            if marker in messages and name not in self.observed:
                self.observed[name] = time.monotonic()
        r = latest_record(self.folder / "joints.jsonl")
        if not r or r.get("shift") is None:
            return
        t = r["sim_time"]
        elapsed = r["shift"]["elapsed_s"]
        if self.pushed is None and elapsed >= 13.0:
            if not self.push.has_connections():
                raise RuntimeError("Gazebo wrench subscriber missing; no pulse applied")
            if self.push.publish(self.command) is False:
                raise RuntimeError("Wrench publication failed")
            self.pushed = t
            self.started_wall = time.monotonic()
            self.event("push_published", t)
        if self.pushed is not None and not self.cleared:
            if self.visual:
                sensor = latest_record(self.folder / "sensors.jsonl")
                if sensor:
                    xyz = sensor["pose"]["position"]
                    self.visual.update(
                        self.force, dict(x=xyz[0], y=xyz[1], height=xyz[2])
                    )
            if t - self.pushed >= self.duration:
                self.clear.publish(self.entity)
                self.cleared = True
                self.event("clear_published", t)
                if self.visual:
                    self.visual.hide()
            elif time.monotonic() - self.started_wall > 10:
                raise TimeoutError(
                    "Pulse clock stalled; runner cleanup will clear force"
                )

    def report(self):
        window = self.events[-1]["sim_time"] - self.pushed if self.cleared else None
        passed = (
            self.cleared
            and set(self.observed) == {"queued", "cleared"}
            and self.duration <= window <= self.duration + 0.03
        )
        return dict(
            passed=bool(passed),
            force_y_n=self.force,
            requested_duration_s=self.duration,
            scheduled_program_time_s=13.0,
            events=self.events,
            command_window_s=window,
            nominal_command_impulse_n_s=None if window is None else self.force * window,
            queued_in_gazebo="queued" in self.observed,
            cleared_in_gazebo="cleared" in self.observed,
            visual=None if self.visual is None else self.visual.status(),
            note="Command timing and Gazebo handling evidence; force calibration and exact actuation timing are not measured.",
        )

    def close(self):
        self.clear.publish(self.entity)
        if self.visual:
            self.visual.close()
