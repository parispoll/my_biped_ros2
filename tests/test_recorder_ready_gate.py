"""Exercise the recorder loop: fresh sensors alone must not arm a gated push."""

import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as S
import unittest
from unittest.mock import Mock, patch
from my_robot_description.experiments.disturbance import PushConfig

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import scripts.experiments.push_experiment as recorder


class RecorderGateTests(unittest.TestCase):
    def exercise(self, root, gate_mode):
        callbacks = {}
        state = {}
        publishers = {}
        pose_callback = []
        counter = [0]
        wall = [0.0]
        node = Mock()
        node.create_subscription.side_effect = (
            lambda kind, topic, callback, qos: callbacks.update({topic: callback})
        )
        transport = Mock()

        def advertise(topic, kind):
            pub = Mock()
            pub.has_connections.return_value = True
            pub.publish.return_value = True
            publishers[topic] = pub
            return pub

        transport.advertise.side_effect = advertise
        transport.subscribe.side_effect = (
            lambda kind, topic, callback: pose_callback.append(callback)
        )
        visual = Mock()
        visual.status.return_value = {}
        gate = None if gate_mode == "legacy" else root / "ready"

        def stamp(t):
            return S(
                sec=int(t),
                nanosec=round((t - int(t)) * 1e9),
                nsec=round((t - int(t)) * 1e9),
            )

        def pump(executor):
            counter[0] += 1
            t = counter[0] * 0.01
            if gate_mode == "delayed" and counter[0] == 100:
                gate.write_text("ready")
            callbacks["/clock"](S(clock=stamp(t)))
            callbacks["/biped/imu"](
                S(
                    header=S(frame_id="base_link", stamp=stamp(t)),
                    orientation=S(x=0.0, y=0.0, z=0.0, w=1.0),
                    angular_velocity=S(y=0.0),
                    linear_acceleration=S(x=0.0),
                )
            )
            for side in ("left", "right"):
                callbacks[f"/biped/{side}_foot/contacts"](S(header=S(stamp=stamp(t))))
            pose_callback[0](
                S(
                    header=S(stamp=stamp(t)),
                    pose=[S(name="my_biped", position=S(x=0.0, y=0.0, z=0.55))],
                )
            )
            state["latest"]["joints"] = dict(
                time=t, names=[], desired=[0.0], actual=[0.0], error=[0.0]
            )

        def attach(node, latest, errors, executor):
            state["latest"] = latest

        def monotonic():
            wall[0] += 0.001
            return wall[0]

        config = PushConfig(force_x=1.0, duration=0.02, baseline=1.0, recovery=2.0)
        with patch.object(recorder, "RosNode", return_value=node), patch.object(
            recorder, "SingleThreadedExecutor"
        ), patch.object(recorder, "GazeboNode", return_value=transport), patch.object(
            recorder, "ForceArrowVisualizer", return_value=visual
        ), patch.object(
            recorder, "ground_contact_pairs", return_value=True
        ), patch.object(
            recorder, "process_sensor_callbacks", side_effect=pump
        ), patch.object(
            recorder.time, "monotonic", side_effect=monotonic
        ), contextlib.redirect_stdout(
            io.StringIO()
        ):
            try:
                recorder.run(
                    config,
                    root / "recording",
                    joint_subscription=attach,
                    analyzer=lambda *args: dict(passed=True),
                    wall_timeout=1.0 if gate_mode == "closed" else 10.0,
                    ready_file=gate,
                )
            except TimeoutError:
                if gate_mode != "closed":
                    raise
        import json

        result = json.loads((root / "recording/summary.json").read_text())
        return result, publishers

    def test_closed_gate_never_publishes_force_despite_fresh_sensors(self):
        with tempfile.TemporaryDirectory() as directory:
            result, pubs = self.exercise(Path(directory), "closed")
            self.assertFalse(result["completed"])
            pubs["/world/standing/wrench/persistent"].publish.assert_not_called()
            self.assertGreater(
                pubs["/world/standing/wrench/clear"].publish.call_count, 0
            )

    def test_gate_release_requires_new_fresh_baseline_before_push(self):
        with tempfile.TemporaryDirectory() as directory:
            result, pubs = self.exercise(Path(directory), "delayed")
            import json

            events = json.loads((Path(directory) / "recording/events.json").read_text())
            self.assertTrue(result["completed"])
            self.assertGreaterEqual(events[0]["sim_time"], 1.5)
            self.assertGreaterEqual(
                next(e["sim_time"] for e in events if e["event"] == "push_published"),
                2.5,
            )
            pubs["/world/standing/wrench/persistent"].publish.assert_called_once()

    def test_ungated_manual_recorder_keeps_existing_behavior(self):
        with tempfile.TemporaryDirectory() as directory:
            result, pubs = self.exercise(Path(directory), "legacy")
            self.assertTrue(result["completed"])
            pubs["/world/standing/wrench/persistent"].publish.assert_called_once()
