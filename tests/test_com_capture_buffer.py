"""Slow marker drawing must not discard independent physics observations."""

import importlib.util, io, json, threading, unittest
from collections import deque
from pathlib import Path
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


class CaptureTests(unittest.TestCase):
    def test_delayed_draw_flushes_every_original_stamp(self):
        spec = importlib.util.spec_from_file_location(
            "com_observer_for_test",
            ROOT / "src/my_robot_description/nodes/sensing/show_center_of_mass.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node = object.__new__(module.ComObserver)
        node.pending = deque()
        node.lock = threading.Lock()
        node.capture_last = None
        node.contacts = {"left": deque(), "right": deque()}
        node.output = io.StringIO()
        node.accepted_states = 0
        with patch.object(
            module,
            "decode_com_state",
            side_effect=lambda data: dict(
                sim_time=data[0],
                mass_kg=22.28,
                com=[0, 0, 0.35],
                feet={"left": [], "right": []},
            ),
        ):
            for t in [1.0, 1.02, 1.04, 1.06, 1.08, 1.10, 1.12]:
                message = type("Message", (), {"data": [t]})()
                node.on_state(message)
        self.assertEqual(node.accepted_states, 7)
        with patch.object(module, "geometric_support", return_value={}):
            node.flush_capture()
            node.flush_capture()
        rows = [json.loads(l) for l in node.output.getvalue().splitlines()]
        self.assertEqual(
            [r["sim_time"] for r in rows], [1.0, 1.02, 1.04, 1.06, 1.08, 1.10, 1.12]
        )
        self.assertFalse(node.pending)

    def test_duplicate_stamp_does_not_silently_create_valid_coverage(self):
        spec = importlib.util.spec_from_file_location(
            "com_observer_duplicate_test",
            ROOT / "src/my_robot_description/nodes/sensing/show_center_of_mass.py",
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node = object.__new__(module.ComObserver)
        node.pending = deque([({"sim_time": 1.0}, 0.0)])
        node.lock = threading.Lock()
        node.capture_last = 1.0
        node.contacts = {}
        node.output = io.StringIO()
        with self.assertRaisesRegex(ValueError, "increase"):
            node.flush_capture()
