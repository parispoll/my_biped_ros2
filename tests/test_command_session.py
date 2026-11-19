"""Commands may stop only at a completed double-support boundary."""

import unittest
from types import SimpleNamespace
from my_robot_description.runtime.command_session import CommandSession


class SessionTests(unittest.TestCase):
    def make(self):
        return CommandSession(
            SimpleNamespace(steps=[dict(end=v) for v in [100, 200, 300]])
        )

    def test_idle_start_stop_resume_and_capacity(self):
        s = self.make()
        s.update(0, False)
        s.update(30, True)
        self.assertEqual(s.elapsed, 0)
        s.command("walk", 3, True)
        s.update(60, True)
        s.command("stop", 1, True)
        s.update(160, True)
        self.assertFalse(s.running)
        self.assertEqual(s.completed, 1)
        hold = s.elapsed
        s.update(500, True)
        self.assertEqual(s.elapsed, hold)
        s.command("step", 1, True)
        s.update(610, True)
        self.assertEqual(s.completed, 2)
        self.assertFalse(s.running)
        s.command("step", 1, True)
        s.update(720, True)
        self.assertEqual(s.completed, 3)
        with self.assertRaises(ValueError):
            s.command("step", 1, True)

    def test_reject_unready_busy_bad_commands(self):
        s = self.make()
        with self.assertRaises(ValueError):
            s.command("step", 1, False)
        for n in [0, -1, True, 4, 1.5]:
            with self.assertRaises(ValueError):
                s.command("walk", n, True)
        with self.assertRaises(ValueError):
            s.command("turn", 1, True)
        s.command("step", 1, True)
        with self.assertRaises(ValueError):
            s.command("walk", 1, True)
        self.assertEqual(s.status(True)["state"], "walking")
