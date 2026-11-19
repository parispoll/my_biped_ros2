"""Behavioral tests of bounded measured-share corrections and command geometry."""

import copy
import os
from pathlib import Path
import subprocess
import sys
import unittest
from my_robot_description.motion.load_feedback import (
    LoadShareFeedback,
    OFFSET_CAP_M,
    RATE_CAP_M_S,
)
from my_robot_description.motion.partial_unload import load_guard

ROOT = Path(__file__).resolve().parents[1]


def guard(left, t=1.0):
    w = 22.28 * 9.81
    return load_guard(
        {
            s: dict(sim_time=t, force_world_n=[0.0, 0.0, w * f])
            for s, f in [("left", left), ("right", 1 - left)]
        },
        t + 0.001,
        22.28,
    )


class LoadFeedbackTests(unittest.TestCase):
    def test_sign_filter_disabled_and_normalized_shares(self):
        a = LoadShareFeedback()
        r = a.update(0.7, guard(0.71), 0.01)
        self.assertLess(r["correction_m"], 0)
        self.assertGreater(r["filtered_error_fraction"], r["error_fraction"])
        b = LoadShareFeedback()
        self.assertGreater(b.update(0.3, guard(0.29), 0.01)["correction_m"], 0)
        off = LoadShareFeedback(False)
        for _ in range(20):
            self.assertEqual(off.update(0.7, guard(0.71), 0.01)["correction_m"], 0.0)

    def test_offset_rate_caps_and_recovery_from_clipping(self):
        c = LoadShareFeedback()
        previous = 0.0
        for _ in range(400):
            r = c.update(0.7, guard(0.5), 0.01)
            self.assertLessEqual(
                abs(r["correction_m"] - previous), RATE_CAP_M_S * 0.01 + 1e-12
            )
            self.assertLessEqual(abs(r["correction_m"]), OFFSET_CAP_M)
            previous = r["correction_m"]
        self.assertEqual(c.offset, OFFSET_CAP_M)
        for _ in range(600):
            c.update(0.5, guard(0.7), 0.01)
        self.assertEqual(c.offset, -OFFSET_CAP_M)

    def test_known_quasistatic_bias_is_reduced_without_changing_reference(self):
        c = LoadShareFeedback()
        bias = 0.003
        sensitivity = 0.2 / 0.052916782237778535
        initial = abs(sensitivity * bias)
        for _ in range(2400):
            measured = 0.7 + sensitivity * (bias + c.offset)
            r = c.update(0.7, guard(measured), 0.01)
        self.assertLess(abs(r["error_fraction"]), initial * 0.1)
        self.assertAlmostEqual(c.offset, -bias, delta=0.0003)
        self.assertEqual(r["target_left_fraction"], 0.7)

    def test_invalid_input_cannot_update_state(self):
        c = LoadShareFeedback()
        c.update(0.7, guard(0.71), 0.01)
        before = c.offset
        for g, dt in [
            (None, 0.01),
            (dict(eligible=False), 0.01),
            (guard(0.71), 0.1),
            (guard(0.71), float("nan")),
        ]:
            with self.assertRaises(ValueError):
                c.update(0.7, g, dt)
            self.assertEqual(c.offset, before)
        g = guard(0.71)
        g["ages_s"]["left"] = 0.051
        with self.assertRaises(ValueError):
            c.update(0.7, g, 0.01)
        self.assertEqual(c.offset, before)
        self.assertEqual(c.update(0.7, guard(0.71), 0.0)["correction_m"], before)

    def test_command_bias_cancellation_flat_feet_and_replay_tamper_detection(self):
        code = """
import copy,xacro
from my_robot_description.motion.load_feedback import LoadTransferProgram,LoadShareFeedback,audit_load_feedback
from my_robot_description.motion.partial_unload import load_guard
from my_robot_description.model.roll_stance import JOINT_NAMES
from my_robot_description.model.roll_geometry import link_transforms
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml()
p=LoadTransferProgram(xml,.3,'left',True,.003)
a=p.sample(15.);b=p.command_sample(15.,-.003)
assert max(abs(x-y) for x,y in zip(a['target_rad'],b['target_rad']))<1e-6
frames=link_transforms(xml,dict(zip(JOINT_NAMES,b['target_rad'])))
assert abs(frames['left_foot'][1][3]+b['pelvis_y_m']-.085)<1e-6
assert abs(p.audit_displacement-p.displacement)<1e-12
q=LoadTransferProgram(xml,.3,'left',False,.003)
assert abs(q.audit_displacement-q.displacement-.003)<1e-12
c=LoadShareFeedback();rows=[];w=22.28*9.81
for i in range(21):
 t=i*.012;nom=p.sample(t);target=nom['predicted_force_n']['left']/w
 g=load_guard({s:dict(sim_time=t,force_world_n=[0,0,w*f]) for s,f in [('left',.51),('right',.49)]},t+.001,22.28)
 state=c.update(target,g,0. if i==0 else .012)
 rows.append(dict(sim_time=t,shift=nom,load_guard=g,load_feedback=state))
assert audit_load_feedback(rows,p,0.,.24)['passed']
bad=copy.deepcopy(rows);bad[5]['load_feedback']['correction_m']+=.001
try:audit_load_feedback(bad,p,0.,.24)
except ValueError:pass
else:raise AssertionError('Corrupted correction accepted')
"""
        result = subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
