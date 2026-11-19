"""Regression checks for test zero load. See the folder README for its role."""

import copy, os, subprocess, sys, unittest
from pathlib import Path
from unittest.mock import patch
from test_foot_loads import sample
from my_robot_description.sensing.contact_state import contact_state
from my_robot_description.motion.zero_load import zero_load_guard, paired_contact_states

ROOT = Path(__file__).resolve().parents[1]


class ZeroLoadGuardTests(unittest.TestCase):
    def states(self, empty=False):
        left = sample("left", 1.0, 9.81)
        right = sample("right", 1.0, 0.0)
        if empty:
            right["message"]["contacts"] = []
        return {
            s: contact_state(r, 1.0, s) for s, r in [("left", left), ("right", right)]
        }

    def test_fresh_empty_only_allowed_near_zero(self):
        states = self.states(True)
        self.assertTrue(zero_load_guard(states, 1.0, 1.0, 0.0, "left")["eligible"])
        self.assertFalse(zero_load_guard(states, 1.0, 1.0, 0.02, "left")["eligible"])

    def test_supporting_contact_never_optional(self):
        states = self.states(True)
        self.assertFalse(zero_load_guard(states, 1.0, 1.0, 0.0, "right")["eligible"])

    def test_missing_stale_and_wrong_load_reject(self):
        states = self.states()
        self.assertFalse(zero_load_guard({}, 1.0, 1.0, 0.0, "left")["eligible"])
        self.assertFalse(zero_load_guard(states, 1.1, 1.0, 0.0, "left")["eligible"])
        self.assertFalse(zero_load_guard(states, 1.0, 2.0, 0.0, "left")["eligible"])
        states["right"] = contact_state(None, 1.0, "right")
        self.assertFalse(zero_load_guard(states, 1.0, 1.0, 0.0, "left")["eligible"])

    def test_pairing_fresh_empty_has_no_invented_pressure(self):
        rows = []
        for i in range(10):
            t = 1 + i * 0.01
            left = sample("left", t, 9.81)
            right = sample("right", t, 0.0)
            right["message"]["contacts"] = []
            rows.extend([left, right])
        pairs = paired_contact_states(rows, 1.0, 1.09)
        self.assertTrue(
            all(
                r["right"]["state"] == "no_contact"
                and r["right"]["cop_world_xy_m"] is None
                for r in pairs
            )
        )
        self.assertTrue(all(abs(r["cop_world_xy_m"][1] - 0.085) < 1e-12 for r in pairs))
        with self.assertRaises(ValueError):
            paired_contact_states([r for r in rows if r["side"] == "left"], 1.0, 1.09)


class ZeroLoadPlanTests(unittest.TestCase):
    def test_mirrored_path_and_force_margin_audit(self):
        code = """
import math,xacro,copy
from unittest.mock import patch
from my_robot_description.motion.zero_load import ZeroLoadProgram,zero_load_guard,audit_zero_load
from my_robot_description.motion.lowered_unload import LoweredUnloadProgram
from my_robot_description.model.roll_geometry import model_com,link_transforms
from my_robot_description.model.roll_stance import JOINT_NAMES,MOTOR_CAPS
from my_robot_description.sensing.contact_state import contact_state
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml()
programs=[ZeroLoadProgram(xml,s) for s in ['left','right']]
assert abs(programs[0].displacement+programs[1].displacement)<1e-9
for p in programs:
 assert .13<abs(p.displacement)<.14
 assert p.setup['total_time_s']==233.
 previous=None
 for i in range(1166):
  r=p.sample(i*.2);q=dict(zip(JOINT_NAMES,r['target_rad']));tf=link_transforms(xml,q);m,c=model_com(xml,q)
  for side in ['left','right']:
   f=tf[side+'_foot'];old=p.planner.feet[side+'_foot']
   assert max(abs(f[k][3]+[r['pelvis_x_m'],r['pelvis_y_m'],r['pelvis_height_m']-p.planner.base['expected_pelvis_height_m']][k]-old[k][3]) for k in range(3))<1e-6
  assert all(abs(t)<.8*cap for t,cap in zip(r['feedforward_nm'],MOTOR_CAPS))
  assert abs(c[1]+r['pelvis_y_m']-r['predicted_com_y_m'])<1e-6
  if previous:assert max(abs(a-b)/.2 for a,b in zip(r['target_rad'],previous))<.05
  previous=r['target_rad']
 assert p.sample(233.)['target_rad']==p.sample(0.)['target_rad']
 from my_robot_description.control.roll_posture import KD
 assert p.derivative_gains(0.)==KD and p.derivative_gains(233.)==KD
 gains=p.derivative_gains(116.);index=9 if p.setup['unloaded_foot']=='right' else 4
 assert abs(gains[index]-.3)<1e-12
 offset=0 if p.setup['supporting_foot']=='left' else 5
 assert all(gains[i]==KD[i] for i in range(10) if i not in [index,offset+3,offset+4])
 assert gains[offset+3]==8. and gains[offset+4]==8.
 from my_robot_description.control.roll_posture import KP
 assert p.proportional_gains(0.)==KP and p.proportional_gains(233.)==KP
 assert p.proportional_gains(116.)[offset+3]==120. and p.proportional_gains(116.)[offset+4]==120.
 assert all(.3-1e-12<=p.derivative_gains(i*.5)[index]<=2. for i in range(467))
 assert abs(p.expected_fraction(116.))<1e-9
try:LoweredUnloadProgram(xml,0.)
except ValueError:pass
else:raise AssertionError('Legacy unloading accepted zero')
p=programs[0];weight=p.planner.base['mass_kg']*9.81;pairs=[];rows=[]
for i in range(5826):
 t=i*.04;r=p.sample(t);forces=r['predicted_force_n'];states={s:dict(side=s,measurement_valid=True,state='loaded_contact' if forces[s]>1 else 'touching_unloaded',sim_time=1+t,force_world_n=[0,0,forces[s]],cop_world_xy_m=[p.planner.foot_x,.085 if s=='left' else -.085]) for s in ['left','right']}
 g=zero_load_guard(states,1+t,p.planner.base['mass_kg'],p.expected_fraction(t),'left')
 rows.append(dict(sim_time=1+t,shift=r,load_guard=g))
 pairs.append(dict(sim_time=1+t,left=states['left'],right=states['right'],total_vertical_n=weight))
corners=[[p.planner.foot_x+dx,.085+dy,0] for dx,dy in [(-.1,-.06),(-.1,.06),(.1,.06),(.1,-.06)]]
com=[dict(sim_time=116+i*.05,com=[p.planner.foot_x,.085,.35],feet=dict(left=corners)) for i in range(40)]
with patch('my_robot_description.motion.zero_load.paired_contact_states',return_value=pairs):
 assert audit_zero_load([],com,rows,1.,234.,p)['passed']
 wrong=copy.deepcopy(com)
 for r in wrong:r['com'][1]=0
 assert not audit_zero_load([],wrong,rows,1.,234.,p)['passed']
wrong=copy.deepcopy(pairs)
for r in wrong:
 if 116<=r['sim_time']<118:r['right']['force_world_n'][2]=.6
with patch('my_robot_description.motion.zero_load.paired_contact_states',return_value=wrong):assert not audit_zero_load([],com,rows,1.,234.,p)['passed']
"""
        subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            check=True,
        )
