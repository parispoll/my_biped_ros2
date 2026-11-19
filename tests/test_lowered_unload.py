"""Positive-load planning, target-specific guards and synthetic audit rejection."""

import os, subprocess, sys, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class LoweredUnloadTests(unittest.TestCase):
    def test_geometry_speed_and_measured_goal(self):
        code = """
import xacro,math,copy,json
from unittest.mock import patch
from my_robot_description.motion.lowered_unload import LoweredUnloadProgram,audit_unloading_goal
from my_robot_description.motion.lowered_shift import LoweredShiftProgram
from my_robot_description.model.roll_stance import JOINT_NAMES,MOTOR_CAPS
from my_robot_description.model.roll_geometry import link_transforms,model_com
from my_robot_description.motion.partial_unload import load_guard
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml()
previews=[]
for target in [.2,.1,.05]:
 endpoints=[]
 for side in ['left','right']:
  p=LoweredUnloadProgram(xml,target,side);end=p.lateral[-1];w=p.planner.base['mass_kg']*9.81
  assert abs(end['predicted_force_n'][p.setup['unloaded_foot']]/w-target)<1e-8
  assert p.setup['total_time_s']==233.
  previous=None
  for i in range(1166):
   r=p.sample(i*.2);q=dict(zip(JOINT_NAMES,r['target_rad']));tf=link_transforms(xml,q);mass,c=model_com(xml,q)
   for foot in ['left','right']:
    f=tf[foot+'_foot'];old=p.planner.feet[foot+'_foot']
    assert max(abs(f[k][3]+[r['pelvis_x_m'],r['pelvis_y_m'],r['pelvis_height_m']-p.planner.base['expected_pelvis_height_m']][k]-old[k][3]) for k in range(3))<1e-6
   assert abs(c[1]+r['pelvis_y_m']-r['predicted_com_y_m'])<1e-6
   assert all(abs(t)<.8*cap for t,cap in zip(r['feedforward_nm'],MOTOR_CAPS))
   if previous:assert max(abs(a-b)/.2 for a,b in zip(r['target_rad'],previous))<.05
   previous=r['target_rad']
  assert p.sample(233.)['target_rad']==p.sample(0)['target_rad']
  loads={s:dict(sim_time=1.,force_world_n=[0,0,end['predicted_force_n'][s]]) for s in ['left','right']}
  assert load_guard(loads,1.,mass,p.minimum_measured_share)['eligible']
  assert not load_guard(loads,1.,mass)['eligible']
  assert not load_guard(loads,1.1,mass,p.minimum_measured_share)['eligible']
  lo,hi=p.setup['analysis_windows']['hold'];foot=p.setup['supporting_foot'];corners=[[p.planner.foot_x+dx,p.planner.feet[foot+'_foot'][1][3]+dy,0] for dx,dy in [(-.1,-.06),(-.1,.06),(.1,.06),(.1,-.06)]]
  com=[dict(sim_time=lo+i*.05,com=[p.planner.foot_x,end['predicted_com_y_m'],.35],feet={foot:corners}) for i in range(40)]
  pairs=[dict(sim_time=lo+i*.02,left=dict(force_world_n=[0,0,end['predicted_force_n']['left']]),right=dict(force_world_n=[0,0,end['predicted_force_n']['right']]),total_vertical_n=w) for i in range(100)]
  with patch('my_robot_description.sensing.foot_loads.paired_loads',return_value=pairs):
   assert audit_unloading_goal([],com,0.,p)['passed']
   wrong=copy.deepcopy(com)
   for r in wrong:r['com'][1]=0.
   assert not audit_unloading_goal([],wrong,0.,p)['passed']
  wrong=copy.deepcopy(pairs)
  for r in wrong:r[p.setup['unloaded_foot']]['force_world_n'][2]=.35*w
  with patch('my_robot_description.sensing.foot_loads.paired_loads',return_value=wrong):assert not audit_unloading_goal([],com,0.,p)['passed']
  endpoints.append(end['predicted_com_y_m'])
  previews.append(dict(target=target,support=side,displacement_m=p.displacement,setup=p.setup,peak_static_utilization=max(abs(t)/cap for r in p.lateral for t,cap in zip(r['feedforward_nm'],MOTOR_CAPS))))
 assert abs(sum(endpoints))<1e-8
for bad in [.01,.3,0,float('nan')]:
 try:LoweredUnloadProgram(xml,bad)
 except ValueError:pass
 else:raise AssertionError('Invalid fraction accepted')
try:LoweredShiftProgram(xml,.08)
except ValueError:pass
else:raise AssertionError('Old fixed-distance envelope was widened')
from pathlib import Path
out=Path('evidence/preparation/lowered-unloading');out.mkdir(parents=True,exist_ok=True)
(out/'offline-preview.json').write_text(json.dumps(dict(measured=False,previews=previews),indent=2)+'\\n')
"""
        subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            check=True,
        )
