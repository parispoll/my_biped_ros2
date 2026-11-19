"""P54 lowering geometry and rejection checks in a separate profile process."""

import os, subprocess, sys, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class LoweringTests(unittest.TestCase):
    def test_planted_feet_centered_com_and_slow_return(self):
        code = """
import xacro,math
from my_robot_description.motion.bilateral_lowering import LoweringProgram
from my_robot_description.model.roll_geometry import link_transforms,model_com
from my_robot_description.model.roll_stance import JOINT_NAMES,MOTOR_CAPS
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml();p=LoweringProgram(xml,.01)
last=None;speed=0.
for i in range(741):
 t=i*.05;r=p.sample(t);q=dict(zip(JOINT_NAMES,r['target_rad']));tf=link_transforms(xml,q);m,c=model_com(xml,q)
 assert abs(c[0]+r['pelvis_x_m']-p.planner.foot_x)<1e-6
 for side in ['left','right']:
  foot=tf[side+'_foot'];old=p.planner.feet[side+'_foot']
  assert abs(foot[0][3]+r['pelvis_x_m']-old[0][3])<1e-6
  assert abs(foot[2][3]+r['pelvis_height_m']-.045)<1e-6
  assert max(abs(foot[a][b]-float(a==b)) for a in range(3) for b in range(3))<1e-9
 assert all(abs(v)<.8*cap for v,cap in zip(r['feedforward_nm'],MOTOR_CAPS))
 if last:speed=max(speed,max(abs(a-b)/.05 for a,b in zip(r['target_rad'],last)))
 last=r['target_rad']
assert speed<.05
assert abs(p.sample(18)['pelvis_height_m']-p.sample(0)['pelvis_height_m']+.01)<1e-8
assert max(abs(a-b) for a,b in zip(p.sample(37)['target_rad'],p.sample(0)['target_rad']))<1e-12
deep=LoweringProgram(xml,.02)
assert deep.setup['total_time_s']==57.
assert abs(deep.sample(28)['pelvis_height_m']-deep.sample(0)['pelvis_height_m']+.02)<1e-8
assert deep.sample(57)['phase']=='done'
prev=None
for i in range(1141):
 r=deep.sample(i*.05)
 tf=link_transforms(xml,dict(zip(JOINT_NAMES,r['target_rad'])))
 for side in ['left','right']:
  assert abs(tf[side+'_foot'][2][3]+r['pelvis_height_m']-.045)<1e-6
 if prev:assert max(abs(a-b)/.05 for a,b in zip(r['target_rad'],prev))<.05
 prev=r['target_rad']
for depth in [.03,.04]:
 deeper=LoweringProgram(xml,depth);total=17+2000*depth
 assert deeper.setup['total_time_s']==total
 assert len(deeper.table)==round(depth/.00025)+1
 prev=None
 for i in range(round(total/.05)+1):
  r=deeper.sample(i*.05)
  q=dict(zip(JOINT_NAMES,r['target_rad']));tf=link_transforms(xml,q);_,c=model_com(xml,q)
  assert abs(c[0]+r['pelvis_x_m']-deeper.planner.foot_x)<1e-6
  for side in ['left','right']:
   assert abs(tf[side+'_foot'][2][3]+r['pelvis_height_m']-.045)<1e-6
  assert all(abs(t)<.8*cap for t,cap in zip(r['feedforward_nm'],MOTOR_CAPS))
  if prev:assert max(abs(a-b)/.05 for a,b in zip(r['target_rad'],prev))<.05
  prev=r['target_rad']
 assert abs(deeper.sample(8+1000*depth)['pelvis_height_m']-deeper.sample(0)['pelvis_height_m']+depth)<1e-8
 assert deeper.sample(total)['phase']=='done'
for bad in [0,-.01,.041,float('nan')]:
 try:LoweringProgram(xml,bad)
 except ValueError:pass
 else:raise AssertionError('Accepted invalid lowering')
from unittest.mock import patch
from my_robot_description.validation.lowering_audit import audit_lowering
import copy
sensors=[];com=[];pairs=[];weight=m*9.81
for i in range(1851):
 t=i*.02;r=p.sample(t)
 sensors.append(dict(sim_time=t,pose=dict(sim_time=t,position=[r['pelvis_x_m'],0.,r['pelvis_height_m']])))
 com.append(dict(sim_time=t,com=[p.planner.foot_x,0.,.4]))
 pairs.append(dict(sim_time=t,total_vertical_n=weight,left=dict(force_world_n=[0,0,weight/2]),cop_world_xy_m=[p.planner.foot_x,0.]))
with patch('my_robot_description.validation.lowering_audit.paired_loads',return_value=pairs):
 sensors[0]['pose']['sim_time']=-.005
 assert audit_lowering(sensors,com,[],0.,37.,p)['passed']
 still=copy.deepcopy(sensors)
 for row in still:row['pose']['position']=[0.,0.,p.table[0]['pelvis_height_m']]
 assert not audit_lowering(still,com,[],0.,37.,p)['passed']
 corrupt=copy.deepcopy(pairs);corrupt[500]['left']['force_world_n'][2]=weight*.6
 with patch('my_robot_description.validation.lowering_audit.paired_loads',return_value=corrupt):
  assert not audit_lowering(sensors,com,[],0.,37.,p)['passed']
"""
        subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            check=True,
        )
