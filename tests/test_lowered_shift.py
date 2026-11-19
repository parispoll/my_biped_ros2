"""Combined fixed-foot geometry, mirrored loads and measurement-audit rejection."""

import os, subprocess, sys, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


class LoweredShiftTests(unittest.TestCase):
    def test_combined_plan_and_independent_force_audit(self):
        code = """
import math,xacro,copy
from unittest.mock import patch
from my_robot_description.motion.lowered_shift import LoweredShiftProgram
from my_robot_description.model.roll_geometry import link_transforms,model_com
from my_robot_description.model.roll_stance import JOINT_NAMES,MOTOR_CAPS
from my_robot_description.validation.lowering_audit import audit_lowering
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml()
for direction in [.005,-.005,.01,-.01,.02,-.02,.03,-.03,.04,-.04]:
 p=LoweredShiftProgram(xml,direction);previous=None;total=p.setup['total_time_s']
 assert total=={.005:121.,.01:137.,.02:169.,.03:201.,.04:233.}[abs(direction)]
 assert len(p.lateral)=={.005:41,.01:81,.02:161,.03:241,.04:321}[abs(direction)]
 for i in range(round(total/.05)+1):
  t=i*.05;r=p.sample(t);q=dict(zip(JOINT_NAMES,r['target_rad']));tf=link_transforms(xml,q);m,c=model_com(xml,q)
  for side in ['left','right']:
   foot=tf[side+'_foot'];old=p.planner.feet[side+'_foot']
   for k,offset in enumerate([r['pelvis_x_m'],r['pelvis_y_m'],r['pelvis_height_m']-p.planner.base['expected_pelvis_height_m']]):
    assert abs(foot[k][3]+offset-old[k][3])<1e-6
   assert max(abs(foot[a][b]-float(a==b)) for a in range(3) for b in range(3))<1e-9
  assert abs(c[0]+r['pelvis_x_m']-p.planner.foot_x)<1e-6
  assert abs(c[1]+r['pelvis_y_m']-r['predicted_com_y_m'])<1e-6
  assert all(abs(t)<.8*cap for t,cap in zip(r['feedforward_nm'],MOTOR_CAPS))
  if previous:assert max(abs(a-b)/.05 for a,b in zip(r['target_rad'],previous))<.05
  previous=r['target_rad']
 hold_time=sum(p.setup['analysis_windows']['hold'])/2
 assert p.sample(hold_time)['pelvis_y_m']==direction
 assert p.sample(hold_time)['predicted_com_y_m']*direction>0
 assert p.sample(total)['phase']=='done'
 assert p.sample(total)['target_rad']==p.sample(0)['target_rad']
 sensors=[];com=[];pairs=[];weight=m*9.81
 for i in range(round(total/.02)+1):
  t=i*.02;r=p.sample(t)
  sensors.append(dict(sim_time=t,pose=dict(sim_time=t,position=[r['pelvis_x_m'],r['pelvis_y_m'],r['pelvis_height_m']])))
  xy=[p.planner.foot_x,r['predicted_com_y_m']]
  com.append(dict(sim_time=t,com=xy+[.4]))
  pairs.append(dict(sim_time=t,total_vertical_n=weight,left=dict(force_world_n=[0,0,r['predicted_force_n']['left']]),cop_world_xy_m=xy))
 with patch('my_robot_description.validation.lowering_audit.paired_loads',return_value=pairs):
  assert audit_lowering(sensors,com,[],0.,total,p,force_reference=p)['passed']
 wrong=copy.deepcopy(pairs)
 for row in wrong:row['left']['force_world_n'][2]=weight/2
 with patch('my_robot_description.validation.lowering_audit.paired_loads',return_value=wrong):
  assert not audit_lowering(sensors,com,[],0.,total,p,force_reference=p)['passed']
for bad in [0,.05,float('nan')]:
 try:LoweredShiftProgram(xml,bad)
 except ValueError:pass
 else:raise AssertionError('Unsupported lowered shift accepted')
"""
        subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            check=True,
        )
