"""Independent geometry/virtual-work checks in an isolated P54 process."""

import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


class FootLiftTests(unittest.TestCase):
    def test_mirrored_geometry_virtual_work_and_rejections(self):
        code = """
import math,xacro
from my_robot_description.motion.foot_lift import FootLiftPreview,gravity_gradient_check,single_support_loads
from my_robot_description.model.roll_stance import JOINT_NAMES
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml()
plans=[FootLiftPreview(xml,s) for s in ('left','right')]
for p in plans:
 for i in [0,10,20,30,40]:
  row=p.table[i];q=dict(zip(JOINT_NAMES,row['target_rad']))
  assert gravity_gradient_check(xml,row['target_rad'],p.support)['passed']
  assert row['predicted_force_n'][p.swing]==0
  assert row['support_margin_m']>.05 and row['peak_static_motor_utilization']<.8
  for side,corners in row['sole_corner_displacements_m'].items():
   for c in corners:
    assert abs(c[0])<1e-10 and abs(c[1])<1e-10
    assert abs(c[2]-(row['requested_clearance_m'] if side==p.swing else 0))<1e-10
  # Supporting leg must be unchanged while the other bends.
  for j,n in enumerate(JOINT_NAMES):
   if n.startswith(p.support+'_'):assert row['target_rad'][j]==p.endpoint['target_rad'][j]
 assert p.sample(0)['target_rad']==p.sample(22)['target_rad']
 assert p.sample(11)['requested_clearance_m']==.002
 assert p.sample(1e-4)['requested_clearance_m']<1e-12
 assert p.setup['live_lift_implemented'] is False
 for bad in [-.001,.0021,float('nan')]:
  try:p.pose(bad)
  except ValueError:pass
  else:raise AssertionError('Invalid height accepted')
 for bad in [-1,float('nan')]:
  try:p.sample(bad)
  except ValueError:pass
  else:raise AssertionError('Invalid time accepted')
 # A grossly unbalanced posture cannot be labeled single support.
 try:single_support_loads(xml,dict(zip(JOINT_NAMES,p.zero.planner.base['target_rad'])),p.support)
 except ValueError:pass
 else:raise AssertionError('COM outside supporting sole accepted')
for a,b in zip(plans[0].table,plans[1].table):
 assert abs(a['support_margin_m']-b['support_margin_m'])<1e-10
 assert abs(a['peak_static_motor_utilization']-b['peak_static_motor_utilization'])<1e-10
for side in ['front',None]:
 try:FootLiftPreview(xml,side)
 except ValueError:pass
 else:raise AssertionError('Invalid support accepted')
"""
        subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=dict(os.environ, BIPED_ACTUATOR_PROFILE="robotis_p54"),
            check=True,
        )
