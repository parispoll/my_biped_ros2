"""Fresh-process regressions for model-derived mass and P54-only transfer goals."""

import os
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]


class P54TransferTests(unittest.TestCase):
    def run_profile(self, name, code):
        env = dict(os.environ, BIPED_ACTUATOR_PROFILE=name)
        env["PYTHONPATH"] = (
            str(ROOT / "tests")
            + os.pathsep
            + str(ROOT / "src/my_robot_description")
            + os.pathsep
            + env.get("PYTHONPATH", "")
        )
        r = subprocess.run(
            [sys.executable, "-c", code],
            cwd=ROOT,
            env=env,
            capture_output=True,
            text=True,
        )
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    def test_com_audit_uses_model_mass_and_rejects_wrong_observed_mass(self):
        self.run_profile(
            "robotis_p54",
            """
import copy
from test_lateral_shift import LateralAuditTests as Fixture
from my_robot_description.validation.lateral_audit import audit_shift
Fixture.setUpClass()
r=audit_shift(Fixture.rows,Fixture.com,Fixture.xml,.01)
assert r['passed'],r
assert r['com_coverage']['expected_mass_kg']==22.28
bad=copy.deepcopy(Fixture.com)
for item in bad:item['mass_kg']=14.42
r=audit_shift(Fixture.rows,bad,Fixture.xml,.01)
assert not r['passed'] and not r['com_coverage']['mass_matches_model']
assert r['com_coverage']['coverage_passed']
""",
        )

    def test_seventy_thirty_target_requires_p54_and_preserves_floor(self):
        self.run_profile(
            "robotis_p54",
            """
import xacro
from my_robot_description.motion.partial_unload import PartialUnloadProgram,MIN_SHARE
from my_robot_description.model.roll_stance import MOTOR_CAPS
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml()
programs=[PartialUnloadProgram(xml,.3,side) for side in ('left','right')]
for p in programs:
 mass=p.planner.base['mass_kg'];foot=p.setup['unloaded_foot']
 assert abs(p.setup['endpoint']['predicted_force_n'][foot]/(mass*9.81)-.3)<1e-10
 assert MIN_SHARE==.25 and p.setup['total_time_s']==37
 for point in p.table:
  assert point['max_fixed_foot_geometry_error']<1e-10
  assert all(abs(t)<=.8*c for t,c in zip(point['feedforward_nm'],MOTOR_CAPS))
assert abs(programs[0].displacement+programs[1].displacement)<1e-10
""",
        )
        self.run_profile(
            "legacy",
            """
import xacro
from my_robot_description.motion.partial_unload import PartialUnloadProgram
xml=xacro.process_file('src/my_robot_description/urdf/biped_roll.urdf.xacro').toxml()
try:PartialUnloadProgram(xml,.3)
except ValueError as e:assert 'robotis_p54' in str(e)
else:raise AssertionError('Legacy target was widened')
""",
        )
