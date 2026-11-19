"""Interface tests using a deliberately reordered two-joint URDF."""
import tempfile
from pathlib import Path
import unittest
import numpy as np
from tools.robokit_lab.adapter import RobotAdapter
from tools.robokit_lab.check import agreement, reference, reference_jacobian

URDF = '''<robot name="fixture">
<link name="base"><inertial><mass value="2"/><inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/></inertial></link>
<link name="arm"><inertial><origin xyz="0.3 0 0"/><mass value="1"/><inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial></link>
<link name="tip"/>
<joint name="z_joint" type="revolute"><parent link="base"/><child link="arm"/><origin xyz=".2 0 0"/><axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="10" velocity="2"/></joint>
<joint name="a_joint" type="revolute"><parent link="arm"/><child link="tip"/><origin xyz=".5 0 0"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="2"/></joint>
</robot>'''


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.path = Path(cls.temp.name) / 'fixture.urdf'
        cls.path.write_text(URDF)
        cls.adapter = RobotAdapter(cls.path, ['a_joint', 'z_joint'])

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_named_mapping_and_rigid_base(self):
        q = {'z_joint': .4, 'a_joint': -.3}
        base = np.array([[0,-1,0,1],[1,0,0,2],[0,0,1,3],[0,0,0,1]], float)
        errors = agreement(self.adapter, q, base, ['tip'])
        self.assertLess(max(errors.values()), 2e-6)
        np.testing.assert_equal(self.adapter.pack(q), [q[n] for n in self.adapter.backend_names])

    def test_reject_invalid_state(self):
        for q in ({'a_joint': 0}, {'a_joint': 0, 'z_joint': float('nan')},
                  {'a_joint': 0, 'z_joint': 2}, {'a_joint':0,'z_joint':0,'extra':0}):
            with self.assertRaises(ValueError):
                self.adapter.pack(q)
        base = np.eye(4)
        base[0,0] = -1
        with self.assertRaises(ValueError):
            self.adapter.evaluate({'a_joint':0,'z_joint':0}, base)
        with self.assertRaises(ValueError):
            RobotAdapter(self.path, ['z_joint','z_joint'])

    def test_reference_jacobian_finite_difference(self):
        q = {'z_joint': .4, 'a_joint': -.3}
        names = list(q)
        jac = reference_jacobian(URDF, q, np.eye(4), 'tip', names)
        for i, name in enumerate(names):
            plus, minus = dict(q), dict(q)
            plus[name] += 1e-6
            minus[name] -= 1e-6
            a = reference(URDF, plus, np.eye(4))[0]['tip'][:3,3]
            b = reference(URDF, minus, np.eye(4))[0]['tip'][:3,3]
            np.testing.assert_allclose(jac[:,i], (a-b)/2e-6, atol=1e-9)


if __name__ == '__main__':
    unittest.main()
