"""IK must converge from an off-target seed and reject unreachable goals."""
import unittest
import numpy as np
from tools.robokit_lab import test_adapter
from tools.robokit_lab.check import reference
from tools.robokit_lab.ik import FootIK


class IKTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        test_adapter.AdapterTests.setUpClass()
        cls.adapter = test_adapter.AdapterTests.adapter
        cls.ik = FootIK(cls.adapter,['arm','tip'])

    @classmethod
    def tearDownClass(cls):
        test_adapter.AdapterTests.tearDownClass()

    def test_perturbed_seed_converges(self):
        target_q={'z_joint':.4,'a_joint':-.3}
        tf,_,_=reference(self.adapter.xml,target_q,np.eye(4))
        result=self.ik.solve({n:tf[n] for n in ('arm','tip')},{'z_joint':.2,'a_joint':-.1})
        self.assertLess(result['position_error_m'],1e-5)
        self.assertLess(result['rotation_error_rad'],1e-4)
        for n in target_q:
            self.assertAlmostEqual(result['positions_rad'][n],target_q[n],places=4)

    def test_unreachable_target_rejected(self):
        tf,_,_=reference(self.adapter.xml,{'z_joint':0.,'a_joint':0.},np.eye(4))
        tf['tip'][0,3]=10.
        with self.assertRaisesRegex(ValueError,'IK residual rejected'):
            self.ik.solve({n:tf[n] for n in ('arm','tip')},{'z_joint':0.,'a_joint':0.})

    def test_invalid_targets_rejected(self):
        with self.assertRaises(ValueError):
            self.ik.solve({'tip':np.eye(4)},{'z_joint':0.,'a_joint':0.})
        reflected=np.eye(4)
        reflected[0,0]=-1
        with self.assertRaises(ValueError):
            self.ik.solve({'arm':np.eye(4),'tip':reflected},{'z_joint':0.,'a_joint':0.})


if __name__=='__main__':
    unittest.main()
