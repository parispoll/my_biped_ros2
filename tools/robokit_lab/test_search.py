"""Candidate selection must never promote a failed or peak-regressing motion."""
import unittest
import numpy as np
from tools.robokit_lab.search import choose, densify, envelope


class SearchTests(unittest.TestCase):
    def test_reject_better_score_with_failed_gate_or_peak_regression(self):
        baseline=dict(passed=True,objective=.2,peak_static_cap_fraction=.53,dx_m=0.,dz_m=0.)
        failed=dict(baseline,passed=False,objective=.01)
        overloaded=dict(baseline,objective=.02,peak_static_cap_fraction=.6)
        acceptable=dict(baseline,objective=.18,dz_m=.005)
        self.assertEqual(choose([baseline,failed,overloaded,acceptable],baseline),acceptable)

    def test_pelvis_offset_has_no_endpoint_jump_or_velocity(self):
        for t in (117.,233.):
            self.assertEqual(envelope(t),0.)
            # C2 fade at both boundaries avoids position and velocity discontinuity.
            self.assertLess(abs(envelope(t+.001)-envelope(t-.001))/.002,1e-7)
        self.assertEqual(envelope(132),1.)
        self.assertEqual(envelope(169),1.)

    def test_densification_keeps_world_targets_and_endpoints(self):
        a=dict(time_s=117.,phase='lift',base=np.eye(4),targets={'left_foot':np.eye(4)},seed={'q':0.})
        b=dict(time_s=132.,phase='advance',base=np.eye(4),targets={'left_foot':np.eye(4)},seed={'q':.2})
        b['base']=b['base'].copy()
        b['base'][2,3]=.1
        frames=densify([a,b],5)
        self.assertEqual(len(frames),6)
        np.testing.assert_allclose(frames[0]['base'],a['base'])
        np.testing.assert_allclose(frames[-1]['base'],b['base'])
        for f in frames:
            np.testing.assert_allclose(f['targets']['left_foot'],np.eye(4))


if __name__=='__main__':
    unittest.main()
