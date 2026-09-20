import unittest
import numpy as np
from cm_trajopt_common import LocalCubicTrajectoryOptimizer

class WarmStartTest(unittest.TestCase):
    def setUp(self):
        self.q=np.array([[0]*6,[.1]*6,[.25]*6],dtype=float)
        self.options={'interior_optimization_samples_per_segment':1,'samples_per_segment':3,
                      'minimum_segment_duration':.1,'maximum_segment_duration':3,'velocity_max':[1]*6}
        self.opt=LocalCubicTrajectoryOptimizer('left',self.q,self.options)
    def test_default_unchanged_zero_inner_speed(self):
        np.testing.assert_array_equal(self.opt.x[6:12],0)
        self.assertEqual(self.opt.sample_count,3)
        self.assertEqual(self.opt.interior_sample_count,1)
    def test_warm_start_c1_with_original_endpoints(self):
        times=np.array([.8,1.2]);vel=np.array([[.2]*6])
        self.opt.set_initial_state(times,vel)
        c=self.opt.coefficients()
        np.testing.assert_allclose(c[0,0],self.q[0])
        np.testing.assert_allclose(c[-1].sum(axis=0),self.q[-1],atol=1e-14)
        np.testing.assert_allclose((c[0,1]+2*c[0,2]+3*c[0,3])/.8,vel[0],atol=1e-14)
        np.testing.assert_allclose(c[1,1]/1.2,vel[0],atol=1e-14)
        np.testing.assert_allclose(c[0,1],0)
        np.testing.assert_allclose((c[-1,1]+2*c[-1,2]+3*c[-1,3])/1.2,0,atol=1e-14)
        np.testing.assert_allclose(self.opt.x[:6],self.q[1])
        self.assertEqual(self.opt.interior_sample_count,1)
    def test_input_arrays_are_copied(self):
        times=np.array([.8,1.2]);vel=np.array([[.2]*6])
        self.opt.set_initial_state(times,vel);times[:]=2;vel[:]=.9
        np.testing.assert_array_equal(self.opt.times,[.8,1.2])
        np.testing.assert_array_equal(self.opt.x[6:12],[.2]*6)
    def test_invalid_time_does_not_mutate(self):
        for times in [[.05,1],[1,4],[1],[np.nan,1],[np.inf,1]]:
            x=self.opt.x.copy();t=self.opt.times.copy()
            with self.assertRaises(ValueError):self.opt.set_initial_state(times,[[.2]*6])
            np.testing.assert_array_equal(self.opt.x,x);np.testing.assert_array_equal(self.opt.times,t)
    def test_invalid_velocity_does_not_mutate(self):
        for v in [[[1.1]*6],[[np.nan]*6],[[np.inf]*6],[[.2]*5],[]]:
            x=self.opt.x.copy();t=self.opt.times.copy()
            with self.assertRaises(ValueError):self.opt.set_initial_state([.8,1.2],v)
            np.testing.assert_array_equal(self.opt.x,x);np.testing.assert_array_equal(self.opt.times,t)
    def test_one_piece_has_no_interior_velocity(self):
        opt=LocalCubicTrajectoryOptimizer('right',self.q[:2],self.options)
        opt.set_initial_state([.8],[])
        np.testing.assert_array_equal(opt.times,[.8])
        self.assertEqual(opt.inner_count,0)

if __name__=='__main__':unittest.main()
