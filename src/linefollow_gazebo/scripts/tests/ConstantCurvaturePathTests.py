import unittest
import numpy as np
from ..helper import quat_from_rpy
from ..ConstantCurvaturePath import ConstantCurvaturePath

class TestCCP(unittest.TestCase):

    def create(self, curvature, length):
        self.curve = ConstantCurvaturePath(curvature, length)


    def test_point_at(self):
        self.create(0.1, 10*2*np.pi)

        ts = np.array([0.0, 0.25, 0.5, 0.75, 1.0])*2*10*np.pi
        pts = [self.curve.point_at(t) for t in ts]
        expected = np.array([[0.0, 0.0, 0.0], [10.0, 10.0, 0.0], [0.0, 20.0, 0.0], [-10.0, 10.0, 0.0], [0.0, 0.0, 0.0]])

        for exp, pt in zip(expected, pts):
            self.assertTrue(np.allclose(pt, exp), "Expected point {0} but got {1}".format(exp, pt))


    
     
    def test_get_length(self):
        self.create(0.1, 10.0)
        self.assertEqual(self.curve.get_length(), 10.0, "Curve length is not {0} but {1}".format(10.0, self.curve.get_length()))

    def test_set_length(self):
        self.create(0.1, 10.0)
        self.curve.set_length(20.0)

        self.assertEqual(self.curve.get_length(), 20.0, "Curve length is not {0} but {1}".format(20.0, self.curve.get_length()))

    def test_set_start_time(self):
        self.create(0.1, 10.0)

        self.assertEqual(self.curve.start_time, 0.0, "Curve start time on creation is not {0} but {1}".format(0.0, self.curve.start_time))

        self.curve.set_start_time(10.0)
        self.assertEqual(self.curve.start_time, 10.0, "Curve start time after setting start time is {0} instead of {1}".format(self.curve.start_time, 10.0))
        self.assertEqual(self.curve.get_duration(), 10.0, "Curve duration is {0} instead of {1}".format(self.curve.get_duration(), 10.0))
        self.assertEqual(self.curve.end_time, 20.0, "Curve end time is {0} instead of {1}".format(self.curve.end_time, 20.0))


    def test_set_start_position(self):
        self.create(0.1, 10.0)
        self.curve.set_start_position(pos=np.array([10.0, 5.0, 0.0]))

        self.assertTrue(np.allclose(self.curve.point_at(0), np.array([10.0, 5.0, 0.0])), "t=0 got {0} instead of {1}".format(self.curve.point_at(0), np.array([10.0, 5.0, 0.0])))


    def test_set_start_orientation(self):
        self.create(0.1, 0.5*10*2*np.pi)

        self.curve.set_start_orientation_rpy(np.array([0.0, 0.0, 90.0]))

        exp_p_t0, p_t0 = np.array([0.0, 0.0, 0.0]), self.curve.point_at(0.0)
        self.assertTrue(np.allclose(p_t0, exp_p_t0), "After rotation of degrees around Z axis got {0} instead of expected {1}".format(p_t0, exp_p_t0))
        
        exp_p_t1, p_t1 = np.array([-10.0, 10.0, 0.0]), self.curve.point_at(0.25*10*2*np.pi)
        self.assertTrue(np.allclose(p_t1, exp_p_t1), "After rotation of 90 deg around Z axis got {0} instead of {1} at quarter way around circle".format(p_t1, exp_p_t1))

        exp_p_t2, p_t2 = np.array([-20.0, 0.0, 0.0]), self.curve.point_at(0.5*10*2*np.pi)
        self.assertTrue(np.allclose(p_t2, exp_p_t2), "After rot of 90 deg around Z got {0} instead of {1} after half way around circle".format(p_t2, exp_p_t2))

        # TODO test tangents and normals


