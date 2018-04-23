import unittest
import numpy as np
from ..Path import Path, ForwardPathTracker, UnboundedSegmentException
from ..helper import quat_from_rpy

class TestPath(unittest.TestCase):

    def setUp(self):
        self.path = Path() 

    def addOneSegment(self, curvature, length):
        """ Shortcut helper to add segments """
        self.path.add_segment(curvature, length)

    def test_instantiation(self):
        
        path = Path([0.1, 0.0, 0.1], [10.0, 10.0, 10.0])
        self.assertEqual(len(path.segments), 3, "Instantiation did not result in 3 segments")
        self.assertEqual(path.get_length(), 30.0, "Instantiaton did not result in length of {0} but instead {1}".format(30.0, path.get_length()))

    def test_fail_instantiation(self):
        cs, lens = [0.1, 0.0, 0.1], [10.0, -1, 10.0]
        self.assertRaises(Exception, Path, cs, lens)

        cs, lens = [0.1, 0.0], [10.0]
        self.assertRaises(Exception, Path, cs, lens)

    def test_add_one_bounded_segment(self):
        self.addOneSegment(0.1, 10*2*np.pi)
        self.assertEqual(len(self.path.segments), 1, "Path did not save new segment")
        self.assertEqual(self.path.end_time, 2*10*np.pi, "Path end time is not correct")

        
    def test_add_one_unbounded_segment(self):
        self.addOneSegment(curvature=0.1, length=-1)

        self.assertEqual(len(self.path.segments), 1, "Path did not save newsegment")
        self.assertEqual(self.path.end_time, 0.0, "Unbounded last segment should imply end time is 0.0")


    def test_add_segment_unbounded_error(self):
        self.addOneSegment(0.1, 10)
        self.addOneSegment(0.1, -1)
   
        self.assertRaises(UnboundedSegmentException, self.path.add_segment, 0.1, 10)


    def test_get_length_unbounded_error(self):
        self.addOneSegment(0.1, -1)
        self.assertRaises(UnboundedSegmentException, self.path.get_length)

    def test_get_length(self):
        self.addOneSegment(0.1, 10.0)
        self.addOneSegment(-0.1, 10.0)

        length = self.path.get_length()
        self.assertEqual(length, 20.0, "Path of two segments of 10 each has overall length of: {0}, rather than: {1}".format(length, 20))


    def get_segment_after(self):
        self.addOneSegment(0.1, 10)
        self.addOneSegment(-0.1, 10)
        self.addOneSegment(0.0, -1)

        a,b,c = self.path.segments
        s = self.path.get_segment_after(a)
        self.assertEqual(s, b, "Expected segment {0} after {1} but got {2}".format(b, a, s))
        s = self.path.get_segment_after(b)
        self.assertEqual(s, c, "Expected segment {0} after {1} but got {2}".format(c, b, s))
        s = self.path.get_segment_after(c)
        self.assertEqual(s, c, "Expected same segment {0} after itself but got {1}".format(c, s))


    def get_segment_for_time(self):
        self.addOneSegment(0.1, 10.0)
        self.addOneSegment(0.1, -1)

        s1, s2 = self.path.segments

        s = self.path.get_segment_for_time(0.0)
        self.assertEqual(s, s1, "Time {0} is not in first segment when it should be".format(0.0))
        self.assertEqual(self.path.get_segment_for_time(5.0), s1, "Time {0} is not in first segment when it should be".format(5.0))
        self.assertEqual(self.path.get_segment_for_time(10.0), s2, "Time {0} is not in second segment when it should  be (curve 1 is 10.0 long -- boundary)".format(10.0))
        self.assertEqual(self.path.get_segment_for_time(20.0), s2, "Time {0} is not in second segment when it should be".format(20.0))


    def test_in_segment(self):
        self.addOneSegment(0.1, 10.0)
        self.addOneSegment(0.1, -1)

        s1, s2 = self.path.segments

        self.assertEqual(self.path._in_segment(0.0, s1), True, "Time {0} is not in first segment when it should be".format(0.0))
        self.assertEqual(self.path._in_segment(5.0, s1), True, "Time {0} is not in first segment when it should be".format(5.0))
        self.assertEqual(self.path._in_segment(10.0, s1), False, "Time {0} is in frist segment when it should not be (curve is 10.0 long -- boundary)".format(10.0))
        self.assertEqual(self.path._in_segment(20.0, s1), False, "Time {0} is in first segment when it should not be".format(20.0))
        self.assertEqual(self.path._in_segment(-10.0, s1), False, "{0} in frist segment when it should not be".format(-10.0))
        
        self.assertEqual(self.path._in_segment(10.0, s2), True, "Time {0} is not in second segment when it should be (unbounded)".format(10.0))
        self.assertEqual(self.path._in_segment(250.0, s2), True, "Time {0} is not in second segment when it should be (unbounded)".format(250.0))
        self.assertEqual(self.path._in_segment(5.0, s2), False, "Time {0} is in second segment when it should not be".format(5.0))


    def test_get_point_at(self):
        self.addOneSegment(0.0, 10.0)
        self.addOneSegment(0.1, 10*2*np.pi)
        self.addOneSegment(0.0, -1)
        # should be a path with 10 meters straight, one full revolution of 10m radius, then unlimited straight

        start = np.array([0.0, 0.0, 0.0])   # t = 0
        mid1 = np.array([5.0, 0.0, 0.0])    # t = 5
        end1 = np.array([10.0, 0.0, 0.0])   # t = 10
        quarter2 = np.array([20.0, 10.0, 0.0]) # t = 10 + 0.25*2*10*np.pi
        mid2 = np.array([10.0, 20.0, 0.0])  # t = 10 + 0.5*2*10*np.pi
        thirdQuarter2 = np.array([0.0, 10.0, 0.0]) # t = 10 + 0.75*2*10*pi
        end2 = np.array([10.0, 0.0, 0.0])   # t = 10 + 2*10*np.pi
        random3 = np.array([100, 0.0, 0.0]) # t = prev_t + 90.0

        points = [start, mid1, end1, quarter2, mid2, thirdQuarter2, end2, random3]
        times = [0.0, 5.0, 10.0, 10+0.25*2*10*np.pi, 10 + 0.5*2*10*np.pi, 10 + 0.75*2*10*np.pi, 10+2*10*np.pi, 10 + 2*10*np.pi + 90]

        computed_points = [self.path.get_point_at(t) for t in times]

        for expected, got in zip(points, computed_points):
            self.assertTrue(np.allclose(got, expected), "Expected {0} and got {1}".format(expected, got))

    def test_set_last_segment_length(self):
        self.addOneSegment(0.0, 10.0)
        self.addOneSegment(0.1, 20.0)

        self.assertEqual(self.path.get_length(), 30.0, "Path length is {0} when it should be 30.0".format(self.path.get_length()))

        self.path.set_last_segment_length(10.0)
        self.assertEqual(self.path.get_length(), 20.0, "Path length is {0} when it whould be 20.0".format(self.path.get_length()))

        self.addOneSegment(0.1, -1)
        self.assertRaises(UnboundedSegmentException, self.path.get_length)

        self.path.set_last_segment_length(100.0)
        self.assertEqual(self.path.get_length(), 120.0, "Path length is {0} when it should be 120.0".format(self.path.get_length()))


    def test_set_start(self):
        self.addOneSegment(0.0, 10.0)
        self.addOneSegment(0.1, 0.5*2*10*np.pi)

        orientation = np.array([0.0, 0.0, 90.0])
        start = np.array([1.0, 1.0, 0.0])
        self.path.set_start(orientation, start)

        ts = [0.0, 5.0, 10.0, 10 + 0.25*2*10*np.pi, 10 + 0.5*2*10*np.pi]
        pts = [self.path.get_point_at(t) for t in ts]
        expected = [[1.0, 1.0, 0.0],
                    [1.0, 6.0, 0.0],
                    [1.0, 11.0, 0.0],
                    [-9.0, 21.0, 0.0],
                    [-19.0, 11.0, 0.0]]

        for expected, computed in zip(expected, pts):
            self.assertTrue(np.allclose(expected, computed), "computed ll{0} is not [almost] equal to expected {1}".format(computed, expected))






