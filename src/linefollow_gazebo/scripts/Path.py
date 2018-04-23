"""
Piecewise paths analytically defined
"""
import numpy as np
from ConstantCurvaturePath import ConstantCurvaturePath

# A Path is defined as a sequence of curves
# Time parametrized from t = 0 at a speed of 1 m/s


# NEW
# curves need start time, end time
# possibly a loop signal (for repeated execution)


class Path(object):
    """ Class Representing a Path, componses of multiple curves. Defined from t = 0 onwards, parametrized at implicity speed 1 m/s so time == distance"""


    def __init__(self, curvatures=[], lengths=[], start_pos=np.array([0.0, 0.0, 0.0]), start_orientation_rpy=np.array([0.0, 0.0, 0.0]), repeat=False):
        if len(curvatures) != len(lengths):
            raise Exception("Path creation must provide both curvatures and lengths for each segment")

        self.segments = []
        self.start_pos = start_pos
        self.start_orientation = start_orientation_rpy
        self.end_time = 0.0
        self.repeat = repeat
        
        if len(curvatures) > 0:

            if -1 in lengths:
                if self.repeat:
                    raise Exception("Cannot have infinite length segment (= -1) and repeat path enabled: {0}".format(lengths))
                if lengths.index(-1) != len(lengths) - 1:
                    raise UnboundedSegmentException("Cannot have infinite length (=-1) segment that is not the last segment: {0}".format(lengths))


            first_segment = ConstantCurvaturePath(curvatures[0], lengths[0])
            first_segment.set_start_time(0.0)
            first_segment.set_start_position(start_pos)
            first_segment.set_start_orientation(start_orientation_rpy)
            self.segments.append(first_segment)
            
            if lengths[0] != -1:
                self.end_time += lengths[0]

            for curvature, length in zip(curvatures[1:], lengths[1:]):
                
                pos = self.segments[-1].point_at(self.end_time)
                orientation_matrix = self.segments[-1].tangent_rotation_matrix_at(self.end_time)
    
                c = ConstantCurvaturePath(curvature, length)
                c.set_start_time(self.end_time)
                c.set_start_position(pos)
                c.set_start_orientation_matrix(orientation_matrix)
                self.segments.append(c)
                if length != -1:
                    self.end_time += length


    def add_segment(self, curvature, length):

        if len(self.segments) > 0:
            # pick out the finishing point of the last segment
            last_segment = self.segments[-1]

            # check to make sure the prior segment has finite length
            if last_segment.get_length() == -1:
                raise UnboundedSegmentException("Last existing segment has unlimited length, cannot add another after!")

            pos = last_segment.point_at(self.end_time)
            orientation_matrix = last_segment.tangent_rotation_matrix_at(self.end_time)


        else:
            pos = self.start_pos
            orientation_quat = self.start_orientation

        new_segment = ConstantCurvaturePath(curvature, length)
        new_segment.set_start_time(self.end_time)
        new_segment.set_start_position(pos)
        new_segment.set_start_orientation_matrix(orientation_matrix)
        self.segments.append(new_segment)

        if length != -1:
            self.end_time += length


    def set_start(self, orientation_rpy_deg, position, apply_orientation_first=True):
        
        # these need to be handled as additional transformations
        # rather than set_start otherwise all non-first segments
        # will be reset to be transformed relative to the origin
        # when we want to transform relative to whereever they were before
        for segment in self.segments:
            if apply_orientation_first:
                segment.add_rotation_rpy(orientation_rpy_deg)
                segment.add_translation(position)
            else:
                segment.add_translation(position)
                segment.add_rotation_rpy(orientation_rpy_deg)


    def set_last_segment_length(self, length):
        """ Used to shorten last segment which may have infinite length """
        prior_length = self.segments[-1].get_length()
        if prior_length != -1:
            self.end_time -= prior_length

        self.segments[-1].set_length(length)
        self.end_time += length


    def get_segment_for_time(self, target_time):
        if self.repeat:
            target_time = target_time % self.end_time
        time = 0.0
        for segment in self.segments:
            if segment.get_length() == -1:
                return segment # unbounded length will catch all following times
            else:
                time += segment.get_length()
                print("Segment end time: {0}, seaching for: {1}".format(time, target_time))
                if target_time <= time:
                    return segment
        # TODO deal with lapping/circuits

    def get_segment_after(self, target_segment):
        index = self.segments.index(target_segment)
        last_index = len(self.segments) - 1
        if index == last_index:
            if self.repeat:
                return 0 # reset to first segment
            else:
                # TODO this might be better off with an Exception
                return index

    def get_point_at(self, t):
        """ This costs a linear lookup in the number of paths - use for one offs """
        segment = self.get_segment_for_time(t)
        return segment.point_at(t)

    def _in_segment(self, time, segment):
        return segment.start_time <= time and (time < segment.start_time + segment.get_length() or segment.get_length() == -1)


    def discretize_points(self, resolution=0.5):
        """ Discretizes entire curve at the given meter resolution. Useful for debugging/plotting. Repeat is ignored. """
        if self.segments[-1].get_length() == -1:
            print("Give last segment a length before discretizing.")
            return None
            
        points = []
        i, segment = 0, self.segments[0]
        for t in np.arange(0.0, self.end_time, resolution):
            if not self._in_segment(t, segment):
                i += 1
                segment = self.segments[i]

            points.append(segment.point_at(t))
        
        return np.array(points)


    def get_tracker(self, start_dist=0.0, max_horizon=5.0):
        return ForwardPathTracker(self, start_dist=start_dist, max_horizon=max_horizon)


    def get_length(self):
        if self.segments[-1].get_length() == -1:
            raise UnboundedSegmentException("Cannot get length when last segment has infinite length")

        return self.end_time


class ForwardPathTracker(object):
    """ Wraps a Path object with some state that allows tracking it in a forward direction only (only allow forward progress) """

    def __init__(self, path, start_dist=0.0, max_horizon=5.0):
        """
        start_time: apply time shift to the path
        start_dist: distance in path to start tracking from
        """

        self.path = path
        
        # everything is implicilty parametrized at speed = 1 m/s
        self.last_t = start_dist    # last 'time' on curve we tracked

        self.active_segment = self.path.get_segment_for_time(self.last_t )
        self.max_horizon = max_horizon 


    def update(self, position):
        self.current_position = position
        self._update_values()

    def _update_values(self):
       
        current = self.active_segment
        next_segment = self.path.get_segment_after(current)

        closest_time = current.closest_point_time(self.current_position, self.last_t, self.max_horizon)
        closest_point = current.point_at(closest_time)
        d = np.linalg.norm(self.current_position, closest_point)

        # NOTE since this could be a repetition, next_comp_closest_time could be less than closest time!
        # this is actually what we want...
        next_comp_closest_time = next_segment.closest_point_time(self.current_position, self.last_t, self.max_horizon)
        next_comp_closest_point = next_segment.point_at(next_comp_closest_time)
        d_next = np.linalg.norm(self.current_position, next_comp_closest_point)

        if d_next < d:
            # we have moved onto the next segment of the path
            self.active_segment = next_segment
            closest_time = next_comp_closest_time
            closest_point = next_comp_closest_point
            d = d_next
      
        self.last_t = closest_time 
        self.closest_point = closest_point
        self.closest_point_dist = d
        self.closest_tangent = self.active_segment.tangent_at(closest_time)


    def get_closest_point(self):
        return self.closest_point

    def get_closest_point_distance(self):
        return self.closest_point_dist

    def get_closest_tangent(self):
        return self.closest_tangent


class UnboundedSegmentException(Exception):
    pass
