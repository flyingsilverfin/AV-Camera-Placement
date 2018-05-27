"""
Piecewise paths analytically defined
"""
import numpy as np
from ConstantCurvaturePath import ConstantCurvaturePath
from helper import rpy_to_matrix

# A Path is defined as a sequence of curves
# Time parametrized from t = 0 at a speed of 1 m/s


# NEW
# curves need start time, end time
# possibly a loop signal (for looped execution)


class Path(object):
    """ Class Representing a Path, componses of multiple curves. Defined from t = 0 onwards, parametrized at implicity speed 1 m/s so time == distance"""


    def __init__(self, curvatures=[], lengths=[], start_pos=np.array([0.0, 0.0, 0.0]), start_orientation_rpy_deg=np.array([0.0, 0.0, 0.0]), loop=False):
        if len(curvatures) != len(lengths):
            raise Exception("Path creation must provide both curvatures and lengths for each segment")

        self.segments = []
        self.start_pos = start_pos
        self.start_orientation_rpy = start_orientation_rpy_deg
        self.end_time = 0.0
        self.loop = loop
        
        if len(curvatures) > 0:

            if -1 in lengths:
                if self.loop:
                    raise Exception("Cannot have infinite length segment (= -1) and loop path enabled: {0}".format(lengths))
                if lengths.index(-1) != len(lengths) - 1:
                    raise UnboundedSegmentException("Cannot have infinite length (=-1) segment that is not the last segment: {0}".format(lengths))


            first_segment = ConstantCurvaturePath(curvatures[0], lengths[0])
            first_segment.set_start_time(0.0)
            first_segment.set_start_position(start_pos)
            first_segment.set_start_orientation_rpy(start_orientation_rpy_deg)
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
            orientation_matrix = rpy_to_matrix(*self.start_orientation_rpy)

        new_segment = ConstantCurvaturePath(curvature, length)
        new_segment.set_start_time(self.end_time)
        # rotation first, then translation!!
        new_segment.set_start_orientation_matrix(orientation_matrix)
        new_segment.set_start_position(pos)
        self.segments.append(new_segment)

        if length != -1:
            self.end_time += length

    def reset_to_origin(self):
        """ Resets the segments transforms to be a connected curve with the first being x-axis aligned again """
        if len(self.segments) == 0:
            return
        self.segments[0].reset_transforms()
        if len(self.segments) > 1:
            prev_pos = self.segments[0].point_at(self.segments[0].end_time)
            prev_rot = self.segments[0].tangent_rotation_matrix_at(self.segments[0].end_time)
            for segment in self.segments[1:-1]:
                end_time = segment.end_time
                if end_time == -1:
                    # this should never arise since we chopped off the last segment in the loop
                    # and only the last segment can be unbounded in a valid path
                    raise Exception("Badly formatted piecewise path with unbounded length that isn't last segment")
                segment.reset_transforms()
                segment.set_start_orientation_matrix(prev_rot)
                segment.set_start_position(prev_pos)

                prev_pos = segment.point_at(end_time)
                prev_rot = segment.tangent_rotation_matrix_at(end_time)

            self.segments[-1].reset_transforms()
            self.segments[-1].set_start_orientation_matrix(prev_rot)
            self.segments[-1].set_start_position(prev_pos)


            


    def set_start(self, position, orientation_rpy_deg, apply_orientation_first=True):
        
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
        if self.loop:
            target_time = target_time % self.end_time
       
        # if querying beyond last segment, return last segment
        if target_time > self.get_length():
            return self.segments[-1]

        time = 0.0
        for segment in self.segments:
            if segment.get_length() == -1:
                return segment # unbounded length will catch all following times
            else:
                time += segment.get_length()
                if target_time <= time:
                    return segment

    def get_segment_after(self, target_segment):
        """ Basic implementation of this shouldn't be used much as it's a linear lookup each time."""
        index = self.segments.index(target_segment)
        last_index = len(self.segments) - 1
        if index == last_index:
            if self.loop:
                return self.segments[0] # reset to first segment
            else:
                # TODO this might be better off with an Exception
                return target_segment #return same thing
        return self.segments[index+1]


    def get_point_at(self, t):
        """ This costs a linear lookup in the number of paths - use for one offs """
        segment = self.get_segment_for_time(t)
        return segment.point_at(t)

    def get_tangent_at(self, t):
        segment = self.get_segment_for_time(t)
        return segment.tangent_at(t)

    def get_normal_at(self, t):
        segment = self.get_segment_for_time(t)
        return segment.normal_at(t)

    def _in_segment(self, time, segment):
        return segment.start_time <= time and (time < segment.start_time + segment.get_length() or segment.get_length() == -1)

    def get_curvature_at(self, time):
        segment = self.get_segment_for_time(time)
        return segment.curvature


    def discretize_points(self, resolution=0.5, width_boundaries=None):
        """ Discretizes entire curve at the given meter resolution. Useful for debugging/plotting. loop is ignored. """
        if self.segments[-1].get_length() == -1:
            print("Give last segment a length before discretizing.")
            return None

        # also computes boundaries of path if width_boundaries is not none
            
        points = []
        boundary_left = []
        boundary_right = []
        i, segment = 0, self.segments[0]
        for t in np.arange(0.0, self.end_time, resolution):
            if not self._in_segment(t, segment):
                i += 1
                segment = self.segments[i]

            pt = segment.point_at(t)
            points.append(pt)

            if width_boundaries is not None:
                normal = segment.normal_at(t)
                boundary_left.append(pt + width_boundaries * normal)
                boundary_right.append(pt - width_boundaries * normal)
        
        if width_boundaries is None:
            return np.array(points)
        else:
            return np.array([points, boundary_left,boundary_right])


    def get_tracker(self, start_dist=0.0, max_horizon=10.0):
        return ForwardPathTracker(self, start_dist=start_dist, max_horizon=max_horizon)

    def get_velocity_profile(self, 
                             lookahead_time=3.0,
                             lookahead_interval=0.5,
                             straight_line_speed=20.0,
                             radius_speed_multiplier=1.0):
        return VelocityProfile(self, lookahead_time, lookahead_interval, straight_line_speed, radius_speed_multiplier)

    def get_length(self):
        if self.segments[-1].get_length() == -1:
            return -1

        return self.end_time

    def save_as_fig(self, path):
        import matplotlib as mpl
        mpl.use('agg')
        import matplotlib.pyplot as plt
        pts = self.discretize_points()
        if pts is None:
            return
        plt.plot(pts[:, 0], pts[:, 1])
        plt.savefig(path)


class VelocityProfile(object):
    """ Calculates velocity profile for a given path """

    def __init__(self, path,                  
                 lookahead_time=3.0,
                 lookahead_interval=0.5,
                 straight_line_speed=20.0,
                 radius_speed_multiplier=1.0):

        self.path = path
        self.lookahead_time = lookahead_time
        self.lookahead_interval = lookahead_interval 
        self.straight_line_speed = straight_line_speed
        self.radius_speed_multiplier = radius_speed_multiplier


    def get_target_speed(self, path_time, current_speed):
        """ expected usage: input path_time corresponding to the path tracker's closest time
            This is used to evaluate points on the curve """

        # since the path_time is actually path distance
        # need to convert lookead_time into lookahead_distance
        lookahead_distance = current_speed * self.lookahead_time
        time_step_size = current_speed * self.lookahead_interval
        steps = np.arange(path_time, path_time + lookahead_distance, time_step_size)

        # Calculate the average target speed over the lookead distance 
        target_speed = 0 
        smallest_speed = 0 
        largest_curvature = 0
        for dist in steps:
            # TODO this is going to be rather inefficient but 
            # it is not totally trivial to make it faster
            # TODO profile lookups
            curvature = np.abs(self.path.get_curvature_at(dist)) # recall curvature is signed!
            # convert curvature into target speed
            speed = self.straight_line_speed if curvature == 0.0 else self.radius_speed_multiplier * 1.0/curvature 

            # don't want to accelerate until leave a curve
            if curvature < largest_curvature:
                target_speed += smallest_speed
            else:
                target_speed += speed
            
            if curvature >= largest_curvature:
                largest_curvature = curvature
                smallest_speed = speed

        # average out 
        target_speed *= (1.0/len(steps))

        return min(self.straight_line_speed, target_speed)

class ForwardPathTracker(object):
    """ Wraps a Path object with some state that allows tracking it in a forward direction only (only allow forward progress) """


    def __init__(self, path, start_dist=0.0, max_horizon=5.0, loop_restart_tolerance=2.0):
        """
        start_time: apply time shift to the path
        start_dist: distance in path to start tracking from
        """

        self.path = path
        self.loop = path.loop
        self.loop_restart_tolerance = loop_restart_tolerance

        # everything is implicilty parametrized at speed = 1 m/s
        self.last_t = start_dist    # last 'time' on curve we tracked

        self.active_segment = self.path.get_segment_for_time(self.last_t )
        self.max_horizon = max_horizon 

        self.finished = False
        self.finish_undershoot = 5.0


    def update(self, position):
        self.current_position = position
        self._update_values()

    def _update_values(self):
        current = self.active_segment
        next_segment = self.path.get_segment_after(current)

        # print("Current start_time: {0}, current end_time: {1}".format(current.start_time, current.end_time))
        # print("next start_time: {0}, current end_time: {1}".format(next_segment.start_time, next_segment.end_time))


        closest_time = current.closest_point_time(self.current_position, self.last_t, self.max_horizon)
        closest_point = current.point_at(closest_time)
        d = np.linalg.norm(self.current_position - closest_point)

        if current != next_segment:
            next_comp_closest_time = next_segment.closest_point_time(self.current_position, self.last_t, self.max_horizon)
            next_comp_closest_point = next_segment.point_at(next_comp_closest_time)
            d_next = np.linalg.norm(self.current_position - next_comp_closest_point)

            if d_next < d:
                print("Swapping to next segment!")
                print("Last segment time range: {0}-{1}, next segment: {2}-{3}".format(current.start_time, current.end_time, next_segment.start_time, next_segment.end_time))
                # we have moved onto the next segment of the path
                self.active_segment = next_segment
                closest_time = next_comp_closest_time
                closest_point = next_comp_closest_point
                d = d_next

        self.last_t = closest_time 
        self.closest_point = closest_point
        self.closest_point_dist = d
        self.closest_tangent = self.active_segment.tangent_at(closest_time)
        self.closest_normal = self.active_segment.normal_at(closest_time)

        # perform a loop if the current segment's end time is less than the last_t we search from
        path_length = self.path.get_length()
        if self.loop and path_length != -1:
            if self.last_t + self.loop_restart_tolerance > path_length:
                print("Restarting loop tracking!")
                self.last_t = 0.0 # ie. reset to 0 distance

        if not self.loop and path_length != -1 and self.last_t + self.finish_undershoot >= self.path.get_length(): # 1 second tolerance seems to make it work!
            self.finished = True

    def get_closest_point(self):
        return self.closest_point

    def get_closest_point_distance(self):
        return self.closest_point_dist

    def get_closest_tangent(self):
        return self.closest_tangent
    
    def get_closest_point_time(self):
        return self.last_t

    def get_closest_normal(self):
        return self.closest_normal


class UnboundedSegmentException(Exception):
    pass
