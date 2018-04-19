import numpy as np
import rospy
import tf2_py as tf2
import tf2_ros
import tf_conversions

tf_transformations = tf_conversions.transformations

import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from helper import *

class ConstantCurvaturePath(object):
    """
    Create an abstract representation of a constant curvature path
    When begin() is called, it takes a pose and orientation and the path is then translated and rotated to that start point   
    
    Designed for path tracking
    Time parametrized requires a speed to work when begin() is called
    """

    def __init__(self, curvature, time_parametrized=True):
        self.init_point = np.array([0.0, 0.0, 0.0])
        # TODO check this is the right Quaternion for x-axis aligned orientation
        self.init_orientation = tf_transformations.quaternion_from_euler(0, 0, 0)
        self.curvature = curvature
        if self.curvature != 0.0:
            self.r = np.abs(1/self.curvature)
        self.time_parametrized = time_parametrized
       
        self.tracking_start_time = 0.0

        # to avoid repeatedly computing curve points if ever needed
        # save them as PointStamped and reuse
        self.cache = {
            "curve": None,
            "tangents": None,
            "speed": 0,
            "end_time": -1,
            "start_time": -1,
            "type": None
        }

    def begin(self, current_time, current_position, current_orientation):
        """begin

        :param current_time: current sim time (`time` from rospy.get_rostime()) 
        :param current_position: current position given as a Point or numpy array (x,y,z)
        :param current_orientation: current orientation quaternion as Quaternion msg or numpy array (x,y,z,w)
        """
        
        rospy.loginfo("starting curve tracking at position: {}".format(current_position))
        rospy.loginfo("starting curve tracking at orientation: {}".format(current_orientation)) 


        # translate and rotate path centered at (0,0,0) and x-axis aligned to match current_pose and current_orientation
        self.path_start_transform = TransformStamped()

        if type(current_position) == Point:
            x, y, z = current_position.x, current_position.y, current_position.z
            pos = np.array([x, y, z])
        elif type(current_position) == type(np.empty(3)):
            pos = current_position
        else:
            rospy.logerr("Unkown position type: " + str(type(current_position)))
            return
        pos_delta = pos - self.init_point

        self.path_start_transform.transform.translation.x = pos_delta[0] 
        self.path_start_transform.transform.translation.y = pos_delta[1] 
        self.path_start_transform.transform.translation.z = pos_delta[2] 
       
        # store in numpy too for cheaper manipulations
        # can remove one method later
        self.path_start_translate_numpy = pos_delta

        # just going to assume init_orientation is x-axis aligned...
        if type(current_orientation) == Quaternion:
            self.path_start_transform.transform.rotation = current_orientation
            self.path_start_rotate_numpy_quat = np.array([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
        elif type(current_orientation) == type(np.empty(4)):
            self.path_start_transform.transform.rotation = Quaternion(*current_orientation)
            self.path_start_rotate_numpy_quat = current_orientation
        else:
            rospy.logerr("Unknown orientation type: " + str(type(current_orientation)))
            return
        
      
        self.tracking_start_time = current_time 
        rospy.loginfo("Tracking start time: " + str(self.tracking_start_time)) 

    def get_concrete_curve_points(self, start_time, end_time, speed=1.0, steps=100, return_type='numpy'):
        curve_points_stamped = self.get_points_transformed(start_time=start_time,
                                                           end_time=end_time,
                                                           speed=speed,
                                                           transform=self.path_start_transform,
                                                           steps=steps)



        if return_type == 'numpy':
            points = np.array([np.array([p.point.x, p.point.y, p.point.z]) for p in curve_points_stamped])
            return points
        elif return_type == 'Point':
            rospy.logerr("Point type concrete path not implemented")
            raise NotImplementedException("Point type not implemented")
        elif return_type == 'PointStamped':
            return curve_points_stamped
        else:
            raise UnknownPathTypeException("Unknown return type requested: " + return_type)


    # TODO test this
    def update_cached_curve(self, start_time, end_time, speed, steps, curve=None, tangents=None, return_type='PointStamped'):
        """
        Save to the cache the concrete path computed with the given parameters
        if `curve` is not None, just save that into the cache as it's been computed before
        """

        if self.cache['curve'] is not None and  \
           self.cache['tangents'] is not None and \
           self.cache['start_time'] == start_time and \
           self.cache['end_time'] == end_time and \
           self.cache['speed'] == speed and \
           len(self.cache['curve']) == steps and \
           self.cache['type'] == return_type:

            rospy.loginfo("Curve is already cached, not updating cache")
            return

        self.cache['start_time'] = start_time
        self.cache['end_time'] = end_time
        self.cache['speed'] = speed

        if curve is None:
            curve = self.get_concrete_curve_points(start_time, end_time, speed, steps, return_type=return_type)

        if tangents is None:
            tangents = self.get_concrete_unit_tangents(start_time, end_time, speed, steps, return_type='numpy')
        self.cache['curve'] = curve 
        self.cache['tangents'] = tangents
        assert self.cache['tangents'].shape[0] == self.cache['curve'].shape[0], "Mismatched curve and tangents!"

    # for debugging, test transformed paths 
    def get_points_transformed(self, start_time, end_time, transform, speed=1.0, steps=100):
        path = self.get_abstract_curve_points(start_time, end_time, speed, steps, return_type='PointStamped')
        # apply transform to all the points
        for i in range(len(path)):
            path[i] = tf2_geometry_msgs.do_transform_point(path[i], transform)
        return path


    # as per ROS/Gazebo standards, speed is m/s
    def get_abstract_curve_points(self, start_time, end_time, speed=1.0, steps=100, return_type='numpy'):
        """ 
        Computes a set of `steps` points representing the time-parametrized curve
        from t=`start_time` to t=`end_time` traversed at a given constant speed
        TODO variable speeds
        returns as a numpy array of numpy arrays, Point, or PointStamped
        """

        abstract_path =[] 
        dt = (float(end_time) - start_time) / steps
        for t in np.arange(start_time, end_time, dt):
            abstract_path.append(self.get_abstract_point(t, speed, return_type=return_type))
        
        return abstract_path
        
   
    # get points on curve relative to abstract location
    # other methods can apply required transforms such as translation/rotation
    def get_abstract_point(self, t, speed, return_type='numpy'):
        dt = t - self.tracking_start_time
        if self.curvature == 0.0:
            # straight line along x axis-from start point
            pt = self.init_point + np.array([dt*speed, 0.0, 0.0])

        else:
            # get coefficient for within x(t) = R*cos(a*t), y(t) = R*sin(a*t)
            # such that the speed is constant as given
            # works out to a = sqrt(speed)/R
            # => a = sqrt(speed) * curvature
            a = np.sqrt(speed) * self.curvature
            dx = self.r * np.cos(a * dt - np.pi/2) # subtract pi/2 to start at x = 0 and increasing
            dy = self.r * np.sin(a * dt - np.pi/2) + (self.r) # add radius to offset to (x,y) = (0,0) and both increasing (if positive curvature)
            dz = 0.0
            pt = np.array([dx, dy, dz])

        if return_type == 'numpy':
            return pt
        elif return_type == 'Point':
            return Point(*pt)
        elif return_type == 'PointStamped':
            return PointStamped(point=Point(*pt))
        else:
            rospy.logerr("Unknown return type: " + return_type)
            raise UnknownPathTypeException("Unknown requested point return type: " + return_type) 


    def get_concrete_point(self, t, speed, return_type='numpy'):
        abstract_point = self.get_abstract_point(t, speed, return_type='numpy')

        # apply transforms
        # apply rotation first
        abstract_point = quat_mult_point(self.path_start_rotate_numpy_quat, abstract_point)
        # then apply translation
        concrete_point = abstract_point + self.path_start_translate_numpy

        if return_type == 'numpy':
            return concrete_point
        elif return_type == 'Point':
            return Point(*concrete_point)
        elif return_type == 'PointStamped':
            return PointStamped(point=Point(*concrete_point))
        else:
            rospy.logerr("Unknown return type: " + return_type)
            raise UnknownPathTypeException("Unknown requested concrete point return type: " + return_type) 

    # gets unit tangent of the curve as numpy vector
    def get_abstract_unit_tangent(self, t, speed, return_type='numpy'):
        if self.curvature == 0.0:
            vec = np.array([1.0, 0.0, 0.0])
        else:
            dt = t - self.tracking_start_time
            a = np.sqrt(speed) * self.curvature
            # use derivative of equations in `get_abstract_point` to get x'(t) and y'(t), z'(t) = 0 always
            x_ = self.r * (-1) * a * np.sin(a*dt - np.pi/2)
            y_ = self.r * a * np.cos(a * dt - np.pi/2)
            vec = np.array([x_, y_, 0.0])

        T = vec/np.dot(vec, vec) # normalize to unit vector

        if return_type == 'numpy':
            return T
        else:
            rospy.logerr("Can only return numpy array for tangent")
            raise TypeError("Bad type for tangent: " + return_type)

    def get_concrete_unit_tangents(self, start_time, end_time, speed=1.0, steps=100, return_type='numpy'):
        """ 
        Computes a set of `steps` points representing the time-parametrized curve's unit tangents
        from t=`start_time` to t=`end_time` traversed at a given constant speed
        returns given type, but depends on abstract_unit_tangent implementation
        """

        tangents = []
        dt = (float(end_time) - start_time) / steps
        for t in np.arange(start_time, end_time, dt):
            tangents.append(self.get_concrete_unit_tangent(t, speed, return_type=return_type))

        if return_type == 'numpy':
            return np.array(tangents)
        else:
            rospy.logerr("Can only return numpy tangents")
            raise TypeError("Bad type for tangents: " + return_type)
        
    def get_concrete_unit_tangent(self, t, speed, return_type='numpy'):
        tangent = self.get_abstract_unit_tangent(t, speed, return_type=return_type)

        # apply rotation to the abstract tangent
        rot_quat = self.path_start_rotate_numpy_quat
        rotated_tangent = quat_mult_point(rot_quat, tangent)
        return rotated_tangent

    """
    "
    # compute an indicated type of tracking error
    # current options under consideration are 
    # time error: Error from position and position it should be at time T
    # crosstrack error: perpendicular distance to closest point on curve
    # hamilton (?) error: related to a point on the curve some distance ahead and crosstrack error?
    def error(self, position, heading, speed, time, error_type='time_error'):
        
        if error_type== 'time_error':
            return self.time_error(position, heading, time, speed)

        elif error_type == 'closest': # aka 'crosstrack' variant
            # calculate closest point on entirety of path
            # BAD for two reasons:
            # 1) may be so far off path we've approached another point and then start tracking in the opposite direction
            # 2) direction agnostic (eg just following closest points while moving)
            # 3) computationally expensive (negated with caching)
            return self.closest_error(self, position, heading)

        elif error_type == 'windowed':
            return self.windowed_error(position, heading, time=time, speed=speed)
        else:
            raise UnknownErrorTypeException("Unkown error metric: " + error_type)
    "
    """

    def _calculate_error(self, position, current_steer_angle, velocity, target, target_direction):
        
        position = get_as_numpy_position(position)
        heading = normalize(get_as_numpy_direction_vec(heading))
        diff = target - position         
        crosstrack_distance = np.linalg.norm(diff)


        # need forward velocity in direction of vehicle orientation...

        speed = np.linalg.norm(get_as_numpy_velocity_vec(velocity))

        target_direction = normalize(target_direction)
        
       
        angle_heading_target = angle_from_to(heading, target_direction)
        angle_steering_target = angle_from_to(target_direction, current_steer_angle)
        

        weighting = 0.2 
        error = distance * (weighting * angle_to_target_position + (1-weighting)*angle_heading_target)/np.pi

        #error = distance * angle / np.pi + sign*0.1*distance 
#        error = distance * angle/np.pi + sign*0.1*distance

        print("Heading: {3}, Diff: {4}, Angle to target pos: {0}, angle diff target heading: {1}, distance: {2}".format(angle_to_target_position, angle_heading_target, distance, heading, diff))

        return error 

    def time_error(self, position, heading, time, speed):
        target_point = self.get_concrete_point(time, speed, return_type='numpy')

        pos = get_as_numpy_position(position)
 

        return self._calculate_error(pos, heading, target_point)


    def closest_point_tangent(self, position, heading, last_closest_point, distance_resolution=0.1, max_horizon=None):
        # set a default max horizon equal to one radius or 10m whichever is less 
        if max_horizon is None:
            if self.curvature == 0:
                max_horizon = 10.0
            else:
                max_horizon = min(10.0, self.r)
        
        pos = get_as_numpy_position(position)
      
        if self.cache['curve'] is None or self.cache['tangents'] is None:
            #note : speed is 1.0 for these => max_horizon/1.0 = time into the future for that horizon 
            self.update_cached_curve(self.tracking_start_time, self.tracking_start_time + max_horizon/1.0, 1.0, max_horizon/distance_resolution, return_type='numpy')

        # the cache must contain last_closest_point
        curve = self.cache['curve']
        index = 0  
        point = curve[index] # default to first point in list
        for i, pt in enumerate(curve):
            if np.array_equal(pt, last_closest_point):
                index = i
                point = pt
                break
      
        cached_start_time = self.cache['start_time']
        cached_end_time = self.cache['end_time']
        dt = cached_end_time - cached_start_time
        steps = len(curve) 
        index_time = float(index)/steps * dt + cached_start_time 

        start_time = index_time 
        speed = 1
        steps = max_horizon/distance_resolution
        end_time = start_time + max_horizon/speed

        rospy.loginfo_throttle(0.25, "Point: {0}, index: {1}, index time = start time: {2}, end_time using horizon: {3}, cached_end_time: {4}".format(point, index, index_time, end_time, cached_end_time))

        # check if the end time we want to search to is in the cached computed points
        if end_time > cached_end_time:
            # TODO update cached curve to include points in the horizon too
            save_ahead_cache_mult = 4 
            self.update_cached_curve(index_time, 
                                     index_time + save_ahead_cache_mult * speed * max_horizon, 
                                     speed,
                                     steps = save_ahead_cache_mult * max_horizon / distance_resolution,
                                     return_type='numpy')

            # update curve, index, and point to reflect cache 
            curve = self.cache['curve']
            index = 0
            assert np.linalg.norm(curve[index] - point) < distance_resolution, "Updated curve has point different from previous closest point: curve[0] {0} vs previous closest: {1}".format(curve[index], point)
            point = curve[index]
       
        # cut off cache so we can only ever move forward on the curve and not lock onto a remote point
        self.cache['curve'] = self.cache['curve'][index:]
        self.cache['tangents'] = self.cache['tangents'][index:]
        self.cache['start_time'] = index_time

        closest_index, closest_dist, closest_point = index, np.linalg.norm(point - pos), point 
        # add a bias towards making progress
        for i, pt in enumerate(curve):
            d = np.linalg.norm(pt - pos)
            # allow some slack to pick a forward point if it's close enough
            if d - distance_resolution/2 < closest_dist:
                closest_dist = d
                closest_point = pt
                closest_index = i
       
        closest_point_tangent = self.cache['tangents'][closest_index]

        return closest_point, closest_point_tangent 

    """
    "
    def closest_concrete_point_in_window(self, position, speed, time, time_window=10.0, distance_resolution=1.0):
       
        start_time = time - time_window
        end_time = time + time_window

        # calculate number of steps to compute to achieve the desired resolution
        total_dist = speed * 2*time_window 
        steps = total_dist / distance_resolution
        
        pts = self.get_concrete_curve_points(start_time, end_time, speed=speed, steps=steps, return_type='numpy')

        if type(position) == type(numpy.empty(3)):
            pos = position
        elif type(position) == Point:
            pos = np.array([position.x, position.y, position.z])
        else:
            rospy.logerr("Unknown position type: {}".format(type(position)))
            return

        distances = np.linalg.norm(pts - pos)
        index = np.argmin(distances)
        closest_point = pts[index]

        # return both the point and the time on the path of that point
        
        return closest_point, start_time + 2*time_window * (index/float(steps))
    "
    """            

    #def next_closest_point(self, position, last_time, distance_resolution=1.0):



class UnknownErrorTypeException(Exception):
    pass
class UnknownPathTypeException(Exception):
    pass
class NotImplementedException(Exception):
    def __init__(self, msg):
        super(NotImplementedException, self).__init__("Not Implemented!" + msg)
