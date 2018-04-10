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
            "speed": 0,
            "end_time": -1,
            "start_time": -1
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



        self.update_cached_curve(start_time, end_time, speed, steps, curve=curve_points_stamped)
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
    def update_cached_curve(self, start_time, end_time, speed, steps, curve=None):
        """
        Save to the cache the concrete path computed with the given parameters
        if `curve` is not None, just save that into the cache as it's been computed before
        """

        if self.cache['curve'] is not None and  \
           self.cache['start_time'] == start_time and \
           self.cache['end_time'] == end_time and \
           self.cache['speed'] == speed and \
           len(self.cache['curve']) == steps:

            rospy.loginfo("Curve is already cached, not updating cache")
            return

        self.cache['start_time'] = start_time
        self.cache['end_time'] = end_time
        self.cache['speed'] = speed

        if curve is None:
            curve = self.get_concrete_curve_points(start_time, end_time, speed, steps, return_type='PointStamped')
        assert type(curve[0]) == PointStamped
        self.cache['curve'] = curve 


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
    def get_unit_tangent(self, t, speed, return_type='numpy'):
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



    # compute an indicated type of tracking error
    # current options under consideration are 
    # time error: Error from position and position it should be at time T
    # crosstrack error: perpendicular distance to closest point on curve
    # hamilton (?) error: related to a point on the curve some distance ahead and crosstrack error?
    def error(self, position, heading, speed, time, error_type='time_error'):
        
        if error_type== 'time_error':
            return self.time_error(position, heading, time, speed)

        elif error_type == 'crosstrack':
            # calculate closest point on entirety of path
            # BAD for two reasons:
            # 1) may be so far off path we've approached another point and then start tracking in the opposite direction
            # 2) direction agnostic (eg just following closest points while moving)
            # 3) computationally expensive
            return self.crosstrack_error(self, position)
        else:
            raise UnknownErrorTypeException("Unkown error metric: " + error_type)

    def time_error(self, position, heading, time, speed):
        target_point = self.get_concrete_point(time, speed, return_type='numpy')

        if type(position) == Point:
            pos = np.array([position.x, position.y, position.z])
        elif type(position) == type(np.empty(3)):
            pos = position
        else:
            rospy.logerr("Can't handle position of type: " + str(type(position)))
            raise Exception("Unknown position type")
        
        if type(heading) == Quaternion:
            # convert Quaternion geomtry_msg to numpy
            q = quat_msg_to_numpy(heading)
            # convert numpy quaternion into a direction vector
            heading = quat_mult_point(q, np.array([1.0, 0.0, 0.0]))
        elif type(heading) == type(np.empty(3)) and heading.shape == (3,):
            pass
        elif type(heading) == type(np.empty(4)) and heading.shape == (4,):
            heading = quat_mult_point(heading, np.array([1.0, 0.0, 0.0]))
        else:
            rospy.logerr("Heading provided is neither a orientation Quaternion msg nor a numpy 3-tuple representing a direction vector")
            raise Exception("Invalid heading type")

        heading = normalize(heading)    

        # target_tangent = self.get_unit_tangent(time, speed)
 
        # use a custom error here:
        # compute angle between position and target point => theta
        # compute distance
        # return (0.1 + sin(theta))*distance
        # this ensure distance is always accounted for but also scaled up if the angle is bad
 
        diff = target_point - pos  
        distance = np.linalg.norm(diff)
        dotprod = np.dot(heading, diff)
        # this dot product is equal to cos(theta) when we divide through by `|diff|` 
        # want to compute sin(theta)
        # sin(theta) = sqrt(1 - cos(theta)**2)
        if distance > EPSILON:
            dotprod = dotprod/distance  # distance is large enough to avoid numerical errors
            sin_theta = np.sqrt(1 - dotprod**2)
        else:
            sin_theta = 9999 # value doesn't matter since distance ~= 0

        error = (0.1+sin_theta) * distance

        return error 
    
    def crosstrack_error(self, position, time, speed, distance_resolution=1.0):
        """
        Used when curve tracking has started
        """
        update_rate = speed/distance_resolution # number of points to sample per second at constant speed
        if self.cached_curve['end_time'] is not None and self.cached_curve['start_time'] is not None and self.cached_curve['curve'] is not None:
            dt = self.cached_curve['end_time'] - self.cached_curve['start_time']
            dt = dt / len(self.cached_curve['curve'])
        else:
            dt = update_rate   
        
        end_time = max(2*time, self.cached_curve['end_time'])
        total_steps = (end_time - self.tracking_start_time)/dt
        self.update_cached_curve(self.tracking_start_time,
                                 end_time,
                                 speed=speed,
                                 steps=total_steps)

        curve = self.cached_curve

        
        # TODO implement crosstrack distance as error


class UnknownErrorTypeException(Exception):
    pass
class UnknownPathTypeException(Exception):
    pass
class NotImplementedException(Exception):
    def __init__(self, msg):
        super(NotImplementedException, self).__init__("Not Implemented!" + msg)
