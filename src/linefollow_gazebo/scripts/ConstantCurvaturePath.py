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
        
        self.start_time = 0.0

        # to avoid repeatedly computing curve points if ever needed
        # save them as PointStamped and reuse
        self.concrete_curve_cache = None

    def begin(self, current_time, current_position, current_orientation, speed):
        # translate and rotate path centered at (0,0,0) and x-axis aligned to match current_pose and current_orientation
        self.speed = speed        
        self.transform = TransformStamped()

        if type(current_position) == Point:
            x, y, z = current_position.x, current_position.y, current_position.z
            pos = np.array([x, y, z])
        elif type(current_position) == type(np.empty(3)):
            pos = current_position
        else:
            rospy.logerr("Unkown position type: " + str(type(current_position)))
            return
        pos_delta = pos - self.init_point

        self.transform.translation.x = pos_delta[0] 
        self.transform.translation.y = pos_delta[1] 
        self.transform.translation.z = pos_delta[2] 
        

        # just going to assume init_orientation is x-axis aligned...
        if type(current_orientation) == Quaternion:
            self.transform.rotation = current_orientation
        elif type(current_orientation) == type(np.empty(4)):
            self.transform.rotation = Quaternion(*current_orientation)
        else:
            rospy.logerr("Unknown orientation type: " + str(type(current_orientation)))
            return
      
        self.start_time = current_time

        self.concrete_curve_cache = None

    def get_concrete_curve_points(self, start_time, end_time, speed=1.0, steps=100, return_type='numpy')
        curve_points_stamped = self.get_points_transformed(start_time=start_time,
                                                           end_time=end_time,
                                                           speed=speed,
                                                           transform=self.transform,
                                                           steps=steps)
        self.concrete_curve_cache = curve_points_stamped

        if return_type == 'numpy':
            points = np.array([np.array(p.point.x, p.point.y, p.point.z) for p in curve_points_stamped])
            return points
        elif return_type == 'Point':
            rospy.logerr("Point type concrete path not implemented")
            raise NotImplementedException("Point type not implemented")
        elif return_type == 'PointStamped'):
            return curve_points_stamped
        else:
            raise UnknownPathTypeException("Unknown return type requested: " + return_type)


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
            abstract_path.append(self.get_point(t, speed, return_type=return_type))
        
        return abstract_path
        
   
    # get points on curve relative to abstract location
    # other methods can apply required transforms such as translation/rotation
    def get_point(self, t, speed, return_type='numpy'):
        dt = t - self.start_time
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
            pt = self.init_point + np.array([dx, dy, dz])

        if return_type == 'numpy':
            return pt
        elif return_type == 'Point':
            return Point(*pt)
        elif return_type == 'PointStamped':
            return PointStamped(point=Point(*pt))
        else:
            rospy.logerr("Unknown return type: " + return_type)
            raise UnknownPathTypeException("Unknown requested point return type: " + return_type) 



    def error(self, position, speed, time, error_type='time_error'):
        
        if error_type='time_error':
            return self.time_error(position, time, speed)

        elif error_type='crosstrack':
            # calculate closest point on entirety of path
            # BAD for two reasons:
            # 1) may be so far off path we've approached another point and then start tracking in the opposite direction
            # 2) direction agnostic (eg just following closest points while moving)
            # 3) computationally expensive
            return self.crosstrack_error(self, position)

        else:
            raise UnknownErrorTypeException("Unkown error metric: " + error_type)

    def time_error(self, position, time):
        target_point = self.get_point(time, speed) 

        # TODO implement some error as a function of current position and target position
    
    def crosstrack_error(self, position):
        if self.concrete_curve_cache == None
            self.get_concrete_curve_points() # updates cache internally

        # TODO implement crosstrack distance as error


class UnknownErrorTypeException(Exception):
    pass
class UnknownPathTypeException(Exception):
    pass
class NotImplementedException(Exception):
    def __init__(self, msg):
        super(NotImplementedException, self).__init__("Not Implemented!" + msg)
