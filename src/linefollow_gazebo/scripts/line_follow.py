#!/usr/bin/env python

import math
import numpy as np
from numpy import pi as PI
import rospy
import tf2_py as tf2
import tf2_ros
import tf_conversions.transformations as tf_transformations

from sensor_msgs.msg import Imu
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry

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
        self.init_point = np.array([0.0, 0.0, 0.0)]
        # TODO check this is the right Quaternion for x-axis aligned orientation
        self.init_orientation = tf_transformations.quaternion_from_euler(0, 0, 0)
        self.curvature = curvature
        if self.curvature != 0.0:
            self.r = 1/self.curvature
        self.time_parametrized = time_parametrized
        
        self.start_time = 0.0

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


    # for debugging, test transformed paths 
    def get_points_transformed(self, start_time, end_time, transform, speed=1.0, steps=100):
        path = self.get_abstract_curve_points(start_time, end_time, speed, steps, return_type='PointStamped')
        # apply transform to all the points
        for i in range(len(path)):
            path[i] = tf2_geometry_msgs.do_transform_point(path[i], transform)
        return path


    # as per ROS/Gazebo standards, speed is m/s
    def get_abstract_curve_points(self, start_time, end_time, speed=1.0, steps=100, return_type='numpy');
        """ 
        Computes a set of `steps` points representing the time-parametrized curve
        from t=`start_time` to t=`end_time` traversed at a given constant speed
        TODO variable speeds
        returns as a numpy array of numpy arrays, Point, or PointStamped
        """

        abstract_path = np.empty(steps)
        dt = (float(end_time) - start_time) / steps
        for (t,i) in enumerate(np.arange(start_time, end_time, dt)):
            abstract_path[i] = self.get_point(t, speed, return_type=return_type)
        
        return abstract_path
        
   
    # get points on curve relative to abstract location
    # other methods can apply required transforms such as translation/rotation
    def get_point(self, t, speed, return_type='numpy'):
        dt = t - self.start_time
        if self.curvature == 0.0:
            # straight line along x axis-from start point
            pt = self.init_point + np.array([dt*speed, 0,0, 0.0)]

        else:
            # get coefficient for within x(t) = R*cos(a*t), y(t) = R*sin(a*t)
            # such that the speed is constant as given
            # works out to a = sqrt(speed)/R
            # => a = sqrt(speed) * curvature
            a = np.sqrt(speed) * self.curvature
            dx = self.r * np.cos(a * (dt - np.pi/2)) # subtract pi/2 to start curve at x = 0 and increasing
            dy = self.r * np.sin(a * (dt - np.pi/2)) + self.r # add self.r to start at (x,y) = (0,0) and both increasing
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
            raise TypeError("Uknown requested point return type: " + return_type) 

class Line_Follow(object):
    def __init__(self, positioning=None):
        rospy.loginfo("Create Line_Follow object!")
        rospy.Subscriber("/imu", Imu, self.on_imu) 
        self.rate = rospy.get_param("~rate", 20.0)
        self.period = 1.0 / self.rate

        self.positioning = positioning

        self.started = True
        self.timer = rospy.Timer(rospy.Duration(self.period), self.update)

    def on_imu(self, imu_msg):
        rospy.loginfo_throttle(10, str(imu_msg.linear_acceleration))
        self.last_imu = imu_msg
        

    def update(self):
        if not self.started:
            return
        # geti map position from either true or calculated position (doesn't matter for testing)
        pose = self.positioning.get_pose()
        position = pose.pose.pose.position

        
        
        
        prius_msg = Control(shift_gears=3, throttle=
        

class True_Position(object):
    def __init__(self):
        rospy.loginfo("Create True Position following")
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.on_odom)
        self.last_post = None

    def on_odom(self, odom_msg):
        rospy.loginfo_throttle(10, odom_msg)
        self.last_pose = odom_msg

    def get_pose(self):
        return self.last_pose


if __name__ == "__main__":
    rospy.init_node("line_follow_py")
    positioning = True_Position()
    line_follow = Line_Follow(positioning=positioning)    
    rospy.spin()
