#!/usr/bin/env python

import abc

import math
import numpy as np
import rospy
import tf2_py as tf2
import tf2_ros
import tf_conversions

from sensor_msgs.msg import Imu
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry

import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

class PriusControlMsgGenerator(object):
    def __init__(self):
        self.seq = 0

    def populate_header(self, prius_msg):
        header = prius_msg.header

        header.seq = self.seq
        self.seq += 1
        
        now = rospy.get_rostime()

        header.stamp.s = now.secs
        header.stamp.ns = now.nsecs

        header.frame_id = 0 # TODO frame ID of base_link...?

    def forward(self, throttle, steer_angle):
        assert throttle >= 0.0 and throttle <= 1.0, "Require throttle to be within range [0.0, 1.0]"
        assert steer_angle >= 1.0 and steer_angle <= 1.0, "Require steer angle to be in range [-1.0, 1.0]"
        msg = Control()
        self.populate_header(msg)
        msg.throttle = throttle
        msg.steer = steer_angle
        msg.brake = 0.0
        msg.shift_gears = 2

        return msg

class LineFollowController(object):
    def __init__(self, curve, throttle, positioning=None):
        rospy.loginfo("Create LineFollowController object!")
#        rospy.Subscriber("/imu", Imu, self.on_imu) 
        self.rate = rospy.get_param("~rate", 20.0)  # low rate to show rate
        self.period = 1.0 / self.rate

        self.positioning = positioning
        self.prius_msg_generator = PriusControlMsgGenerator
        self.prius_control_pub = rospy.Publisher("/prius", Control, 5)

        self.curve = curve
        self.previous_v_x = 0.0

    def begin(self, throttle):
        self.throttle = throttle 
        # get vehicle moving up to constant speed

        self.repeated_msg = self.prius_msg_generator.forward(self.throttle, 0.0) # get speed up until acceleration stops
        self.prius_control_pub.publish(self.repeated_msg)

        # Wait until at speed to start tracking! 
        self.timer = rospy.Timer(rospy.Duration(self.period), self.wait_until_at_speed)

    def wait_until_at_speed(self, event, accel_tolerance=0.01):

        pose = self.positioning.get_pose()

        vel_linear = pose.twist.twist.linear
        v_x = vel_linear.x
        dv_x = v_x - self.previous_v_x
        self.previous_v_x = v_x

        # if change in velocity is less than tolerance, we've reached desired speed

        # check if have stopped accelerating and velocity has some magnitude
        # that means we've finished accelerating
        if dv_x < accel_tolerance and np.abs(v_x) > 0.01:
			self.speed = v_x # save attained speed for later use	
            self.timer.shutdown() # stop this update loop
            self.begin_track_path() # begin path tracking!
            return

        self.prius_control_pub.publish(self.repeated_msg)

    def begin_track_path(self):

        now = rospy.get_rostime()
        current_pose = self.positioning.get_pose()
        
        rospy.loginfo("Starting path tracking at: " + str(current_pose))

        self.curve.begin(now.to_sec(), current_pose.pose.pose.position,
                         current_pose.pose.pose.rotation)
        
        
        self.timer = rospy.Timer(rospy.Duration(self.period), self.track_path_update)
        self.tracking = True

        
    def track_path_update(self, event):
        if not self.tracking:
            rospy.logdebug("Path tracking update called when not tracking")
            return
        # get map position from either true or calculated position (doesn't matter for testing)
        pose = self.positioning.get_pose() # either a nav_msgs/Odometry or some other msg type
        
        position = pose.pose.pose.position
        orientation = pose.pose.pose.rotation

		# orietation should be very close to 		
		# the Transform from vehicle to world coordinates
		# of the vehicle's orientation
		#TODO this sanity check

        now = rospy.get_rostime()
        secs = now.to_sec()

        err = self.curve.error(position, heading, self.tracking_speed, secs, error_type='time_error')

        # TODO implement PID/LQR controller based on this error

		
        
        
        

        
class Positioning(object):
    def __init__(self):
        self.last_pose = None

    def on_update(self, msg):
        self.last_pose = msg

    def get_pose(self):
        return self.last_pose

class True_Position(Positioning):
    def __init__(self):
        rospy.loginfo("Create True Positioning")
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.on_update)


# Next steps...
class EKF_Position(Positioning):
    def __init__(self):
        pass
    
    def on_imu(self, imu_msg):
        rospy.loginfo_throttle(10, str(imu_msg.linear_acceleration))
        self.last_imu = imu_msg

if __name__ == "__main__":
    rospy.init_node("line_follow_py")
    positioning = True_Position()
    line_follow = Line_Follow(positioning=positioning)    
    rospy.spin()
