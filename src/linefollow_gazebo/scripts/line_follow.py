#!/usr/bin/env python

import numpy as np
import rospy

from sensor_msgs.msg import Imu
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry

import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from std_msgs.msg import Float64, Bool

from PriusControlMsgGenerator import PriusControlMsgGenerator
from Positioning import TruePositioning
from ConstantCurvaturePath import ConstantCurvaturePath
from PID import PID
from VisualizeMarker import VisualizeMarker
from helper import *

class LineFollowController(object):
    def __init__(self, curve, positioning=None):
        rospy.loginfo("Create LineFollowController object")
        self.rate = rospy.get_param("~rate", 20.0)  # low rate to show 
        self.period = 1.0 / self.rate

        self.positioning = positioning
        self.prius_msg_generator = PriusControlMsgGenerator()
        # topic to publish prius Control message
        self.prius_move = rospy.Publisher("/prius", Control, queue_size=5)

        self.prius_steering_pid = PID(kp=7.0, ki=0.9, kd=0.00, setpoint=0.0, lower_limit=-1.0, upper_limit=1.0, windup=2.0)

        self.curve = curve
        self.previous_v_x = 0.0
        self.tracking = False

        self.visualize = VisualizeMarker(rate=4)

    def begin(self, throttle):
        self.throttle = throttle 
        # get vehicle moving up to constant speed

        self.repeated_msg = self.prius_msg_generator.forward(self.throttle, 0.0) # get speed up until acceleration stops
        self.prius_move.publish(self.repeated_msg)

        # Wait until at speed to start tracking! 
        self.timer = rospy.Timer(rospy.Duration(self.period), self.wait_until_at_speed)

    def wait_until_at_speed(self, event, accel_tolerance=0.5):

        pose = self.positioning.get_pose()
        
        if pose is None:
            print("Pose is None")
            rospy.loginfo("Pose received from positioning is None, will accelerate when Pose is received")
            return

        vel_linear = pose.twist.twist.linear
        v_x, v_y, v_z = vel_linear.x, vel_linear.y, vel_linear.z
        dv_x = v_x - self.previous_v_x
        self.previous_v_x = v_x

        # if change in velocity is less than tolerance, we've reached desired speed

        # check if have stopped accelerating and velocity has some magnitude
        # that means we've finished accelerating
        if dv_x < accel_tolerance and np.abs(v_x) > 0.2:
            self.avg_speed = np.sqrt(v_x**2 + v_y**2 + v_z**2) # save attained speed for later use    
            self.timer.shutdown() # stop this update loop
            self.begin_track_path() # begin path tracking!
            return

        self.prius_msg_generator.update_msg(self.repeated_msg) #update seq number
        rospy.loginfo("Sending Prius Control msg: {}".format(self.repeated_msg))
        self.prius_move.publish(self.repeated_msg)

    def begin_track_path(self):

        # don't attempt to re-initalize tracking if it's running
        # messes with pubs/subs
        if self.tracking:
            rospy.logerror("Already tracking path!")
            return

        now = rospy.get_rostime()
        current_pose = self.positioning.get_pose()
        
        rospy.loginfo("Starting path tracking at: " + str(current_pose))

        pos = current_pose.pose.pose.position
    
        self.last_target_point = get_as_numpy_position(pos)

        self.curve.begin(now.to_sec(), pos,
                         current_pose.pose.pose.orientation)
      

        self.timer = rospy.Timer(rospy.Duration(self.period), self.track_path_update)

        self.tracking = True


    def track_path_update(self, event):
        if not self.tracking:
            rospy.logerror("Path tracking updatecalled when not tracking, call begin_track_path_first!")
            return

        now = rospy.get_rostime()
        secs = now.to_sec()
        
        # get position from either true or calculated position (doesn't matter for testing)
        pose = self.positioning.get_pose() # either a nav_msgs/Odometry or some other msg type

        vel = pose.twist.twist.linear
        speed = np.linalg.norm([vel.x, vel.y, vel.z])

        # some rough estimate of our average speed
        self.avg_speed = self.avg_speed * 0.99 + speed*0.01

        position = pose.pose.pose.position
        orientation = pose.pose.pose.orientation

        # orientation should be very close to direction of linear velocity, use this instead       

        heading = pose.twist.twist.linear
        err, target_point = self.curve.closest_error(position, heading, self.last_target_point)
        self.last_target_point = target_point

        p = [position.x, position.y, position.z]
        self.visualize.draw_n_points([p, self.last_target_point])

        steering_control = self.prius_steering_pid.update(secs, err)
        rospy.loginfo("Tracking error: {0}, PID response: {1}".format(err, steering_control, self.avg_speed))
        prius_msg = self.prius_msg_generator.forward(self.throttle, steering_control)
        self.prius_move.publish(prius_msg)
        
        
    def stop_path_tracking(self):
        self.tracking = False
        self.timer.shutdown() 

        

if __name__ == "__main__":
    rospy.init_node("line_follow_py")


    # wait until clock messages arrive
    # and hence simulation is ready
    while rospy.get_time() == 0:
        rospy.sleep(1)

    rospy.sleep(2)
    rospy.loginfo("Clock is no longer zero")

    positioning = TruePositioning()
    path = ConstantCurvaturePath(curvature=0.04) # turn radius limit around curvature=0.3
    line_follow = LineFollowController(curve=path, positioning=positioning)
    line_follow.begin(throttle=0.2)
    rospy.spin()