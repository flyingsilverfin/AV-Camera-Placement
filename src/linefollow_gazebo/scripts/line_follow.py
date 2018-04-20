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
from Positioning import TruePositioning, EKFPositioning
from ConstantCurvaturePath import ConstantCurvaturePath
from PID import PID
from VisualizeMarker import VisualizeMarker
from helper import *

class LineFollowController(object):
    def __init__(self, curve, positioning=None):
        rospy.loginfo("Create LineFollowController object")
        self.rate = rospy.get_param("~rate", 50.0)   
        self.period = 1.0 / self.rate

        self.positioning = positioning
        self.prius_msg_generator = PriusControlMsgGenerator()
        # topic to publish prius Control message
        self.prius_move = rospy.Publisher("/prius", Control, queue_size=5)


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

        pose = self.positioning.get_odom()
        
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
        current_pose = self.positioning.get_odom()
        
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

        steer_angle = self.hoffman_control()
        rospy.loginfo("New steering angle (hoffman): {0}".format(steer_angle))
        prius_msg = self.prius_msg_generator.forward(self.throttle, steer_angle)
        self.prius_move.publish(prius_msg)
       

    def hoffman_control(self, steering_angle_limit=0.8727, viz=True): # pull angle limit from URDF
        """ Returns a steering command from -1.0 to 1.0 according to hoffman controller """
        now = rospy.get_rostime()
        secs = now.to_sec()

        pose = self.positioning.get_odom()

        heading = vel = get_as_numpy_velocity_vec(pose.twist.twist.linear) # NOTE: orientation from `pose` should be close to heading using velocity!
        position = get_as_numpy_position(pose.pose.pose.position)
        
        closest_point, closest_heading = self.curve.closest_point_tangent(position, heading, self.last_target_point, max_horizon=2.0)
        self.last_target_point = closest_point

        if viz:
            self.visualize.draw_n_points([position, self.last_target_point])

        crosstrack_dist = np.linalg.norm(closest_point - position)
        new_wheel_angle = self.hoffman_steer_angle(crosstrack_dist, vel, closest_heading, k=3.30)

        # convert angle to command angle
        steer_command = new_wheel_angle/steering_angle_limit
        if steer_command < -1.0:
            return -1.0
        elif steer_command > 1.0:
            return 1.0

        return steer_command

    
    
    def hoffman_steer_angle(self, crosstrack_distance, velocity, target_heading, k=1.0):
        """ Calculates steering angle directly (no feedback) using Stanley 2006 line tracking function (Hoffman et al)
        """
        
        heading = normalize(velocity)
        speed = np.linalg.norm(velocity)
        #angle = angle_from_to(heading, target_heading)
        angle = angle_from_to(target_heading, heading)
        rospy.loginfo("Angle Phi heading->target_heading: {0}, crosstrack_dist: {1}, speed: {2}".format(angle, crosstrack_distance, speed))

        new_steering_angle = angle + np.arctan2(k*crosstrack_distance, speed)

        return new_steering_angle
        
        
    def stop_path_tracking(self):
        self.tracking = False
        self.timer.shutdown() 



# Occasionally publishes a true(er) position with some variance to a topic
class Testing(object):
    def __init__(self, true_positioning, update_period=2.0, variance=1.0):
        rospy.loginfo("Create Testing object")
        self.true_pos = true_positioning 
        self.pub = rospy.Publisher("/testing/occasional_odom", Odometry, queue_size=5)
        self.variance = variance
        
        self.timer = rospy.Timer(rospy.Duration(update_period), self.update)

    def update(self, event):
        pose = self.true_pos.get_odom()
        rospy.loginfo("Pose to update tru(er) position: {0}".format(pose))
        if pose is None:
            return
        
        # add some noise
        noise = np.diag(np.random.normal(scale=self.variance, size=6)).ravel()
        cov = np.array(pose.pose.covariance)
        pose.pose.covariance = cov + noise

        self.pub.publish(pose)


if __name__ == "__main__":
    rospy.init_node("line_follow_py")


    # wait until clock messages arrive
    # and hence simulation is ready
    while rospy.get_time() == 0:
        rospy.sleep(1)

    rospy.sleep(2)
    rospy.loginfo("Clock is no longer zero")

    ekf_positioning = EKFPositioning() # republishes odom as pose for display on RViz
    positioning = TruePositioning()
    path = ConstantCurvaturePath(curvature=0.00) # turn radius limit around curvature=0.3
    line_follow = LineFollowController(curve=path, positioning=positioning)
    line_follow.begin(throttle=0.2)

    #Testing(true_positioning=positioning)



    rospy.spin()
