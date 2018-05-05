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

from std_srvs.srv import Empty, SetBool

from PriusControlMsgGenerator import PriusControlMsgGenerator
from Positioning import TruePositioning, EKFPositioning
from ConstantCurvaturePath import ConstantCurvaturePath
from VisualizeMarker import VisualizeMarker
from helper import *
from Path import Path, ForwardPathTracker
from PID import PID

class LineFollowController(object):
    def __init__(self, path, straight_line_velocity=20.0, positioning=None):
        rospy.loginfo("Create LineFollowController object")
        self.rate = rospy.get_param("/LineFollowController/track_path_update_rate", 50.0)   
        self.period = 1.0 / self.rate

        self.positioning = positioning
        self.prius_msg_generator = PriusControlMsgGenerator()
        # topic to publish prius Control message
        self.prius_move = rospy.Publisher("/prius", Control, queue_size=3)

        self.path = path
        self.path_tracker = None    # init to avoid undefined vars before path tracking starts

        self.previous_v_x = 0.0
        self.tracking = False
        self.begin_finish = None

        self.visualize = VisualizeMarker(rate=4)

    def begin(self, throttle):
        self.throttle = throttle 
        # get vehicle moving up to constant speed

        self.repeated_msg = self.prius_msg_generator.forward(self.throttle, 0.0) # get speed up until acceleration stops
        self.prius_move.publish(self.repeated_msg)


        # NOTE perform a service call the EKF node to begin tracking
        self.set_ekf_running = rospy.ServiceProxy('/ekf/set_running', SetBool)
        self.set_ekf_running(True)

        self.timer = rospy.Timer(rospy.Duration(self.period), self.wait_until_at_speed)


    def wait_until_at_speed(self, event, accel_tolerance=0.5):

        pose = self.positioning.get_odom()
        
        if pose is None:
            print("Pose is None")
            rospy.loginfo("Pose received from positioning is None, will accelerate when Pose is received")
            return

        vel_linear = pose.twist.twist.linear
        vel = v_x, v_y, v_z = np.array([vel_linear.x, vel_linear.y, vel_linear.z])
        speed = np.linalg.norm(vel)
        speed_change = speed - self.previous_v_x
        self.previous_speed = speed 

        # if change in velocity is less than tolerance, we've reached desired speed

        # check if have stopped accelerating and velocity has some magnitude
        # that means we've finished accelerating
        if speed_change < accel_tolerance and speed > 0.2:
            self.timer.shutdown()   # stop this update loop
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

        pos = get_as_numpy_position(current_pose.pose.pose.position)
        orientation = get_as_numpy_quaternion(current_pose.pose.pose.orientation)
        orientation_rpy = quat_to_rpy(orientation)

        self.path.set_start(pos, np.rad2deg(orientation_rpy))
        max_horizon = 10.0 if len(self.path.segments) > 1 else 25.0
        self.path_tracker = self.path.get_tracker(max_horizon=max_horizon)
        self.path_tracker.update(pos)

        self.velocity_profile = self.path.get_velocity_profile(lookahead_time=5.0, 
                                                               lookahead_interval=0.25,
                                                               straight_line_speed=13.0, 
                                                               radius_speed_multiplier=0.5) 
        self.velocity_pid = PID()

        self.timer = rospy.Timer(rospy.Duration(self.period), self.track_path_update)

        self.tracking = True


    def track_path_update(self, event):
        if not self.tracking:
            rospy.logerror("Path tracking update called when not tracking, call begin_track_path_first!")
            return

        now = rospy.get_rostime()
        secs = now.to_sec()

        pose = self.positioning.get_odom()
        position = get_as_numpy_position(pose.pose.pose.position)
        heading = vel = get_as_numpy_velocity_vec(pose.twist.twist.linear)
        speed = np.linalg.norm(vel)

        target_speed = self.velocity_profile.get_target_speed(self.path_tracker.get_closest_point_time(), speed)

        print("Position: \n\t {0}\n Velocity: \n\t {1}, Speed: \n\t {2}, Target Speed: {3}".format(position, vel, speed, target_speed))

        self.path_tracker.update(position)

        if self.begin_finish is None and self.path_tracker.finished:
            self.begin_finish = rospy.get_rostime().to_sec()
            self.begin_finish_speed = speed

        steer_angle = self.hoffman_control(position, heading, vel)
        throttle = self.velocity_pid.update(secs, speed - target_speed)
        rospy.loginfo("New steering angle (hoffman): {0}".format(steer_angle))
        rospy.loginfo("New throttle (PID): {0}".format(throttle))
        prius_msg = self.prius_msg_generator.forward(throttle, steer_angle)
        self.prius_move.publish(prius_msg)

        # make sure we elapse the required overshoot distance
        # because the path tracker could indicate 'finished' early!
        if self.begin_finish is not None and \
           rospy.get_rostime().to_sec() - self.begin_finish > self.path_tracker.finish_undershoot/self.begin_finish_speed:
            self.tracking = False
            self.timer.shutdown()
            print("FINISHED tracking")
            self.prius_move.publish(self.prius_msg_generator.forward(-1.0, 0.0))


    def hoffman_control(self, position, heading, vel, steering_angle_limit=0.8727, viz=True): # pull angle limit from URDF
        """ Returns a steering command from -1.0 to 1.0 according to hoffman controller """

        closest_point = self.path_tracker.get_closest_point()
        closest_heading = self.path_tracker.get_closest_tangent()
        rospy.loginfo("Closest target point: {0}, current position: {1}".format(closest_point, position))
        if viz:
            self.visualize.draw_n_points([position, closest_point], duration=60.0)

        diff = closest_point - position
        crosstrack_dist = np.linalg.norm(diff)
        # needs to be a signed crosstrack distance
        if a_rhs_of_b(diff, heading):
            # if target is on LHS of heading negate
            crosstrack_dist *= -1

        new_wheel_angle = self.hoffman_steer_angle(crosstrack_dist, vel, closest_heading, k=1.5)

        # convert angle to command angle
        steer_command = new_wheel_angle/steering_angle_limit
        if steer_command < -1.0:
            return -1.0
        elif steer_command > 1.0:
            return 1.0

        return steer_command

    
    
    def hoffman_steer_angle(self, signed_crosstrack_distance, velocity, target_heading, k=1.0):
        """ Calculates steering angle directly (no feedback) using basic Stanley 2006 line tracking function (Hoffman et al)
        """
        
        heading = normalize(velocity)
        speed = np.linalg.norm(velocity)
        #angle = angle_from_to(heading, target_heading)
        angle = angle_from_to(target_heading, heading)
        rospy.loginfo("Angle Phi heading->target_heading: {0}, signed_crosstrack_dist: {1}, speed: {2}".format(angle, signed_crosstrack_distance, speed))

        new_steering_angle = angle + np.arctan2(k*signed_crosstrack_distance, speed)

        return new_steering_angle
        
        
    def stop_path_tracking(self):
        self.tracking = False
        self.timer.shutdown() 


    def is_finished_tracking(self):
        return not self.tracking and (self.path_tracker is not None and self.path_tracker.finished)



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
    rospy.init_node("line_follow_py", disable_signals=True)


    # wait until clock messages arrive
    # and hence simulation is ready
    while rospy.get_time() == 0:
        rospy.sleep(1)

    rospy.sleep(2)
    rospy.loginfo("Clock is no longer zero")

    # positioning = TruePositioning()
    positioning = EKFPositioning() # just a subscriber to the EKF node that retains the last Odom message for querying

    # path = Path(loop=True)
    # path.add_segment(curvature=0.05, length=0.5*np.pi*2/0.05)
    # path.add_segment(curvature=-0.05, length=0.5*np.pi*2/0.05)
    # path.add_segment(curvature=0.05, length=0.5*np.pi*2/0.05)
    # path.add_segment(curvature=0.0, length=50.0)
    # path.add_segment(curvature=0.05/3, length=0.5*np.pi*2*3/0.05)
    # path.add_segment(curvature=0.0, length=50.0)
   
    path = Path()   #loop=True)
    #path.add_segment(curvature=0.0, length=-1)
    path.add_segment(curvature=0.01, length=2*np.pi/0.01)
# TODO pull path definition from parameter server
# build the path accordingly here


# TODO save path fig to current experiment folder from param server
    path.save_as_fig('/media/data/Uni/Year4/Dissertation/results/path_tracking/path.png')

    line_follow = LineFollowController(path=path, positioning=positioning)

    def begin_tracking(msg):
        """ Execute a path tracking, and quit automatically when finished """
        line_follow.begin(throttle=0.2)

        # TODO wrap this in a param get for RVIZ_visualization
        # ground truth repbulishing for RVIZ
        repubber = TruePositioning(repub=True)

        while not line_follow.is_finished_tracking():
            rospy.rostime.wallsleep(0.5) # this should allow stop_tracking() to be called externally as well
        rospy.signal_shutdown("Done executing path") 
    
    rospy.Service('/VehicleController/begin_path_tracking', Empty, begin_tracking)

    rospy.spin()

