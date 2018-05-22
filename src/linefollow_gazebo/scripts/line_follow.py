#!/usr/bin/env python

import numpy as np
import rospy
import os

from sensor_msgs.msg import Imu
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry

import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from std_msgs.msg import Float64, Bool
from std_srvs.srv import Empty, SetBool, SetBoolResponse
from custom_messages.msg import PathUpdate

from PriusControlMsgGenerator import PriusControlMsgGenerator
from Positioning import TruePositioning, EKFPositioning
from ConstantCurvaturePath import ConstantCurvaturePath
from VisualizeMarker import VisualizeMarker
from helper import *
from Path import Path, ForwardPathTracker
from PID import PID

class LineFollowController(object):
    def __init__(self, path, straight_line_velocity=20.0, rviz=True):
        rospy.loginfo("Create LineFollowController object")
        self.rate = rospy.get_param("/controller/update_rate", 50.0)   
        self.period = 1.0 / self.rate

        # self.positioning = positioning
        self.prius_msg_generator = PriusControlMsgGenerator()
        # topic to publish prius Control message
        self.prius_move = rospy.Publisher("/prius", Control, queue_size=25)

        self.path = path
        self.path_tracker = None    # init to avoid undefined vars before path tracking starts


        self.last_steer_angle = None

        self.previous_v_x = 0.0
        self.tracking = False
        self.begin_finish = None

        if rviz:
            self.visualize = VisualizeMarker(rate=4)
        self.rviz = rviz

        self.k = rospy.get_param('/controller/hoffman_steering/distance_multiplier')


    def begin(self, throttle):
        self.throttle = throttle 
        # get vehicle moving up to constant speed

        self.repeated_msg = self.prius_msg_generator.forward(self.throttle, 0.0) # get speed up until acceleration stops
        self.prius_move.publish(self.repeated_msg)


        # NOTE perform a service call the EKF node to begin tracking
        self.set_ekf_running = rospy.ServiceProxy('/ekf/set_running', SetBool)
        self.set_ekf_running(True)

        # self.timer = rospy.Timer(rospy.Duration.from_sec(self.period), self.wait_until_at_speed)
        # run off subscriber directly
        self.subber = rospy.Subscriber('/ekf_odom', Odometry, self.step_wait)
        self.dont_wait = False
        self.last_update = rospy.get_rostime().to_sec()

    def step_wait(self, true_odom):
        self.new_true_odom = true_odom
        if self.dont_wait:
            return
        self.wait_until_at_speed()


    def wait_until_at_speed(self, accel_tolerance=0.5):
        t = rospy.get_rostime().to_sec()
        if t - self.last_update < self.period:
            return
        self.last_update = t

        # pose = self.positioning.get_odom()
        pose = self.new_true_odom
        
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
        if speed > 0.2:
            # self.timer.shutdown()   # stop this update loop
            self.subber.unregister()
            self.dont_wait = True
            self.begin_track_path() # begin path tracking!
            return

        self.prius_msg_generator.update_msg(self.repeated_msg) #update seq number
        # rospy.loginfo("Sending Prius Control msg: {}".format(self.repeated_msg))
        self.prius_move.publish(self.repeated_msg)

    def begin_track_path(self):

        # don't attempt to re-initalize tracking if it's running
        # messes with pubs/subs
        if self.tracking:
            rospy.logerr("Already tracking path!")
            raise Exception()

        now = rospy.get_rostime()
        self.last_update = now.to_sec()
        # current_pose = self.positioning.get_odom()
        current_pose = self.new_true_odom
 
        rospy.loginfo("Starting path tracking at: " + str(current_pose))


        # set up path, and get a path tracker
        pos = get_as_numpy_position(current_pose.pose.pose.position)
        orientation = get_as_numpy_quaternion(current_pose.pose.pose.orientation)
        orientation_rpy = quat_to_rpy(orientation)

        # self.path.set_start(pos, np.rad2deg(orientation_rpy)) # see if we can get away this right now, so don't have to move cameras too
        max_horizon = 10.0 if len(self.path.segments) > 1 else 25.0
        max_horizon = rospy.get_param('/controller/path_tracking/max_horizon', default=10.0)
        max_horizon = max_horizon if len(self.path.segments) > 1 else max(25.0, 2*max_horizon)
        self.path_tracker = self.path.get_tracker(max_horizon=max_horizon)
        self.path_tracker.update(pos)


        # configure and start a velocity controller
        vel_controller_params = rospy.get_param('/controller/velocity_controller')
        
        lookahead_time = vel_controller_params['lookahead_time']
        lookahead_interval = vel_controller_params['lookahead_interval']
        straight_speed = vel_controller_params['straight_speed']
        radius_mult = vel_controller_params['radius_speed_mult']

        self.velocity_profile = self.path.get_velocity_profile(lookahead_time=lookahead_time, 
                                                               lookahead_interval=lookahead_interval,
                                                               straight_line_speed=straight_speed, 
                                                               radius_speed_multiplier=radius_mult) 

        p,i,d = vel_controller_params['pid']
        self.velocity_pid = PID(kp=p, ki=i, kd=d)

        # publisher for path/tracking info to log
        self.path_update_pub = rospy.Publisher('/path_update', PathUpdate, queue_size=2)

        # self.timer = rospy.Timer(rospy.Duration.from_sec(self.period), self.track_path_update)
        self.subber = rospy.Subscriber('/ekf_odom', Odometry, self.step_path)
        self.tracking = True

    def step_path(self, true_odom):
        self.new_true_odom = true_odom
        self.track_path_update()

    def track_path_update(self):
        if not self.tracking:
            rospy.logerr("Path tracking update called when not tracking, call begin_track_path_first!")
            return

        now = rospy.get_rostime()
        secs = now.to_sec()
        
        if secs - self.last_update < self.period:
            return
        self.last_update = secs

        pose = self.new_true_odom
        position = get_as_numpy_position(pose.pose.pose.position)
        heading = vel = get_as_numpy_velocity_vec(pose.twist.twist.linear)
        speed = np.linalg.norm(vel)
        speed = max(0.01, speed) # 0 speed => errors!


        # print("Position: \n\t {0}\n Velocity: \n\t {1}, Speed: \n\t {2}, Target Speed: {3}".format(position, vel, speed, target_speed))

        self.path_tracker.update(position)

        if self.begin_finish is None and self.path_tracker.finished:
            self.begin_finish = rospy.get_rostime().to_sec()
            self.begin_finish_speed = speed


        target_speed = self.velocity_profile.get_target_speed(self.path_tracker.get_closest_point_time(), speed)

        steer_angle = self.hoffman_control(position, heading, vel)
        self.last_steer_angle = steer_angle
        throttle = self.velocity_pid.update(secs, speed - target_speed)
        # rospy.loginfo("New steering angle (hoffman): {0}".format(steer_angle))
        # rospy.loginfo("New throttle (PID): {0}".format(throttle))
        prius_msg = self.prius_msg_generator.forward(throttle, steer_angle)
        self.prius_move.publish(prius_msg)


        # send path update 
        msg = PathUpdate()
        target_pt = msg.target_point
        target_pt.x, target_pt.y, target_pt.z = self.path_tracker.get_closest_point()
        target_heading = msg.target_heading
        target_heading.x, target_heading.y, target_heading.z = self.path_tracker.get_closest_tangent()
        target_normal = msg.path_normal
        target_normal.x, target_normal.y, target_normal.z = self.path_tracker.get_closest_normal()
        msg.velocity_controller_target = target_speed 
        msg.path_curvature = self.path_tracker.active_segment.curvature 
        msg.current_ekf_position = pose.pose.pose.position 
        msg.current_time = secs
        self.path_update_pub.publish(msg)

        # make sure we elapse the required overshoot distance
        # because the path tracker could indicate 'finished' early!
        if self.begin_finish is not None and \
           rospy.get_rostime().to_sec() - self.begin_finish > self.path_tracker.finish_undershoot/self.begin_finish_speed:
            self.tracking = False
            # self.timer.shutdown()
            self.subber.unregister()
            print("FINISHED tracking")
            self.prius_move.publish(self.prius_msg_generator.forward(-1.0, 0.0))
            # shutdown this node => kills all others
            rospy.signal_shutdown("Done executing path") 


    def hoffman_control(self, position, heading, vel, steering_angle_limit=0.8727): # pull angle limit from URDF
        """ Returns a steering command from -1.0 to 1.0 according to hoffman controller """

        closest_point = self.path_tracker.get_closest_point()
        closest_heading = self.path_tracker.get_closest_tangent()
        # rospy.loginfo("Closest target point: {0}, current position: {1}".format(closest_point, position))
        if self.rviz:
            self.visualize.draw_n_points([position, closest_point], duration=60.0)

        diff = closest_point - position
        crosstrack_dist = np.linalg.norm(diff)
        # needs to be a signed crosstrack distance
        if a_rhs_of_b(diff, heading):
            # if target is on LHS of heading negate
            crosstrack_dist *= -1

        new_wheel_angle = self.hoffman_steer_angle(crosstrack_dist, vel, closest_heading)

        # convert angle to command angle
        steer_command = new_wheel_angle/steering_angle_limit
        if steer_command < -1.0:
            return -1.0
        elif steer_command > 1.0:
            return 1.0
        
        if np.isnan(steer_command):
            return self.last_steer_angle

        return steer_command

    
    
    def hoffman_steer_angle(self, signed_crosstrack_distance, velocity, target_heading):
        """ Calculates steering angle directly (no feedback) using basic Stanley 2006 line tracking function (Hoffman et al)
        """
        
        speed = np.linalg.norm(velocity)
        if speed == 0.0:
            heading = np.array([1.0, 0, 0]) # should be a very rare case but just assume on X axis if not moving (likely not started yet)
        else:
            heading = normalize(velocity)
        #angle = angle_from_to(heading, target_heading)
        angle = angle_from_to(target_heading, heading)
        # rospy.loginfo("Angle Phi heading->target_heading: {0}, signed_crosstrack_dist: {1}, speed: {2}".format(angle, signed_crosstrack_distance, speed))

        new_steering_angle = angle + np.arctan2(self.k*signed_crosstrack_distance, speed)

        return new_steering_angle
        
        
    def stop_path_tracking(self):
        self.tracking = False
        # self.timer.shutdown() 
        self.subber.unregister()


    def is_finished_tracking(self):
        return not self.tracking and (self.path_tracker is not None and self.path_tracker.finished)



# Occasionally publishes a true(er) position with some variance to a topic
class Testing(object):
    def __init__(self, true_positioning, update_period=2.0, variance=1.0):
        rospy.loginfo("Create Testing object")
        self.true_pos = true_positioning 
        self.pub = rospy.Publisher("/testing/occasional_odom", Odometry, queue_size=25)
        self.variance = variance
        
        self.timer = rospy.Timer(rospy.Duration.from_sec(update_period), self.update)

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

    looping = rospy.get_param('/road/path/loop', default=False)
    segments = rospy.get_param('/road/path/segments')

    path = Path(loop=looping)
    for segment in segments:
        path.add_segment(curvature=segment["curvature"], length=segment["length"])

    
    save_dir = rospy.get_param('/results_dir')
    path.save_as_fig(save_dir + '/path.png')

    line_follow = LineFollowController(path=path)#, positioning=positioning)

    rviz = rospy.get_param('/rviz')

    def begin_tracking(msg):
        """ Execute a path tracking, and quit automatically when finished """
        print("Received begin_path_tracking service call!")
        line_follow.begin(throttle=1.0)

        # TODO wrap this in a param get for RVIZ_visualization
        # ground truth repbulishing for RVIZ
        if rviz:
            repubber = TruePositioning(repub=True)
            print("True position at start line tracking: {0}".format(repubber.get_odom()))
        
        return SetBoolResponse(True, "Started Tracking")

   
    print("Ready for begin_path_tracking service call")
    # wait for external signal to start tracking
    rospy.Service('/VehicleController/begin_path_tracking', SetBool, begin_tracking)

    rospy.spin()

