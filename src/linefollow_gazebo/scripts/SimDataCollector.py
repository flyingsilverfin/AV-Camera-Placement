#!/usr/bin/python

import numpy as np
import rospy

from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from custom_messages.msg import CameraUpdate, SimulationDataMsg, PathUpdate
from prius_msgs.msg import Control



class SimDataAggregator(object):
    """
    General idea: save each msg for single-consumption.
    If no relevant msg to consume, don't attach that data
    Bundles all the messages up together for consumption at one time during processing
    """

    def __init__(self):

        self.from_ekf_sub = rospy.Subscriber('/ekf_to_simcollector', SimulationDataMsg, self.receive_from_ekf)

        # self.truth_sub = rospy.Subscriber("/base_pose_ground_truth", Odometry, self.receive_ground_truth)
        self.camera_sub = rospy.Subscriber("/camera_update", CameraUpdate, self.receive_camera_update)
        # self.ekf_sub = rospy.Subscriber('/ekf_odom', Odometry, self.receive_ekf_odom) 
        self.controller_sub = rospy.Subscriber("/prius", Control, self.receive_prius_control_msg)
        self.path_update_sub = rospy.Subscriber('/path_update', PathUpdate, self.receive_path_update)

        self.prius_steer_limits = rospy.get_param("/prius/steer_limit", default=0.872)

      
        # saved states to republish when required
        self.last_true_odom = None
        self.last_ekf = None
        self.last_prius = None
        self.last_camera = None
        self.last_path_msg = None

        # publishing sim data summaries
        self.pub = rospy.Publisher("/simulation_data", SimulationDataMsg, queue_size=25)
        self.seq = 0

    def receive_prius_control_msg(self, control_msg):
        self.last_prius = control_msg

    # def receive_ekf_odom(self, odom_msg):
        # self.last_ekf = odom_msg
        # publish a new data packet here
        # self.publish_sim_data()

    def receive_camera_update(self, camera_update):
        self.last_camera = camera_update

    # def receive_ground_truth(self, odom_msg):
        # self.last_true_odom = odom_msg

    def receive_from_ekf(self, sim_data_msg):
        self.last_true_odom = sim_data_msg.true_odom
        self.last_ekf = sim_data_msg.ekf_odom

        self.publish_sim_data(sim_data_msg) #pass it on retaining stamp to sort by later!

    def receive_path_update(self, path_msg):
        self.last_path_msg = path_msg


    def publish_sim_data(self, msg):

        # msg = SimulationDataMsg()
        header = msg.header
        header.stamp = rospy.get_rostime() # want to retain stamp!
        header.seq = self.seq
        self.seq += 1
        header.frame_id = "map"

        if self.last_true_odom is None or self.last_prius is None or self.last_path_msg is None:
            print("HAVE A NONE! Prius: {0} \n path_msg: {1}".format(self.last_prius, self.last_path_msg))
            return
        
        assert self.last_ekf is not None, "Last EKF is None, should never happen!"

        # transmit true data
        # msg.true_odom = self.last_true_odom
        # self.last_true_odom = None
       
        # transmit last camera update, if there was one
        # this is all now done by EKF!
        # if self.last_camera is not None:
            # msg.camera_update = self.last_camera
            # msg.has_camera_update = True
            # self.last_camera = None
        # else:
            # msg.has_camera_update = False

        # transmit controls
        # I'm going to unpack this a bit for ease of use later
        steer = self.last_prius.steer
        steer_angle = steer * self.prius_steer_limits

        throttle = self.last_prius.throttle
        brake = self.last_prius.brake
        throttle = -1*brake if brake > 0 else throttle
        # processing module can always get true/estimated speed from other components

        msg.prius_steer_angle = steer_angle
        msg.prius_throttle_percent = throttle
        # self.last_prius = None # don't consume prius messages so always retain the last command until changed
    
        # transmit EKF
        # msg.ekf_odom = self.last_ekf
        # self.last_ekf = None
       
        # transmit path data
        msg.path_update = self.last_path_msg
        # self.last_path_msg = None # easiest solutionfor now...

        self.pub.publish(msg)
        print("Passed on sim data msg: {0}".format(msg))




if __name__ == "__main__":
    rospy.init_node("sim_data_collector")
    collector = SimDataAggregator()
    rospy.spin()
