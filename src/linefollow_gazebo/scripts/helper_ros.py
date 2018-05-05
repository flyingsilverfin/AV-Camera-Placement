"""
All re-used helper functions that depend on ROS/GAZEBO functionality
"""

import rospy
from gazebo_msgs.msg import ModelState

def set_model(model_name, position, orientation_quat, linear_vel, angular_vel):
    msg = ModelState()

