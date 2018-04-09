
import numpy as np
import rospy
import tf_conversions

import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


def quat_msg_to_numpy(quat):
    x, y, z, w = quat.x, quaty, quat.z, quat.w
    numpy_quat = np.array([w, x, y, z])
    return numpy_quat


def quat_mult_quat(quat1, quat2):
    return tf_conversions.transformations.quaternion_multiply(quat1, quat2)

def quat_mult_point(quat, vec):
    vec_quat = np.array([0, vec[0], vec[1], vec[2]])
    q_conj = tf_conversions.transformations.quaternion_conjugate(quat)

    # rotation = q*r*q_conj
    a = quat_mult_quat(quat, vec_quat)
    rotated = quat_mult_quat(a, q_conj)
    return rotated[1:]

def normalize(vec):
	return vec/np.dot(vec, vec)
