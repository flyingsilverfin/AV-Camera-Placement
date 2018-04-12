
import numpy as np
import rospy
import tf_conversions

import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion


# Accuracy near 0 for normalization
EPSILON = 0.001

def quat_msg_to_numpy(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    numpy_quat = np.array([w, x, y, z])
    return numpy_quat


def quat_mult_quat(quat1, quat2):
    return tf_conversions.transformations.quaternion_multiply(quat1, quat2)

def quat_mult_point(quat, vec):
    vec_quat = np.array([vec[0], vec[1], vec[2], 0])
    q_conj = tf_conversions.transformations.quaternion_conjugate(quat)

    # rotation = q*r*q_conj
    a = quat_mult_quat(quat, vec_quat)
    rotated = quat_mult_quat(a, q_conj)
    return rotated[:3]

def normalize(vec):
    return vec/np.linalg.norm(vec)


def get_as_numpy_position(p):
  
   if type(p) == Point:
       return np.array([p.x, p.y, p.z])
   elif type(p) == type(np.empty(3)):
       return p 
   else:
       rospy.logerr("Can't handle position of type: " + str(type(position)))
       raise Exception("Unknown position type")


def get_as_numpy_quaternion(quat):
    if type(quat) == Quaternion:
        # convert Quaternion geomtry_msg to numpy
        q = quat_msg_to_numpy(quat)
        # convert numpy quaternion into a direction vector
        quat = quat_mult_point(q, np.array([1.0, 0.0, 0.0]))
    elif type(quat) == type(np.empty(3)) and quat.shape == (3,):
        pass
    elif type(quat) == type(np.empty(4)) and quat.shape == (4,):
        quat = quat_mult_point(quat, np.array([1.0, 0.0, 0.0]))
    else:
        rospy.logerr("Quat provided is neither a orientation Quaternion msg nor a numpy 3-tuple representing a direction vector")
        raise Exception("Invalid quat type")

    return quat
