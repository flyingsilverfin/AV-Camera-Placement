
import numpy as np
import rospy
import tf_conversions

from geometry_msgs.msg import PointStamped, TransformStamped, Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

EPSILON = 0.001

def quat_msg_to_numpy(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    numpy_quat = np.array([x, y, z, w])
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

def quat_inv_mult(quat1, quat2):
    """  Returns quaternion transforming quat1 -> quat2 ??

    """

    inv = tf_conversions.transformations.quaternion_conjugate(quat1)
    q = tf_conversions.transformations.quaternion_multiply(quat2, inv)
    return q 

def quat_inv(quat):
    return tf_conversions.transformations.quaternion_inverse(quat)


def quat_from_rpy(r, p, y):
    rpy = np.deg2rad([r, p, y])
    return tf_conversions.transformations.quaternion_from_euler(*rpy)

def rpy_to_matrix(r, p, y):
    rpy = np.deg2rad([r, p, y])
    return tf_conversions.transformations.euler_matrix(*rpy)

def inv_transform(matrix):
    return tf_conversions.transformations.inverse_matrix(matrix)

def get_translation_matrix(vector):
    return tf_conversions.transformations.translation_matrix(vector)

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
    #elif type(quat) == type(np.empty(3)) and quat.shape == (3,):
    #    pass
    elif type(quat) == type(np.empty(4)) and quat.shape == (4,):
        pass
    else:
        rospy.logerr("Quat provided is neither a orientation Quaternion msg nor a numpy 3-tuple representing a direction vector")
        raise Exception("Invalid quat type")

    return normalize(quat)

def get_as_numpy_direction_vec(orientation):
    # returns an orientation as a vector along that direction
    if type(orientation) == type(np.empty(3,)) and orientation.shape == (3,):
        return orientation

    elif type(orientation) == Vector3:
        return np.array([orientation.x, orientation.y, orientation.z])
    else:
        q = get_as_numpy_quaternion(orientation)
        return quat_mult_point(q, np.array([1.0, 0, 0]))
        
def get_as_numpy_velocity_vec(vel):
    if type(vel) == np.ndarray:
        return vel

    elif type(vel) == Vector3:
        return np.array([vel.x, vel.y, vel.z])

    else:
        raise TypeError("Uknown velocity type: {}".format(type(vel)))


def angle_from_to(from_, to):
    # we can get the angle without the sign using a.b = |a||b|cos(theta)
    dot = np.dot(from_, to)
    dot /= (np.linalg.norm(from_) * np.linalg.norm(to))

    angle = np.arccos(dot) # require these to be normalized

    # adjust the sign
    rhs = a_rhs_of_b(from_, to)
    # and angle needs to be negated (or any number of other things could be reversed)
    if rhs:
        angle *= -1

    return angle


def a_rhs_of_b(a, b):
    """ Returns True if vector `a` is on the RHS or `b` using cross product """
    normal = np.cross(a, b)
    # if pointing UP, ie. z > 0, then b is to the RHS of a
    return normal[2] > 0

