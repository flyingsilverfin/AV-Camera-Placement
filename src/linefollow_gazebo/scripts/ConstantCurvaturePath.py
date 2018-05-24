import numpy as np
import tf_conversions

tf_transformations = tf_conversions.transformations

import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped, TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from helper import *


class ConstantCurvaturePath(object):
    
    def __init__(self, curvature, length=-1):
        
        # Init order matters!
        self.curvature = curvature
        if np.abs(curvature) > 0.0:
            self.r = 1.0/self.curvature


        # set up some defaults
        self.reset_transforms()
        self.set_start_time()

        self.length = length 
        self.speed = 1.0 # default to 1.0 m/s for parametrization 
        self.duration = -1 if self.length == -1 else self.length/self.speed
        self.end_time = -1 if self.length == -1 else self.start_time + self.duration 


    def point_at(self, t, speed=None, return_type='numpy'):
        if speed is None:
            speed = self.speed

        point = self.abstract_point_at(t, speed, return_type='numpy')
        point = self.to_world_coord(point) 

        if return_type != 'numpy':
            print("Non-Numpy points currently not supported")
            raise Exception()
        
        return point


    def tangent_at(self, t, speed=None, return_type='numpy'):
        if speed is None:
            speed = self.speed

        tangent = self.abstract_tangent_at(t, speed, return_type='numpy')
        tangent = self.to_world_rotation(tangent)

        if return_type != 'numpy':
            raise Exception("Non-Numpy tangents currently not supported")

        return tangent

    def tangent_rotation_matrix_at(self, t, return_type='numpy'):
        tangent = self.tangent_at(t)

        z = np.array([0.0, 0.0, 1.0]) # always want Z UP otherwise get rotations about Z axis
        normal = normalize(np.cross(z, tangent))

        # normal = self.normal_at(t)
        # z = np.cross(tangent, normal)

        # construct the change of basis matrix
        # which is equivalent to the rotation matrix from 
        # standard coordinate system (XYZ aligned) to (tangent, normal, z)
        # note this can flip axes... so not just rotations

        basis = np.array([tangent, normal, z]).T 
        # extend for homogenous coords
        rotation = np.vstack([basis, [0, 0, 0]])
        rotation = np.hstack([rotation, np.array([0, 0, 0, 1]).reshape(-1, 1)])

        return rotation

    def normal_at(self, t, speed=None, return_type='numpy'):
        if speed is None:
            speed = self.speed
        normal = self.abstract_normal_at(t, speed, return_type='numpy')
        normal = self.to_world_rotation(normal)

        if return_type != 'numpy':
            raise Exception("Non-Numpy normals not currently supproted")

        return normal


    def abstract_point_at(self, t, speed=None, return_type='numpy'):

        if speed is None:
            speed = self.speed
        
        dt = t - self.start_time

        distance = dt * self.speed
        if self.length != -1 and distance > self.length:
            print("WARN: request time is behind length of curve - {0}".format(t))


        if self.curvature == 0.0:
            # straight line along x axis-from start point
            pt = np.array([dt*speed, 0.0, 0.0])

        else:
            # get coefficient for within x(t) = R*cos(a*t), y(t) = R*sin(a*t)
            # such that the speed is constant as given
            # works out to a = speed/R
            # => a = sqrt(speed) * curvature
            a = speed * self.curvature
            dx = self.r * np.cos(a * dt - np.pi/2) # subtract pi/2 to start at x = 0 and increasing
            dy = self.r * np.sin(a * dt - np.pi/2) 
            dz = 0.0
            pt = np.array([dx, dy, dz])

        if return_type == 'numpy':
            return pt
        elif return_type == 'Point':
            return Point(*pt)
        elif return_type == 'PointStamped':
            return PointStamped(point=Point(*pt))
        else:
            raise UnknownPathTypeException("Unknown requested point return type: " + return_type) 


    def abstract_tangent_at(self, t, speed=None, return_type='numpy'):
        if speed is None:
            speed = self.speed

        if self.curvature == 0.0:
            vec = np.array([1.0, 0.0, 0.0])
        else:
            dt = t - self.start_time
            a = speed * self.curvature
            # use derivative of equations in `get_abstract_point` to get x'(t) and y'(t), z'(t) = 0 always
            x_ = self.r * (-1) * a * np.sin(a*dt - np.pi/2)
            y_ = self.r * a * np.cos(a * dt - np.pi/2)
            vec = np.array([x_, y_, 0.0])

        T = normalize(vec) # normalize to unit vector

        if return_type == 'numpy':
            return T
        else:
            raise TypeError("Bad type for tangent: " + return_type)

    def abstract_normal_at(self, t, speed=None, return_type='numpy'):
        if speed is None:
            speed = self.speed

        if self.curvature == 0.0:
            return np.array([0.0, 1.0, 0.0]) # RH rule such that Z points up

        else:
            tangent = self.abstract_tangent_at(t, speed)

            # compute unit normal using cross product
            # pick a point +1.0 Z or -1.0 Z depending on curvature
            # define positive curvature to curve left of tangent
            # therefore needs -1.0Z if positive curvature
            # then cross tangent with that point

            offset = np.array([0.0, 0.0, 1.0])
            if self.curvature > 0:
                offset *= -1

            normal = normalize(np.cross(tangent, offset))
            return normal



    def to_world_rotation(self, point):
        if self.rotation is None:
            return point
        # convert point to homogenous coords
        point = np.append(point, [1.0])
        rotated = np.matmul(self.rotation, point)
        vector = rotated[:-1]/rotated[-1]   # convert to normal coordinates again
        return vector
       

    def to_world_coord(self, point):
        if self.transform is None:
            print("WARN: no curve => world transform!")
            return point
        # point => homogenous
        point = np.append(point, [1.0])
        transformed = np.matmul(self.transform, point)
        vector = transformed[:-1]/transformed[-1]
        return vector

    def from_world_coord(self, point):
        if self.inv_transform is None:
            print("WARN: no world => curve transform!")
            return point
        # point => homogenous
        point = np.append(point, [1.0])
        transformed = np.matmul(self.inv_transform, point)
        vector = transformed[:-1]/transformed[-1]
        return vector

    def closest_point_time(self, target_point, min_time=None, max_horizon=None):
        """ Compute the closest point's time in the curve parametrization

        target_point: the point to find the closest point to
        min_time: the time T found must be greater than this
        max_horizon: the time T found cannot be more than this many meters from min_time

        #TODO this can be massively cleaned up and unified
        probably written more elegantly considering mathematical underpinnings as well
        """

        if min_time < self.start_time:
            return self.start_time

        # obtain target point in abstract frame
        point = self.from_world_coord(target_point)
        
        if min_time is None:
            min_time = self.start_time # make all computations relative to this curve's start time  

        if self.curvature == 0.0:

            # direction vector = (1,0,0) in local coords always
            direction = np.array([1.0, 0, 0])
            start_point = np.array([0.0, 0, 0])

            # project point onto direction
            distance_in_direction = np.dot(point, direction)
            
            # compensate for traversal speed (not guaranteed to be 1.0)
            dt = distance_in_direction / self.speed
            

        else:
            # this can be solved analytically
            # for a curve, the time at speed 1.0
            # is just the angle drawn out by the ray going to the point from the origin
            # here also compensate for radius and speed
            x,y,z = point
            a = np.abs(self.speed / self.r)
            # dtheta = np.arctan2(y, x)
            # dtheta += np.sign(self.curvature)*np.pi/2 
            # dtheta *= np.sign(self.curvature) # account for flipped direction for negative curvature

            dtheta = np.sign(self.curvature)*(np.arctan2(y,x)) + np.pi/2 # same as above but in 1 line
            if dtheta < 0:
                # arctan2 gives in range [-pi,pi] while I want [0, 2pi] but flipped from the middle
                dtheta = 2*np.pi + dtheta
            
            dt = dtheta / a
    
        # now dt contains how far along this curve in time the point is

        # check if the point found is too early along the curve
        if dt + self.start_time < min_time:
            # print("Using min_time, dt: {0}, dt+start_time: {1}".format(dt, dt+self.start_time))
            # print("\tcurvature: {0}, target point: {1}, target_point in local coords: {2}".format(self.curvature, target_point, point))
            return min_time 

        if max_horizon is None:
            if self.end_time != -1 and dt > self.duration:
                return self.end_time
            else:
                # in valid range
                return self.start_time + dt
        else:
            # if beyond horizon return min time to solve jumping segments
            if self.duration != -1 and dt > self.duration:
                # print("Using end_time since dt > duration, dt: {0}, dt+start_time: {1}, min-time: {2}".format(dt, dt+self.start_time, min_time))
                # print("\tcurvature: {0}, target point: {1}, target_point in local coords: {2}".format(self.curvature, target_point, point))
                return min_time
            # check if within max_horizon
            if dt + self.start_time > min_time + max_horizon/self.speed:
                # print("Using min_time since dt+start_time > horizon, dt: {0}, dt+start_time: {1}, min_time: {2}".format(dt, dt+self.start_time, min_time))
                # print("\tcurvature: {0}, target point: {1}, target_point in local coords: {2}".format(self.curvature, target_point, point))
                # NOTE if we are beyond our limit always pick the starting point in the valid range
                # this solves the loop restart problem
                return min_time 
            else:
                # print("Using dt + start_time, dt: {0}, dt+start_time: {1}, min_time: {2}".format(dt, dt+self.start_time, min_time))
                # print("\tcurvature: {0}, target point: {1}, target_point in local coords: {2}".format(self.curvature, target_point, point))
                # in valid range
                return self.start_time + dt


    def get_length(self):
        return self.length

    def get_duration(self):
        return self.duration

    def set_length(self, length):
        self.length = float(length)
        self.duration = self.length/self.speed
        self.end_time = self.start_time + self.duration

    def set_start_time(self, time=0.0):
        self.start_time = float(time)
        if hasattr(self, 'end_time') and self.end_time != -1:
            self.end_time = self.start_time + self.duration

    def reset_transforms(self):
        # list to store sequence of transformations, rotations or translations in homogenous coordinates
        # order matters - stored in reverse order for easy append
        # reversed on concat
        self.transformations = []   
        self.rotations = []     # used for rotating tangents and normals that don't care about translations
        # store composed transform
        if self.curvature != 0.0:
            initial_offset = get_translation_matrix(np.array([0.0, self.r, 0.0]))
            self.transformations = [initial_offset]
            self.transform = initial_offset
            self.inv_transform = inv_transform(initial_offset)
        else:
            self.transform, self.inv_transform, self.transformations = None, None, []
        self.rotations = []
        self.rotation = None 

    def set_start_position(self, pos=np.array([0.0, 0.0, 0.0])):
        # if there are any existing translations besides the initial one, cannot set a start translation
        if self.curvature != 0.0 and len(self.transformations) - 1 != len(self.rotations) \
           or self.curvature == 0.0 and len(self.transformations) != len(self.rotations):
            raise Exception("Cannot set start position if any translations already exist")

        pos = get_as_numpy_position(pos)

        translation_matrix = get_translation_matrix(pos)
        self.transformations.append(translation_matrix)
        
        if self.transform is None:
            self.transform = translation_matrix
        else:
            # add this transform by left multiplying with existing transform
            self.transform = np.matmul(translation_matrix, self.transform)
        self.inv_transform = inv_transform(self.transform)

    def set_start_orientation_rpy(self, orientation_rpy_deg=np.array([0, 0, 0.0])):
        rotation_matrix = rpy_to_matrix(*orientation_rpy_deg)
        self.set_start_orientation_matrix(rotation_matrix)


    def set_start_orientation_matrix(self, rotation_matrix):
        # if there are existing rotations, cannot set start rotation
        if len(self.rotations) != 0:
            raise Exception("Cannot set start orientation if any rotations exist")
        
        self.transformations.append(rotation_matrix)
        self.rotations.append(rotation_matrix)

        self.transform = rotation_matrix if self.transform is None else np.matmul(rotation_matrix, self.transform)
        self.rotation = rotation_matrix if self.rotation is None else np.matmul(rotation_matrix, self.rotation)
        self.inv_transform = inv_transform(self.transform)
    
    def add_translation(self, vector):
        # if it's a curve and there is only 1 transformation (mandatory translation) or that translation and a rotation
        # or its a straight line and there are no transforms/1 transform and its a rotation
        # then we're just setting the initial start position
        if self.curvature != 0.0 and (len(self.transformations) == 1 or (len(self.transformations) == 2 and len(self.rotations) == 1)) \
           or self.curvature == 0.0 and (len(self.transformations) == 0 or (len(self.transformations) == 1 and len(self.rotations) == 1)):
            self.set_start_position(vector)
            return

        vector = get_as_numpy_position(vector)
        translation_matrix = get_translation_matrix(vector)
        self.transformations.append(translation_matrix)
        self.transform = np.matmul(translation_matrix, self.transform)
        self.inv_transform = inv_transform(self.transform)


    def add_rotation_rpy(self, rotation_rpy_deg):
        rotation_matrix = rpy_to_matrix(*rotation_rpy_deg)
        self.add_rotation_matrix(rotation_matrix)

    def add_rotation_matrix(self, rotation_matrix):
        # if its the first rotation, just do a set rather than an add
        if len(self.rotations) == 0:
            self.set_start_orientation_matrix(rotation_matrix)
            return

        self.transformations.append(rotation_matrix)
        self.rotations.append(rotation_matrix)

        self.transform = np.matmul(rotation_matrix, self.transform)
        self.rotation = np.matmul(rotation_matrix, self.rotation)
        
        self.inv_transform = inv_transform(self.transform)



class UnknownErrorTypeException(Exception):
    pass
class UnknownPathTypeException(Exception):
    pass
class NotImplementedException(Exception):
    def __init__(self, msg):
        super(NotImplementedException, self).__init__("Not Implemented!" + msg)
