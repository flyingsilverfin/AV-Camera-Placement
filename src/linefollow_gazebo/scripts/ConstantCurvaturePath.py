import numpy as np
import rospy
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
        normal = self.normal_at(t)
        z = np.cross(tangent, normal)

        # construct the change of basis matrix
        # which is equivalent to the rotation matrix from 
        # standard coordinate system (XYZ aligned) to (tangent, normal, z)

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
            rospy.logerr("Unknown return type: " + return_type)
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
            rospy.logerr("Can only return numpy array for tangent")
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
            return point
        # point => homogenous
        point = np.append(point, [1.0])
        transformed = np.matmul(self.transform, point)
        vector = transformed[:-1]/transformed[-1]
        return vector

    def from_world_coord(self, point):
        if self.inv_transform is None:
            return point
        # point => homogenous
        point = np.append(point, [1.0])
        transformed = np.matmul(self.inv_transform, point)
        vector = transformed[:-1]/transformed[-1]
        return vector

    def closest_point_time(self, target_point, min_time=0, max_horizon=None):
        """ Compute the closest point's time in the curve parametrization

        target_point: the point to find the closest point to
        min_time: the time T found must be greater than this
        max_horizon: the time T found cannot be more than this many meters from min_time
        """

        # obtain target point in abstract frame
        point = self.from_world_coord(target_point)

        min_time += self.start_time # make all computations relative to this curve's start time  

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
            a = self.speed / self.r
            dtheta = np.arctan2(y, x)
            if dtheta < 0:
                # arctan2 gives in range [-pi,pi] while I want [0, 2pi] but flipped from the middle
                dtheta = 2*np.pi + dtheta
            
            # need to add pi/2 since that's where my parametrization starts
            dtheta += np.pi/2
            dt = dtheta / a
    
        # now dt contains how far along this curve in time the point is

        # check if the point found is too early along the curve
        if dt + self.start_time < min_time:
            return min_time 

        if max_horizon is None:
            if self.end_time != -1 and dt > self.duration:
                return self.end_time
            else:
                # in valid range
                return self.start_time + dt
        else:
            # check if within max_horizon
            if dt + self.start_time > min_time + max_horizon/self.speed:
                print("Point in local coords: {4}, Min time: {0}, start_time; {1}, dt: {2}, max_horizon/speed: {3}".format(min_time, self.start_time, dt, max_horizon/self.speed, point))
                # ******** This is being hit every time - or not!? ******!!!!
                return min_time + max_horizon/self.speed
            else:
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
        initial_offset = get_translation_matrix(np.array([0.0, self.r, 0.0]))
        self.transformations = [initial_offset]
        self.rotations = []
        self.transform = initial_offset
        self.inv_transform = inv_transform(initial_offset)
        self.rotation = None 

    def set_start_position(self, pos=np.array([0.0, 0.0, 0.0])):
        # if there are any existing translations besides the initial one, cannot set a start translation
        if len(self.transformations) - 1 != len(self.rotations):
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
        if len(self.transformations) == 1 or (len(self.transformations) == 2 and len(self.rotations) == 1):
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
        # check if it's the first rotation, if so go to the other code (avoids a bit of duplication)
        if len(self.transformations) == 1 or (len(self.transformations) == 2 and len(self.rotations) == 0):
            self.set_start_orientation_matrix(rotation_matrix)
            return

        self.transformations.append(rotation_matrix)
        self.rotations.append(rotation_matrix)

        self.transform = np.matmul(rotation_matrix, self.transform)
        self.rotation = np.matmul(rotation_matrix, self.rotation)
        
        self.inv_transform = inv_transform(self.transform)

# class ConstantCurvaturePath(object):
    # """
    # Create an abstract representation of a constant curvature path
    # When begin() is called, it takes a pose and orientation and the path is then translated and rotated to that start point   
    
    # Designed for path tracking
    # Time parametrized requires a speed to work when begin() is called
    # """

    # def __init__(self, curvature, time_parametrized=True):
        # self.init_point = np.array([0.0, 0.0, 0.0])
        # # TODO check this is the right Quaternion for x-axis aligned orientation
        # self.init_orientation = tf_transformations.quaternion_from_euler(0, 0, 0)
        # self.curvature = curvature
        # if self.curvature != 0.0:
            # self.r = np.abs(1/self.curvature)
        # self.time_parametrized = time_parametrized
       
        # self.tracking_start_time = 0.0

        # # to avoid repeatedly computing curve points if ever needed
        # # save them as PointStamped and reuse
        # self.cache = {
            # "curve": None,
            # "tangents": None,
            # "speed": 0,
            # "end_time": -1,
            # "start_time": -1,
            # "type": None
        # }

    # def begin(self, current_time, current_position, current_orientation):
        # """begin

        # :param current_time: current sim time (`time` from rospy.get_rostime()) 
        # :param current_position: current position given as a Point or numpy array (x,y,z)
        # :param current_orientation: current orientation quaternion as Quaternion msg or numpy array (x,y,z,w)
        # """
        
        # rospy.loginfo("starting curve tracking at position: {}".format(current_position))
        # rospy.loginfo("starting curve tracking at orientation: {}".format(current_orientation)) 


        # # translate and rotate path centered at (0,0,0) and x-axis aligned to match current_pose and current_orientation
        # self.path_start_transform = TransformStamped()

        # if type(current_position) == Point:
            # x, y, z = current_position.x, current_position.y, current_position.z
            # pos = np.array([x, y, z])
        # elif type(current_position) == type(np.empty(3)):
            # pos = current_position
        # else:
            # rospy.logerr("Unkown position type: " + str(type(current_position)))
            # return
        # pos_delta = pos - self.init_point

        # self.path_start_transform.transform.translation.x = pos_delta[0] 
        # self.path_start_transform.transform.translation.y = pos_delta[1] 
        # self.path_start_transform.transform.translation.z = pos_delta[2] 
       
        # # store in numpy too for cheaper manipulations
        # # can remove one method later
        # self.path_start_translate_numpy = pos_delta

        # if type(current_orientation) == Quaternion:
            # self.path_start_transform.transform.rotation = current_orientation
            # self.path_start_rotate_numpy_quat = np.array([current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w])
        # elif type(current_orientation) == type(np.empty(4)):
            # self.path_start_transform.transform.rotation = Quaternion(*current_orientation)
            # self.path_start_rotate_numpy_quat = current_orientation
        # else:
            # rospy.logerr("Unknown orientation type: " + str(type(current_orientation)))
            # return
        
      
        # self.tracking_start_time = current_time 
        # rospy.loginfo("Tracking start time: " + str(self.tracking_start_time)) 

    # def get_concrete_curve_points(self, start_time, end_time, speed=1.0, steps=100, return_type='numpy'):
        # curve_points_stamped = self.get_points_transformed(start_time=start_time,
                                                           # end_time=end_time,
                                                           # speed=speed,
                                                           # transform=self.path_start_transform,
                                                           # steps=steps)



        # if return_type == 'numpy':
            # points = np.array([np.array([p.point.x, p.point.y, p.point.z]) for p in curve_points_stamped])
            # return points
        # elif return_type == 'Point':
            # rospy.logerr("Point type concrete path not implemented")
            # raise NotImplementedException("Point type not implemented")
        # elif return_type == 'PointStamped':
            # return curve_points_stamped
        # else:
            # raise UnknownPathTypeException("Unknown return type requested: " + return_type)


    # # TODO test this
    # def update_cached_curve(self, start_time, end_time, speed, steps, curve=None, tangents=None, return_type='PointStamped'):
        # """
        # Save to the cache the concrete path computed with the given parameters
        # if `curve` is not None, just save that into the cache as it's been computed before
        # """

        # if self.cache['curve'] is not None and  \
           # self.cache['tangents'] is not None and \
           # self.cache['start_time'] == start_time and \
           # self.cache['end_time'] == end_time and \
           # self.cache['speed'] == speed and \
           # len(self.cache['curve']) == steps and \
           # self.cache['type'] == return_type:

            # rospy.loginfo("Curve is already cached, not updating cache")
            # return

        # self.cache['start_time'] = start_time
        # self.cache['end_time'] = end_time
        # self.cache['speed'] = speed

        # if curve is None:
            # curve = self.get_concrete_curve_points(start_time, end_time, speed, steps, return_type=return_type)

        # if tangents is None:
            # tangents = self.get_concrete_unit_tangents(start_time, end_time, speed, steps, return_type='numpy')
        # self.cache['curve'] = curve 
        # self.cache['tangents'] = tangents
        # assert self.cache['tangents'].shape[0] == self.cache['curve'].shape[0], "Mismatched curve and tangents!"

    # # for debugging, test transformed paths 
    # def get_points_transformed(self, start_time, end_time, transform, speed=1.0, steps=100):
        # path = self.get_abstract_curve_points(start_time, end_time, speed, steps, return_type='PointStamped')
        # # apply transform to all the points
        # for i in range(len(path)):
            # path[i] = tf2_geometry_msgs.do_transform_point(path[i], transform)
        # return path


    # # as per ROS/Gazebo standards, speed is m/s
    # def get_abstract_curve_points(self, start_time, end_time, speed=1.0, steps=100, return_type='numpy'):
        # """ 
        # Computes a set of `steps` points representing the time-parametrized curve
        # from t=`start_time` to t=`end_time` traversed at a given constant speed
        # TODO variable speeds
        # returns as a numpy array of numpy arrays, Point, or PointStamped
        # """

        # abstract_path =[] 
        # dt = (float(end_time) - start_time) / steps
        # for t in np.arange(start_time, end_time, dt):
            # abstract_path.append(self.get_abstract_point(t, speed, return_type=return_type))
        
        # return abstract_path
        
   
    # # get points on curve relative to abstract location
    # # other methods can apply required transforms such as translation/rotation
    # def get_abstract_point(self, t, speed, return_type='numpy'):
        # dt = t - self.tracking_start_time
        # if self.curvature == 0.0:
            # # straight line along x axis-from start point
            # pt = self.init_point + np.array([dt*speed, 0.0, 0.0])

        # else:
            # # get coefficient for within x(t) = R*cos(a*t), y(t) = R*sin(a*t)
            # # such that the speed is constant as given
            # # works out to a = sqrt(speed)/R
            # # => a = sqrt(speed) * curvature
            # a = np.sqrt(speed) * self.curvature
            # dx = self.r * np.cos(a * dt - np.pi/2) # subtract pi/2 to start at x = 0 and increasing
            # dy = self.r * np.sin(a * dt - np.pi/2) + (self.r) # add radius to offset to (x,y) = (0,0) and both increasing (if positive curvature)
            # dz = 0.0
            # pt = np.array([dx, dy, dz])

        # if return_type == 'numpy':
            # return pt
        # elif return_type == 'Point':
            # return Point(*pt)
        # elif return_type == 'PointStamped':
            # return PointStamped(point=Point(*pt))
        # else:
            # rospy.logerr("Unknown return type: " + return_type)
            # raise UnknownPathTypeException("Unknown requested point return type: " + return_type) 


    # def get_concrete_point(self, t, speed, return_type='numpy'):
        # abstract_point = self.get_abstract_point(t, speed, return_type='numpy')

        # # apply transforms
        # # apply rotation first
        # abstract_point = quat_mult_point(self.path_start_rotate_numpy_quat, abstract_point)
        # # then apply translation
        # concrete_point = abstract_point + self.path_start_translate_numpy

        # if return_type == 'numpy':
            # return concrete_point
        # elif return_type == 'Point':
            # return Point(*concrete_point)
        # elif return_type == 'PointStamped':
            # return PointStamped(point=Point(*concrete_point))
        # else:
            # rospy.logerr("Unknown return type: " + return_type)
            # raise UnknownPathTypeException("Unknown requested concrete point return type: " + return_type) 

    # # gets unit tangent of the curve as numpy vector
    # def get_abstract_unit_tangent(self, t, speed, return_type='numpy'):
        # if self.curvature == 0.0:
            # vec = np.array([1.0, 0.0, 0.0])
        # else:
            # dt = t - self.tracking_start_time
            # a = np.sqrt(speed) * self.curvature
            # # use derivative of equations in `get_abstract_point` to get x'(t) and y'(t), z'(t) = 0 always
            # x_ = self.r * (-1) * a * np.sin(a*dt - np.pi/2)
            # y_ = self.r * a * np.cos(a * dt - np.pi/2)
            # vec = np.array([x_, y_, 0.0])

        # T = vec/np.dot(vec, vec) # normalize to unit vector

        # if return_type == 'numpy':
            # return T
        # else:
            # rospy.logerr("Can only return numpy array for tangent")
            # raise TypeError("Bad type for tangent: " + return_type)

    # def get_concrete_unit_tangents(self, start_time, end_time, speed=1.0, steps=100, return_type='numpy'):
        # """ 
        # Computes a set of `steps` points representing the time-parametrized curve's unit tangents
        # from t=`start_time` to t=`end_time` traversed at a given constant speed
        # returns given type, but depends on abstract_unit_tangent implementation
        # """

        # tangents = []
        # dt = (float(end_time) - start_time) / steps
        # for t in np.arange(start_time, end_time, dt):
            # tangents.append(self.get_concrete_unit_tangent(t, speed, return_type=return_type))

        # if return_type == 'numpy':
            # return np.array(tangents)
        # else:
            # rospy.logerr("Can only return numpy tangents")
            # raise TypeError("Bad type for tangents: " + return_type)
        
    # def get_concrete_unit_tangent(self, t, speed, return_type='numpy'):
        # tangent = self.get_abstract_unit_tangent(t, speed, return_type=return_type)

        # # apply rotation to the abstract tangent
        # rot_quat = self.path_start_rotate_numpy_quat
        # rotated_tangent = quat_mult_point(rot_quat, tangent)
        # return rotated_tangent

    # """
    # "
    # # compute an indicated type of tracking error
    # # current options under consideration are 
    # # time error: Error from position and position it should be at time T
    # # crosstrack error: perpendicular distance to closest point on curve
    # # hamilton (?) error: related to a point on the curve some distance ahead and crosstrack error?
    # def error(self, position, heading, speed, time, error_type='time_error'):
        
        # if error_type== 'time_error':
            # return self.time_error(position, heading, time, speed)

        # elif error_type == 'closest': # aka 'crosstrack' variant
            # # calculate closest point on entirety of path
            # # BAD for two reasons:
            # # 1) may be so far off path we've approached another point and then start tracking in the opposite direction
            # # 2) direction agnostic (eg just following closest points while moving)
            # # 3) computationally expensive (negated with caching)
            # return self.closest_error(self, position, heading)

        # elif error_type == 'windowed':
            # return self.windowed_error(position, heading, time=time, speed=speed)
        # else:
            # raise UnknownErrorTypeException("Unkown error metric: " + error_type)
    # "
    # """

    # def _calculate_error(self, position, current_steer_angle, velocity, target, target_direction):
        
        # position = get_as_numpy_position(position)
        # heading = normalize(get_as_numpy_direction_vec(heading))
        # diff = target - position         
        # crosstrack_distance = np.linalg.norm(diff)


        # # need forward velocity in direction of vehicle orientation...

        # speed = np.linalg.norm(get_as_numpy_velocity_vec(velocity))

        # target_direction = normalize(target_direction)
        
       
        # angle_heading_target = angle_from_to(heading, target_direction)
        # angle_steering_target = angle_from_to(target_direction, current_steer_angle)
        

        # weighting = 0.2 
        # error = distance * (weighting * angle_to_target_position + (1-weighting)*angle_heading_target)/np.pi

        # #error = distance * angle / np.pi + sign*0.1*distance 
# #        error = distance * angle/np.pi + sign*0.1*distance

        # print("Heading: {3}, Diff: {4}, Angle to target pos: {0}, angle diff target heading: {1}, distance: {2}".format(angle_to_target_position, angle_heading_target, distance, heading, diff))

        # return error 

    # def time_error(self, position, heading, time, speed):
        # target_point = self.get_concrete_point(time, speed, return_type='numpy')

        # pos = get_as_numpy_position(position)
 

        # return self._calculate_error(pos, heading, target_point)


    # def closest_point_tangent(self, position, last_closest_point, distance_resolution=0.1, max_horizon=None):
        # # set a default max horizon equal to one radius or 5m whichever is less 
        # if max_horizon is None:
            # if self.curvature == 0:
                # max_horizon = 5.0
            # else:
                # max_horizon = min(5.0, self.r)
        
        # pos = get_as_numpy_position(position)
      
        # if self.cache['curve'] is None or self.cache['tangents'] is None:
            # #note : speed is 1.0 for these => max_horizon/1.0 = time into the future for that horizon 
            # self.update_cached_curve(self.tracking_start_time, self.tracking_start_time + max_horizon/1.0, 1.0, max_horizon/distance_resolution, return_type='numpy')

        # # the cache must contain last_closest_point
        # curve = self.cache['curve']
        # index = 0  
        # point = curve[index] # default to first point in list
        # for i, pt in enumerate(curve):
            # if np.array_equal(pt, last_closest_point):
                # index = i
                # point = pt
                # break
      
        # cached_start_time = self.cache['start_time']
        # cached_end_time = self.cache['end_time']
        # dt = cached_end_time - cached_start_time
        # steps = len(curve) 
        # index_time = float(index)/steps * dt + cached_start_time 

        # start_time = index_time 
        # speed = 1.0
        # steps = max_horizon/distance_resolution
        # end_time = start_time + max_horizon/speed

        # rospy.loginfo_throttle(0.25, "Point: {0}, index: {1}, index time = start time: {2}, end_time using horizon: {3}, cached_end_time: {4}".format(point, index, index_time, end_time, cached_end_time))

        # # check if the end time we want to search to is in the cached computed points
        # if end_time > cached_end_time:
            # # TODO update cached curve to include points in the horizon too
            # save_ahead_cache_mult = 4 
            # self.update_cached_curve(index_time, 
                                     # index_time + save_ahead_cache_mult * speed * max_horizon, 
                                     # speed,
                                     # steps = save_ahead_cache_mult * max_horizon / distance_resolution,
                                     # return_type='numpy')

            # # update curve, index, and point to reflect cache 
            # curve = self.cache['curve']
            # index = 0
            # assert np.linalg.norm(curve[index] - point) < distance_resolution, "Updated curve has point different from previous closest point: curve[0] {0} vs previous closest: {1}".format(curve[index], point)
            # point = curve[index]
       
        # # cut off cache so we can only ever move forward on the curve and not lock onto a remote point
        # curve = self.cache['curve'] = self.cache['curve'][index:]
        # tangents = self.cache['tangents'] = self.cache['tangents'][index:]
        # self.cache['start_time'] = index_time

        # closest_index, closest_dist, closest_point = index, np.linalg.norm(point - pos), point 

        # horizon_steps = int(max_horizon/distance_resolution)
        # for i, pt in enumerate(curve[0 : horizon_steps]):
            # d = np.linalg.norm(pt - pos)
            # # allow some slack to pick a forward point if it's close enough so we squash oscillations
            # if d  < closest_dist: #- distance_resolution/2 < closest_dist:
                # closest_dist = d
                # closest_point = pt
                # closest_index = i
       
        # closest_point_tangent = tangents[closest_index]

        # return closest_point, closest_point_tangent 

    # """
    # "
    # def closest_concrete_point_in_window(self, position, speed, time, time_window=10.0, distance_resolution=1.0):
       
        # start_time = time - time_window
        # end_time = time + time_window

        # # calculate number of steps to compute to achieve the desired resolution
        # total_dist = speed * 2*time_window 
        # steps = total_dist / distance_resolution
        
        # pts = self.get_concrete_curve_points(start_time, end_time, speed=speed, steps=steps, return_type='numpy')

        # if type(position) == type(numpy.empty(3)):
            # pos = position
        # elif type(position) == Point:
            # pos = np.array([position.x, position.y, position.z])
        # else:
            # rospy.logerr("Unknown position type: {}".format(type(position)))
            # return

        # distances = np.linalg.norm(pts - pos)
        # index = np.argmin(distances)
        # closest_point = pts[index]

        # # return both the point and the time on the path of that point
        
        # return closest_point, start_time + 2*time_window * (index/float(steps))
    # "
    # """            

    # #def next_closest_point(self, position, last_time, distance_resolution=1.0):



class UnknownErrorTypeException(Exception):
    pass
class UnknownPathTypeException(Exception):
    pass
class NotImplementedException(Exception):
    def __init__(self, msg):
        super(NotImplementedException, self).__init__("Not Implemented!" + msg)
