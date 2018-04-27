import abc
import numpy as np
import rospy
from Positioning import TruePositioning

from helper import quat_to_rpy, get_as_numpy_quaternion

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3

class SensorSource(object):
    """ Superclass for sensor sources """
    def __init__(self, topic, msg_type):

        subscriber = rospy.Subscriber(topic, msg_type, self.on_msg)
        self.consumed_last_msg = True

    def on_msg(self, msg):
        self.consumed_last_msg = False
        self.process_msg(msg)

    def has_new_msg(self):
        return self.consumed_last_msg

    @abc.abstractmethod
    def process_msg(self, msg):
        pass

    @abc.abstractmethod
    def get_data(self):
        pass

"""
"
# Decided working out how to use the quaternion covariance in the EKF is too much of a pain...
class ImuSensorSource(SensorSource):

    
    def __init__(self, topic, only_orientation=True):
        if not only_orientation:
            raise Exception("Only support using IMU for orientation information currently")

        super(ImuSensorSource, self).__init__(topic, Imu)

    def process_msg(self, msg);
        
        # only in Z plane => only need one component of the quaternion?

"
"""

class OdometryEKF(object):
    
    def __init__(self, model_step_noise_coeffs=np.array([0.1, 0.1, 0.1, 0.1])):

        self.model_step_noise_coeffs = model_step_noise_coeffs

        self.timer = rospy.Timer(rospy.Duration(self.period), self.step)

        self.true_positioning = TruePositioning()


        self.last_true_position, self.last_true_theta = None, None

        self.sensing_sources = []

    def get_real_delta():
        # get d_translation, rotation 1, rotation 2 from real simulation position

        odom = self.true_positioning.get_odom() # get the latest position... might need to interpolate a bit? #TODO
        position = get_as_numpy_position(pose.position)
        rpy = quat_to_rpy(get_as_numpy_quaternion(pose.orientation)) 
        heading_angle = rpy[2] # yaw = heading
        dist, d_start_angle, d_end_angle = 0.0, 0.0, 0.0 
        if self.last_true_position is not None:
            diff = position - self.last_position
            angle = np.arctan2(diff[1], diff[0])
            dist = np.linalg.norm(diff)
            d_start_angle = angle - self.last_true_theta
            d_end_angle = heading_angle - self.last_true_theta - d_start_angle

        self.last_true_position = position
        self.last_true_theta = heading_angle

        return [dist, d_start_angle, d_end_angle]
        


    def step(self, event):
        dt = event.last_duration

        self.predict(dt)

        for source in self.sensing_sources:
            if source.has_new_data():
                self.update(source.consume_data())


    def predict(self, dt):

        real_deltas = self.get_real_deltas()
        self.state, Q_k = self.step_motion_model(self.state, real_deltas, dt)


    
    def update(self, data):



    def step_motion_model(self, state, real_deltas, dt):
        """ Essentially computes f(state, change)
            Intuitively does the motion model prediction
            In this case using odometry plus some noise
            returns a new state and a matrix Q of added noise variance
            Note that Q is estimated and should NOT be diagonal, just for testing right now!
            
            # bicycle model
            # positional odometry based

            :param real_change: [translation distance, rotation 1, rotation 2]
            """
                
        position = state[:2]
        heading_angle = state[2]
        dist, d_start_angle, d_end_angle = real_deltas
        
        diff, rotation_1, rotation_2 = real_deltas
        alpha_1, alpha_2, alpha_3, alpha_4 = self.model_step_noise_coeffs

        stddev_start_angle_noise = alpha_1 * np.abs(d_start_angle) + alpha_2 * diff
        stddev_end_angle_noise = alpha_1 * np.abs(d_end_angle) + alpha_2 * dist
        stddev_dist_noise = alpha_3 * dist + alpha_4 * np.abs(d_start_angle + d_end_angle)

        error_start_angle = np.random.normal(0.0, stddev_start_angle_noise)
        error_end_angle = np.random.normal(0.0, stddev_end_angle_noise)
        error_dist = np.random.normal(0.0, stddev_dist_noise)

        dist_with_error = dist + error_dist
        d_start_angle_with_error = d_start_angle + error_start_angle
        d_end_angle_with_error = d_end_angle + error_end_angle

        # we are only going to update position and orientation
        # and not touch velocity components
        # note that we only save the angle here and convert it when
        # we want to export it!

        dx = dist_with_error * np.cos(state[2] + d_start_angle_with_error)
        state[0] += dx
        dy = dist_with_error * np.sin(state[2] + d_start_angle_with_error)
        state[1] += dy
        dtheta = d_start_angle_with_error + d_end_angle_with_error
        state[2] += dtheta 

        state[3] = dx/dt
        state[4] = dy/dt
        state[5] = dtheta/dt 

        # form Q process noise matrix
        # this is definitely NOT right
        # should be cross-correlations
        # and also just ignoring cos and sin effects on variance...
        Q = np.diag(np.tile([
            stddev_dist_noise**2,
            stddev_dist_noise**2, 
            stddev_start_angle_noise**2 + stddev_end_angle_noise**2], 2))
        
        return (state, Q)

