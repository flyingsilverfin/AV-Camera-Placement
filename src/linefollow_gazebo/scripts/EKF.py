#!/usr/bin/python

import abc
import numpy as np
from numpy import matmul
import rospy

from Positioning import TruePositioning
from helper import quat_to_rpy, get_as_numpy_quaternion, get_as_numpy_position, quat_from_rpy_rad

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovarianceStamped


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

        self.rate = rospy.get_param("~ekf_rate", 25.0)   
        self.period = 1.0/self.rate

        self.true_positioning = TruePositioning()
        self.last_true_position, self.last_true_theta = None, None

        # state: x, y, theta, x', y', theta'
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.cov = np.zeros((6,6))

        # we can attach update sources here, conforming to SensorSource interface 
        self.sensing_sources = []

        # reusable matrices, save initization
        self.I = np.diag(np.ones(6))

        # publishing
        self.publisher = rospy.Publisher("/ekf_pose", PoseWithCovarianceStamped, queue_size=3)
        self.seq_num = 0
       
        self.timer = rospy.Timer(rospy.Duration(self.period), self.step)

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()

        msg.header.seq = self.seq_num
        self.seq_num += 1
        msg.header.stamp = rospy.get_rostime().to_sec()
        msg.header.frame_id = 'map'
        
        position = msg.pose.pose.position
        position.x, position.y = self.state[0], self.state[1]
        orientation = msg.pose.pose.orientation
        quat = quat_from_rpy_rad(0.0, 0.0, self.state[2])
        orientation.x, orientation.y, orientation.z, orientation.w = quat

        # how the hell do I convert covariance between heading and other values
        # to covariance between quaternion and other values??

        # => don't actually care about covariance between anything other than x, y in my applcation
        # so just set covariances for quaternion to 0?

        cov = np.zeros((6,6))
        cov[:3, :3] = self.cov[:3, :3]
        msg.pose.covariance = list(cov)

        self.publisher.publish(msg)


    def step(self, event):
        if not self.true_positioning.ready():
            print("True positining not ready - not starting EKF")
            return

        dt = event.last_duration
        if dt is None:
            return

        self.predict(dt)
        print("State: {0}, Covariance: {1}".format(self.state, self.cov))

        for source in self.sensing_sources:
            if source.has_new_data():
                self.update(source)

    def predict(self, dt):

        real_deltas = self.get_real_deltas()
        sampled_deltas, sampled_stddevs = self.sample_deltas(real_deltas)

        # apply `f` ie. motion model
        self.state, Q_k = self.motion_model(self.state, sampled_deltas, sampled_stddevs, dt)

        # get jacobian
        F_k = self.motion_model_jacobian(self.state, sampled_deltas, dt)
        # compute update covariance
        self.cov = matmul(F_k, matmul(self.cov, F_k.T)) + Q_k


    
    def update(self, sensor_source):
        """ Following Wikipedia EKF notation """
        sensor_values = sensor_source.consume_data()
        h_t = sensor_source.evaluate(self.state)
        # compute residual
        y_t = sensor_values - h_t

        # get jacobian
        H_t = sensor_source.jacobian(self.state)

        # get sensor source covariance
        R_t = sensor_source.get_covariance()
        
        # residual covariance/innovation covariance
        S_t = matmul(H_t, matmul(self.cov, H_t.T)) + R_t

        # gain
        K_t = matmul(self.cov, matmul(H_t, np.linalg.inv(S_t))) # this is the expensive step?

        # update estimates
        self.state = self.state + matmul(K_t, y_t)
        self.cov = matmul((self.I - matmul(K_t, H_t)), self.cov)
        
    
    def get_real_deltas(self):
        # get d_translation, rotation 1, rotation 2 from real simulation position

        odom = self.true_positioning.get_odom() # get the latest position... might need to interpolate a bit? #TODO
        pose = odom.pose.pose
        position = get_as_numpy_position(pose.position)
        rpy = quat_to_rpy(get_as_numpy_quaternion(pose.orientation)) 
        heading_angle = rpy[2] # yaw == heading
        dist, d_start_angle, d_end_angle = 0.0, 0.0, 0.0 
        if self.last_true_position is not None:
            diff = position - self.last_true_position
            angle = np.arctan2(diff[1], diff[0])
            dist = np.linalg.norm(diff)
            d_start_angle = angle - self.last_true_theta
            d_end_angle = heading_angle - self.last_true_theta - d_start_angle

        self.last_true_position = position
        self.last_true_theta = heading_angle

        return [dist, d_start_angle, d_end_angle]
        
    def sample_deltas(self, real_deltas):

        dist, d_start_angle, d_end_angle = real_deltas
        
        diff, rotation_1, rotation_2 = real_deltas
        alpha_1, alpha_2, alpha_3, alpha_4 = self.model_step_noise_coeffs

        stddev_start_angle_noise = alpha_1 * np.abs(d_start_angle) + alpha_2 * diff
        stddev_end_angle_noise = alpha_1 * np.abs(d_end_angle) + alpha_2 * dist
        stddev_dist_noise = alpha_3 * dist + alpha_4 * np.abs(d_start_angle + d_end_angle)

        error_start_angle = np.random.normal(0.0, stddev_start_angle_noise)
        error_end_angle = np.random.normal(0.0, stddev_end_angle_noise)
        error_dist = np.random.normal(0.0, stddev_dist_noise)

        sampled_dist = dist + error_dist
        sampled_start_angle = d_start_angle + error_start_angle
        sampled_end_angle = d_end_angle + error_end_angle
        
        samples = (sampled_dist, sampled_start_angle, sampled_end_angle)
        stddevs = (stddev_start_angle_noise, stddev_end_angle_noise, stddev_dist_noise)
        return (samples, stddevs) 


    def motion_model(self, state, sampled_deltas, sampled_stddevs, dt):
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

        sampled_dist, sampled_start_angle, sampled_end_angle = sampled_deltas
        sampled_dist_stddev, sampled_start_angle_stddev, sampled_end_angle_stddev = sampled_stddevs

        # we are only going to update position and orientation
        # and not touch velocity components
        # note that we only save the angle here and convert it when
        # we want to export it!

        dx = sampled_dist * np.cos(state[2] + sampled_start_angle)
        state[0] += dx
        dy = sampled_dist * np.sin(state[2] + sampled_start_angle)
        state[1] += dy
        dtheta = sampled_start_angle + sampled_end_angle 
        state[2] += dtheta 

        state[3] = dx/dt
        state[4] = dy/dt
        state[5] = dtheta/dt 

        # form Q process noise matrix
        # this is definitely NOT right
        # should be cross-correlations
        # and also just ignoring cos and sin effects on variance...
        Q = np.diag(np.tile([
            sampled_dist_stddev**2,
            sampled_dist_stddev**2,
            sampled_start_angle_stddev**2 + sampled_end_angle_stddev**2], 2))
        
        return (state, Q)

    def motion_model_jacobian(self, state, sampled_deltas, dt):
        
        sampled_dist, sampled_start_angle, sampled_end_angle = sampled_deltas

        F = np.diag([1.0, 1, 1, 0, 0, 0])

        # only the column corresponding to theta has cross-termed derivatives
        a = -sampled_dist * np.sin(state[2] * sampled_start_angle)
        b = sampled_dist * np.cos(state[2] * sampled_end_angle)
        d_theta_column = np.array([a, b, 1.0, a/dt, b/dt, 0.0])

        F[:, 2] = d_theta_column

        return F



if __name__ == "__main__":
    rospy.init_node("ekf_positioning")


    # wait until clock messages arrive
    # and hence simulation is ready
    while rospy.get_time() == 0:
        rospy.sleep(1)

    ekf = OdometryEKF() 
    rospy.spin()
