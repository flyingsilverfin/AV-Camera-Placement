#!/usr/bin/python

import abc
import numpy as np
from numpy import matmul
import rospy

from Positioning import TruePositioning
from helper import quat_to_rpy, get_as_numpy_quaternion, get_as_numpy_position, quat_from_rpy_rad, get_as_numpy_velocity_vec

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class SensorSource(object):
    """ Superclass for sensor sources """
    def __init__(self, topic, msg_type, update_states, noise_matrix, update_period=5.0):

        subscriber = rospy.Subscriber(topic, msg_type, self.on_msg)
        self.consumed_last_msg = True
        self.R_k = noise_matrix
        self.update_states = update_states

        # init some state
        self.state_data = np.zeros_like(self.update_states)

        # for regulating updates
        self.last_update_consumed_time = 0.0
        self.update_period = update_period

    def on_msg(self, msg):
        self.consumed_last_msg = False
        self.process_msg(msg)

    def has_new_data(self):
        return rospy.get_rostime().to_sec() - self.last_update_consumed_time > self.update_period and not self.consumed_last_msg
        
    @abc.abstractmethod
    def process_msg(self, msg):
        pass

    def consume_state(self):
        now = rospy.get_rostime().to_sec()
        if now - self.last_update_consumed_time > self.update_period:
            self.consumed_last_msg = True
            self.last_update_consumed_time = now
            return self.state_data

    def calculate_expected_state(self, current_state):
        # expected is h(state) ie. the measurement we should see using current estimate
        expected = current_state * self.update_states # just pull out the states we want to update
        return expected

    @abc.abstractmethod
    def get_jacobian(self, state):
        pass

    def get_noise_matrix(self):
        return self.R_k

    def get_updatable_states(self):
        return self.update_states

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

class FakeOdomSensorSource(SensorSource):
    """ 
    SensorSource that takes some Odometry source (likely ground truth)
    adds some noise, and returns it as a measurement (eg. imagine simulating GPS or something)
    """

    def __init__(self, topic, noise_variance=1e-2, update_period=5.0):
        noise = np.diag(np.repeat([noise_variance], 6))
        # update x,y, v_x, v_y positions
        update_states = np.array([True, True, True, True, True, True])
        super(FakeOdomSensorSource, self).__init__(topic, Odometry, update_states, noise, update_period)

        self.static_jacobian = np.diag(np.ones(6)) * self.update_states

    def process_msg(self, odom):
        """ Converts Odometry message into a a state vector """

        # most sensor-specific code goes here
        # in effect converts specific message into data consumable by the EKF

        # since this is face odom, we use the true state
        # and add some noise
        # and save that as R_k, sensor noise

        position = get_as_numpy_position(odom.pose.pose.position)
        orientation = quat_to_rpy(get_as_numpy_quaternion(odom.pose.pose.orientation))
        linear_velocity = get_as_numpy_velocity_vec(odom.twist.twist.linear)
        angular_velocity = get_as_numpy_velocity_vec(odom.twist.twist.angular)

        # note: true data has no covariance, we will add diagonal cov later
        x, y = position[:2]
        heading = orientation[2]
        vx, vy = linear_velocity[:2]
        v_theta = angular_velocity[2]
        
        self.state_data = np.array([x, y, heading, vx, vy, v_theta]) * self.update_states 

    def get_jacobian(self, state):
        return self.static_jacobian 


class OdometryEKF(object):
    """ EKF tracking, x, y, heading (theta), x', y', theta' """

    def __init__(self, model_step_noise_coeffs=np.array([0.01, 0.01, 0.01, 0.01]), motion_model_threshold=0.001, publish_rviz_pose=True):

        self.model_step_noise_coeffs = model_step_noise_coeffs

        self.rate = rospy.get_param("~ekf_rate", 30.0)   
        self.period = 1.0/self.rate

        self.true_positioning = TruePositioning()
        self.last_true_position, self.last_true_theta = None, None
        self.last_true_position_time = None
        self.last_true_position_time = None

        # state: x, y, theta, x', y', theta'
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.cov = np.zeros((6,6))
        self.motion_model_threshold = motion_model_threshold

        # we can attach update sources here, conforming to SensorSource interface 
        self.sensors = []

        # reusable matrices, saves initization
        self.I = np.diag(np.ones(6))

        # publishing
        self.publish_rviz_pose = publish_rviz_pose
        self.odom_publisher = rospy.Publisher("/ekf_odom", Odometry, queue_size=3)
        if publish_rviz_pose:
            self.pose_publisher = rospy.Publisher("/ekf_pose", PoseWithCovarianceStamped, queue_size=3)
        self.seq_num = 0

        self.timer = rospy.Timer(rospy.Duration(self.period), self.step)

    def publish_pose(self):

        odom = Odometry()

        header = odom.header
        header.seq = self.seq_num
        self.seq_num += 1
        header.stamp = rospy.get_rostime()
        header.frame_id = 'map'
 
        position = odom.pose.pose.position
        position.x, position.y = self.state[0], self.state[1]
        orientation = odom.pose.pose.orientation
        quat = quat_from_rpy_rad(0.0, 0.0, self.state[2])
        orientation.x, orientation.y, orientation.z, orientation.w = quat

        # how the hell do I convert covariance between heading and other values
        # to covariance between quaternion and other values??
        # => don't actually care about covariance between anything other than x, y in my applcation
        # so just set covariances for quaternion to 0?

        cov = np.zeros((6,6))
        cov[:3, :3] = self.cov[:3, :3]
        odom.pose.covariance = list(cov.ravel())

        # insert velocity information
        linear_vel= odom.twist.twist.linear
        linear_vel.x, linear_vel.y, linear_vel.z = self.state[3], self.state[4], 0.0

        angular_vel = odom.twist.twist.angular
        angular_vel.z = self.state[5] # TODO check this is right!
        # don't bother setting covariance since I don't understand it for angular vel's and I don't actually use it

        self.odom_publisher.publish(odom)

        if self.publish_rviz_pose:
            pose = PoseWithCovarianceStamped()
            pose.header = header
            pose.pose = odom.pose
            self.pose_publisher.publish(pose)

    def step(self, event):
        if not self.true_positioning.ready():
            print("True positining not ready - not starting EKF")
            return
        if self.last_true_position_time is None:
            self.last_true_position_time = self.true_positioning.get_odom_time()

        true_pos_time = self.true_positioning.get_odom_time()
        dt = (true_pos_time - self.last_true_position_time).to_sec()
        if dt == 0: # not received new true position, don't bother updating
            return

        self.predict(dt)
        print("State: {0}, Covariance: {1}".format(self.state, self.cov))

        for sensor_source in self.sensors:
            print("Sensor has new data: {0}".format(sensor_source.has_new_data()))
            if sensor_source.has_new_data():
                self.update(sensor_source)

        self.publish_pose()

        self.last_true_position_time = true_pos_time

    def predict(self, dt):

        real_deltas = self.get_real_deltas()

        # print("Predicting - real deltas: {0}, time since last used odom: {1}".format(real_deltas, dt))
        sampled_deltas, sampled_stddevs = self.sample_deltas(real_deltas)
        # print("\tsampled deltas, stddevs added: {0} with stddev {1}".format(sampled_deltas, sampled_stddevs))

        # apply `f` ie. motion model
        self.state, Q_k = self.motion_model(self.state, sampled_deltas, sampled_stddevs, dt)
        # print("\tstate: {0}".format(self.state))

        # get jacobian
        F_k = self.motion_model_jacobian(self.state, sampled_deltas, dt)
        # print("--Covariances--\n Jacobian: \n {0} \n Current Covariance: \n {1} \n Q_k: \n {2}".format(F_k, self.cov, Q_k))
        # compute update covariance
        self.cov = matmul(F_k, matmul(self.cov, F_k.T)) + Q_k


    
    def update(self, sensor_source):
        """ Following Wikipedia EKF notation """
        sensor_values = sensor_source.consume_state()
        h_t = sensor_source.calculate_expected_state(self.state)

        # compute residual
        y_t = sensor_values - h_t

        # get jacobian
        H_t = sensor_source.get_jacobian(self.state)
        # get sensor source covariance
        R_t = sensor_source.get_noise_matrix()
        
        # residual covariance/innovation covariance
        S_t = matmul(H_t, matmul(self.cov, H_t.T)) + R_t

        # gain
        K_t = matmul(self.cov, matmul(H_t, np.linalg.inv(S_t))) # this is the expensive step?

        # update estimates
        self.state = self.state + matmul(K_t, y_t)
        self.cov = matmul((self.I - matmul(K_t, H_t)), self.cov)

        print("Updated state!")
        print("State: \n {0}, \n Covariance: \n {1}".format(self.state, self.cov))
        
    
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
            if dist < self.motion_model_threshold:
                return [0.0, 0.0, 0.0]
            d_start_angle = angle - self.last_true_theta
            d_end_angle = heading_angle - angle
            # print("\t[real deltas] Position: {0}, last_position: {1}, movement last => now: {2}".format(position, self.last_true_position, diff))
            # print("\t[real deltas] angle of movement: {0}, distance: {1}, current rpy: {3}, last true angle: {2}".format(angle, dist, self.last_true_theta, rpy)) 
            # print("\t[real deltas] start angle to movement angle: {0}, end angle to movement angle {1}".format(d_start_angle, d_end_angle))

        self.last_true_position = position
        self.last_true_theta = heading_angle

        return [dist, d_start_angle, d_end_angle]
        
    def sample_deltas(self, real_deltas):

        
        dist, rotation_1, rotation_2 = real_deltas
        alpha_1, alpha_2, alpha_3, alpha_4 = self.model_step_noise_coeffs

        stddev_start_angle_noise = alpha_1 * np.abs(rotation_1) + alpha_2 * dist
        stddev_end_angle_noise = alpha_1 * np.abs(rotation_2) + alpha_2 * dist
        stddev_dist_noise = alpha_3 * dist + alpha_4 * np.abs(rotation_1 + rotation_2)

        error_start_angle = np.random.normal(0.0, stddev_start_angle_noise)
        error_end_angle = np.random.normal(0.0, stddev_end_angle_noise)
        error_dist = np.random.normal(0.0, stddev_dist_noise)

        sampled_dist = dist + error_dist
        sampled_start_angle = rotation_1 + error_start_angle
        sampled_end_angle = rotation_2 + error_end_angle
        
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
        a = -sampled_dist * np.sin(state[2] +  sampled_start_angle)
        b = sampled_dist * np.cos(state[2] + sampled_start_angle)
        d_theta_column = np.array([a, b, 1.0, a/dt, b/dt, 0.0])

        F[:, 2] = d_theta_column

        return F

    def attach_sensor(self, sensor):
        if not isinstance(sensor, SensorSource):
            print("WARN: given sensor is not of type SensorSource, not adding to EKF")
            return
        self.sensors.append(sensor)



if __name__ == "__main__":
    rospy.init_node("ekf_positioning")

    # wait until clock messages arrive
    # and hence simulation is ready
    while rospy.get_time() == 0:
        rospy.sleep(1)

    ekf = OdometryEKF(model_step_noise_coeffs=np.array([0.001, 0.001, 0.001, 0.001]), motion_model_threshold=0.001, publish_rviz_pose=True)
    # fake_sensor = FakeOdomSensorSource("/base_pose_ground_truth", noise_variance=0.1, update_period=.0)
    # ekf.attach_sensor(fake_sensor) 

    rospy.spin()
