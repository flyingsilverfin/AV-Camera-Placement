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
from std_srvs.srv import SetBool
from custom_messages.msg import CameraUpdate, SimulationDataMsg

from CameraModel import Camera

class SensorSource(object):
    """ Superclass for sensor sources """
    def __init__(self, topic, msg_type, update_states, noise_matrix, update_period=5.0):

        subscriber = rospy.Subscriber(topic, msg_type, self.on_msg)
        self.consumed_last_msg = True
        self.R_k = noise_matrix
        self.update_states = update_states

        self.n_states = np.count_nonzero(update_states)

        # init some state
        self.state_data = np.zeros_like(self.n_states)

        # for regulating updates
        self.last_update_consumed_time = 0.0
        self.update_period = update_period

    def on_msg(self, msg):
        self.consumed_last_msg = False
        self.process_msg(msg)

    def has_new_data(self):
        return not self.consumed_last_msg and rospy.get_rostime().to_sec() - self.last_update_consumed_time > self.update_period
        
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
        # I just return the relevant current_states!

        #resize expected down to those elements just containing values we track (valid state)
        valid = self.update_states == True 
        return current_state[valid]

    @abc.abstractmethod
    def get_jacobian(self, state):
        pass

    def get_noise_matrix(self):
        return self.R_k


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

class CameraSensorSource(SensorSource):

    def __init__(self, topic):
        update_states = np.array([True, True, False, False, False, False])

        super(CameraSensorSource, self).__init__(topic, CameraUpdate, update_states, None, 0)

        # nx6 array, with ones in first diagonal
        self.static_jacobian = np.zeros(shape=(self.n_states, 6))
        self.static_jacobian[0,0] = 1
        self.static_jacobian[1,1] = 1


    def process_msg(self, camera_update):
        print("Received camera update mssage!")
        pos = camera_update.position
        self.R_k = np.array(camera_update.covariance).reshape(2,2)
        # only care about top left 2x2
        
        self.state_data = np.array([pos.x, pos.y])
        self.update_time = camera_update.header.stamp.to_sec()

        self.last_msg = camera_update
        
    
    def get_jacobian(self, ekf_state):
        return self.static_jacobian

    def consume_state(self, ekf_time, ekf_state, ground_truth_odom):
        """ TODO cheat here...
            Something isn't matching up in my camera =>  vehicle update (big distances!)
            Instead, use a normal sensor model eg. use the covariance here
            to sample within the error ellipse and add it to the ground truth state...
            This is kinda voiding my proposition a bit since i'm not testing the whole
            raytrace => estimate bit but I showed earlier the model fits well...
        """
        position = super(CameraSensorSource, self).consume_state()
        if position is None:
            return

        # apply a time correction using current EKF state and time from message stamp/current time
        # msg_time = self.update_time
        # dt = ekf_time - msg_time
        # v_x, v_y = ekf_state[3], ekf_state[4]
        # dx, dy = v_x * dt, v_y * dt
        # position += np.array([dx, dy])

        cov = self.R_k
        
        sampled_delta = np.random.multivariate_normal(np.array([0,0]), cov)
        ground_truth_position = get_as_numpy_position(ground_truth_odom.pose.pose.position)[:2]
        perturbed = ground_truth_position + sampled_delta
        print("Pertubring Ground truth position {0} with delta {1} to form {2}".format(ground_truth_position, sampled_delta, perturbed))
        self.last_msg.position.x, self.last_msg.position.y = perturbed 
        return perturbed


        # return position

    def get_noise_matrix(self):
        return self.R_k*2



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

        self.static_jacobian = np.diag(np.ones(6) * self.update_states)

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

    def __init__(self, model_step_noise_coeffs=np.array([0.01, 0.01, 0.01, 0.01]), motion_model_threshold=0.001, update_rate=25.0, initial_cov_diag=np.array([0.0,0,0,0,0,0]), publish_rviz_pose=True):

        self.model_step_noise_coeffs = model_step_noise_coeffs

        self.rate = update_rate 
        self.period = 1.0/self.rate
        print("EKF update rate: {0}".format(self.rate))

        # self.true_positioning = TruePositioning()
        self.true_position_subscriber = rospy.Subscriber('/base_pose_ground_truth', Odometry, self.receive_ground_truth)
        self.new_true_odom, self.new_true_odom_time = None, None
        self.last_true_position, self.last_true_theta = None, None
        self.last_true_position_time = None

        # state: x, y, theta, x', y', theta'
        self.state = None # will use true state when first stepping #np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.cov = np.diag(initial_cov_diag)
        self.motion_model_threshold = motion_model_threshold

        # we can attach update sources here, conforming to SensorSource interface 
        self.sensors = []

        # reusable matrices, saves initization
        self.I = np.diag(np.ones(6))

        # publishing
        self.publish_rviz_pose = publish_rviz_pose
        self.odom_publisher = rospy.Publisher("/ekf_odom", Odometry, queue_size=25)
        if publish_rviz_pose:
            self.pose_publisher = rospy.Publisher("/ekf_pose", PoseWithCovarianceStamped, queue_size=25)

        self.seq_num = 0

        self.running = False


        # for transmitting odoms to sim data collector
        self.to_sim_data_collector = rospy.Publisher("/ekf_to_simcollector", SimulationDataMsg, queue_size=25)

    def set_running(self, msg):
        """ Called by service to enable/disable EKF """
        set_run = msg.data
        if set_run:
            return {"success": self.start(), "message": ""}
        else:
            return {"success" : self.stop(), "message": ""}

    def start(self):
        if self.running:
            print("EKF already running")
            return False
        else:
            self.running = True
            # self.timer = rospy.Timer(rospy.Duration.from_sec(self.period), self.step)
            # above unrealiable with interleaving data arrival/use

            return True

    def stop(self):
        if not self.running:
            print("EKF already running")
            return False
        else:
            self.running = False
            # self.timer.shutdown()
            self.true_position_subscriber.unregister()
            return True
        

    def publish_pose(self, last_ekf_state, pre_update_ekf_state, pre_update_ekf_cov, last_true_position, last_true_theta, current_true_odom):
        print("Publishing EKF pose! Time: {0}".format(rospy.get_rostime().to_sec()))

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

        # ----publish a SimDataMessage for the sim data collector to deal with---
        if last_true_position is None:
            return

        msg = SimulationDataMsg()
        msg.true_odom = current_true_odom
        p = msg.last_true_pos
        p.x, p.y, p.z = last_true_position
        msg.last_true_heading = last_true_theta
        msg.last_ekf_state = last_ekf_state.tolist()

        # NOTE: we may have had a camera update here
        # so we want to write the position before jumping due to camera update!
        position = msg.ekf_odom.pose.pose.position
        position.x, position.y = pre_update_ekf_state[0], pre_update_ekf_state[1]
        orientation = msg.ekf_odom.pose.pose.orientation
        quat = quat_from_rpy_rad(0.0, 0.0, pre_update_ekf_state[2])
        orientation.x, orientation.y, orientation.z, orientation.w = quat
    
        cov = np.zeros((6,6))
        cov[:3, :3] = pre_update_ekf_cov[:3, :3]
        msg.ekf_odom.pose.covariance = list(pre_update_ekf_cov.ravel())
        msg.ekf_odom.header.stamp = rospy.get_rostime()

        # insert velocity information
        linear_vel= msg.ekf_odom.twist.twist.linear
        linear_vel.x, linear_vel.y, linear_vel.z = pre_update_ekf_state[3], pre_update_ekf_state[4], 0.0

        angular_vel = msg.ekf_odom.twist.twist.angular
        angular_vel.z = pre_update_ekf_state[5]

        # insert camera update if had any
        if self.last_sensor_position is None:
            msg.has_camera_update = False
        else:
            msg.has_camera_update = True
            # save the saved values into the message
            # have to do it this way because 
            # ROS may interrupt/receive new data 
            # at any time it seems...
            msg_pos = msg.camera_update.position
            msg_pos.x, msg_pos.y = self.last_sensor_position
            msg.camera_update.covariance = self.last_sensor_cov.ravel().tolist()


        self.to_sim_data_collector.publish(msg)

    def receive_ground_truth(self, odom):
        self.new_true_odom = odom
        self.new_true_odom_time = odom.header.stamp.to_sec()
        print("Received new ground truth at time: {0}".format(self.new_true_odom_time))

        self.step()

    def step(self): #, event):

        # if not self.true_positioning.ready():
        if self.new_true_odom is None:
            print("True positioning not ready - not starting EKF")
            return
        if self.last_true_position_time is None:
            self.last_true_position_time = self.new_true_odom_time

        if self.new_true_odom_time - self.last_true_position_time < self.period:
            return

        if self.state is None:
            # odom = self.true_positioning.get_odom()
            odom = self.new_true_odom
            position = odom.pose.pose.position
            orientation_rpy = quat_to_rpy(get_as_numpy_quaternion(odom.pose.pose.orientation)) # orientation quat => rpy
            self.state = np.array([position.x, position.y, orientation_rpy[2], 0.0, 0.0, 0.0]) # just 0 velocity will be overriden next update step anyway

        true_pos_time = self.new_true_odom_time 
        print("Using true position time: {0}, last true position time: {1}".format(true_pos_time, self.last_true_position_time))
        dt = (true_pos_time - self.last_true_position_time)
        if dt == 0: # not received new true position, don't bother updating
            print("DT is 0!")
            return

        # HACK retain these for publishing later
        if self.last_true_position is not None:
            last_true_pos = self.last_true_position.copy()
            last_true_heading = self.last_true_theta
        else:
            last_true_pos = np.array([self.state[0], self.state[1], 0.0])
            last_true_heading = 0.0

        current_true_odom = self.new_true_odom
        last_ekf_state = self.state.copy()

        self.predict(dt)

        pre_update_ekf_state = self.state.copy()
        pre_update_ekf_cov = self.cov.copy()

        # self.sensor_source_update = None
        self.last_sensor_position = None
        self.last_sensor_cov = None
        for sensor_source in self.sensors:
            if sensor_source.has_new_data():
                self.update(sensor_source)

        self.publish_pose(last_ekf_state, pre_update_ekf_state, pre_update_ekf_cov, last_true_pos, last_true_heading, current_true_odom)

        self.last_true_position_time = true_pos_time

    def predict(self, dt):

        real_deltas = self.get_real_deltas()
        if real_deltas == [0.0, 0.0, 0.0]:
            return 

        print("Predicting - real deltas: {0}, time since last used odom: {1}".format(real_deltas, dt))
        sampled_deltas, sampled_stddevs = self.sample_deltas(real_deltas)
        print("\tsampled deltas, stddevs added: {0} with stddev {1}".format(sampled_deltas, sampled_stddevs))

        pre_predict_state = self.state.copy()
        # apply `f` ie. motion model
        self.state, Q_k = self.motion_model(self.state, sampled_deltas, sampled_stddevs, dt)
        print("\tstate: {0}".format(self.state))

        # get jacobian
        F_k = self.motion_model_jacobian(pre_predict_state, sampled_deltas, dt)
        # print("--Covariances--\n Jacobian: \n {0} \n Current Covariance: \n {1} \n Q_k: \n {2}".format(F_k, self.cov, Q_k))
        # compute update covariance
        self.cov = matmul(F_k, matmul(self.cov, F_k.T)) + Q_k


    
    def update(self, sensor_source):
        """ Following Wikipedia EKF notation """
        sensor_values = sensor_source.consume_state(rospy.get_rostime().to_sec(), self.state, self.new_true_odom)
        self.last_sensor_position = sensor_values.copy() #HACK breaking my APIs :( running out of time though
        self.last_sensor_cov = sensor_source.R_k.copy()
        # self.sensor_source_update = sensor_source # save this for writing to sim data collector
        h_t = sensor_source.calculate_expected_state(self.state)
        print("Received update: {0}".format(sensor_values))
        print("current state is: {0}, current cov is : {1}".format(self.state, self.cov))

        # compute residual
        y_t = sensor_values - h_t
        print("Residual: {0}".format(y_t))
        

        # get jacobian
        H_t = sensor_source.get_jacobian(self.state)
        # get sensor source covariance
        R_t = sensor_source.get_noise_matrix()
        # R_t[0,1] = R_t[1,0] = 0
        print("Jacobian: {0}".format(H_t))
        print("Sensor noise: {0}".format(R_t))
        
        # residual covariance/innovation covariance
        S_t = matmul(H_t, matmul(self.cov, H_t.T)) + R_t
        print("Residual Covariance: {0}".format(S_t))

        # gain
        K_t = matmul(self.cov, matmul(H_t.T, np.linalg.inv(S_t))) # this is the expensive step
        print("Kalman gain: {0}".format(K_t))

        # update estimates
        self.state = self.state + matmul(K_t, y_t)
        print("Updated State: {0}".format(self.state))
        self.cov = matmul((self.I - matmul(K_t, H_t)), self.cov)

        print("Updated state!")
        print("State: \n {0}, \n Covariance: \n {1}".format(self.state, self.cov))
 

    def get_real_deltas(self):
        # get d_translation, rotation 1, rotation 2 from real simulation position

        # odom = self.true_positioning.get_odom() # get the latest position... might need to interpolate a bit? #TODO
        odom = self.new_true_odom
        pose = odom.pose.pose
        position = get_as_numpy_position(pose.position)
        rpy = quat_to_rpy(get_as_numpy_quaternion(pose.orientation)) 
        current_heading_angle = rpy[2] # yaw == heading
        dist, d_start_angle, d_end_angle = 0.0, 0.0, 0.0 
        if self.last_true_position is not None:
            diff = position - self.last_true_position
            angle = np.arctan2(diff[1], diff[0])
            dist = np.linalg.norm(diff)
            heading_angle = current_heading_angle # may be modified
            if dist < self.motion_model_threshold:
                return [0.0, 0.0, 0.0]
            # shift the deltas around into non-rollover zones!!
            # shift angles into 0 - 6.28
            if angle < 0 :
                angle += 2*np.pi
            last_true_theta = self.last_true_theta
            if last_true_theta < 0:
                last_true_theta += 2*np.pi
            if heading_angle < 0:
                heading_angle += 2*np.pi
            # now shift danger areas away!
            if angle > 5 or angle < 1 or last_true_theta > 5 or last_true_theta < 1 or heading_angle > 5 or heading_angle < 1:
                angle = (angle + np.pi) % (2*np.pi)
                last_true_theta = (last_true_theta + np.pi)%(2*np.pi)
                heading_angle = (heading_angle + np.pi)%(2*np.pi)
            # now safely calculate deltas
            d_start_angle = angle - last_true_theta
            d_end_angle = heading_angle - angle
            # print("\t[real deltas] Position: {0}, last_position: {1}, movement last => now: {2}".format(position, self.last_true_position, diff))
            # print("\t[real deltas] angle of movement: {0}, distance: {1}, current rpy: {3}, last true angle: {2}".format(angle, dist, self.last_true_theta, rpy)) 
            # print("\t[real deltas] start angle to movement angle: {0}, end angle to movement angle {1}".format(d_start_angle, d_end_angle))


            print("Current position: {0}, last positiong: {1}, start_angle: {2}, end_angle: {3}, Movement angle: {4}, initial angle error: {5}, finish angle error: {6}".format(position, self.last_true_position, last_true_theta, heading_angle, angle, d_start_angle, d_end_angle ))

        self.last_true_position = position
        self.last_true_theta = current_heading_angle

        return [dist, d_start_angle, d_end_angle]
        
    def sample_deltas(self, real_deltas):

        
        dist, rotation_1, rotation_2 = real_deltas
        alpha_1, alpha_2, alpha_3, alpha_4 = self.model_step_noise_coeffs

        stddev_start_angle_noise = alpha_1 * np.abs(rotation_1) + alpha_2 * dist
        stddev_end_angle_noise = alpha_1 * np.abs(rotation_2) + alpha_2 * dist
        stddev_dist_noise = alpha_3 * dist + alpha_4 * (np.abs(rotation_1) + np.abs(rotation_2))

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
        # d_theta_column = np.array([a, b, 1.0, a/dt, b/dt, 0.0])
        d_theta_column = np.array([a,b,1.0,0,0,0])

        F[:, 2] = d_theta_column

        return F

    def attach_sensor(self, sensor):
        if not isinstance(sensor, SensorSource):
            print("WARN: given sensor is not of type SensorSource, not adding to EKF")
            return
        self.sensors.append(sensor)



if __name__ == "__main__":
    rospy.init_node("ekf_positioning")


    ekf_params = rospy.get_param('/ekf')
    ekf_update_rate = ekf_params['update_rate']
    ekf_cov_diag = np.array(ekf_params['initial_cov_diagonal'])
    ekf_model_step_noise = np.array(ekf_params['model_step_noise'])
    ekf_model_thresh = ekf_params['motion_model_threshold']

    rviz = rospy.get_param('/rviz')




    # wait until clock messages arrive
    # and hence simulation is ready
    while rospy.get_time() == 0:
        rospy.sleep(1)


    ekf = OdometryEKF(model_step_noise_coeffs=ekf_model_step_noise, motion_model_threshold=ekf_model_thresh, update_rate=ekf_update_rate, initial_cov_diag=ekf_cov_diag, publish_rviz_pose=rviz)

    camera_update_source = CameraSensorSource('/camera_update')
    ekf.attach_sensor(camera_update_source)


    # TODO query parameter server to add sensor sources etc
    toggler = rospy.Service('/ekf/set_running', SetBool, ekf.set_running)

    rospy.spin()
