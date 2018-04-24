"""
A ROS node that takes in a base pose ground truth
And corrupts it to emulate wheel encoder geometry
The general idea is pretty intuitive - 
add a random walk of noise to the position and orientation

Specific implementation follows 

http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/06-motion-models.pdf
"""

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from helper import get_as_numpy_position, get_as_numpy_quaternion, quat_to_rpy, quat_from_rpy


class FakeWheelOdometry(object):
    
    def __init__(self, true_position_topic, fake_position_topic, alpha_1=0.1, alpha_2=0.1, alpha_3=0.1, alpha_4=0.1):
       
        self.rate = rospy.get_param("fake_wheel_odom_rate", 50.0)
        self.period = 1.0/self.rate
        
        rospy.Subscriber(true_position_topic, Odometry, self.on_update)
        self.fake_odom_publisher = rospy.Publisher(fake_position_topic, Odometry, queue_size=3)

        # these need to be retained to aggregate errors
        self.fake_odom_position = None
        self.fake_odom_heading_angle = None
        self.fake_odom_cov = None
        
        # used to compute errors to accumulate into fake_odom values
        self.last_true_position = None
        self.last_true_heading_angle = None
        # true position has no covariance, don't need to save

        # stddev multipliers for computing error magnitudes
        self.alpha_1, self.alpha_2, self.alpha_3, self.alpha_4 = alpha_1, alpha_2, alpha_3, alpha_4

        self.timer = rospy.Timer(rospy.Duration(self.period), self.publish_odom)

    def on_update(self, msg):
        """
        http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/06-motion-models.pdf

        Implements a noise model on the odometry to model wheel odometry
        """

        position = get_as_numpy_position(msg.pose.pose.position)
        heading = get_as_numpy_quaternion(msg.pose.pose.orientation)
        heading_angle = quat_to_rpy(heading)[2] # yaw angle, from x axis

        if self.last_true_position is not None: 
            # general idea:
            # use true movement to compute scale of noise to add
            # to the last reported odometry

            diff = position - self.last_true_position
            dx, dy = diff[0], diff[1] 
            angle = np.arctan2(dy, dx)

            dist = np.linalg.norm(diff)
            d_start_angle = angle - self.last_true_heading_angle
            d_end_angle = heading_angle - self.last_true_heading_angle - d_start_angle
            
            stddev_start_angle_noise = self.alpha_1 * np.abs(d_start_angle) + self.alpha_2 * diff
            stddev_end_angle_noise = self.alpha_1 * np.abs(d_end_angle) + self.alpha_2 * dist
            stddev_dist_noise = self.alpha_3 * dist + self.alpha_4 * np.abs(d_start_angle + d_end_angle)

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

            self.fake_odom_position[0] += dist_with_error * np.cos(self.fake_odom_heading_angle + d_start_angle_with_error)
            self.fake_odom_position[1] += dist_with_error * np.sin(self.fake_odom_heading_angle + d_start_angle_with_error)
            
            self.fake_odom_heading_angle += d_start_angle_with_error + d_end_angle_with_error


        self.last_true_position = position
        self.last_true_heading_angle = heading
        
    def publish_odom(self, event):
        """ Takes last caculated fake wheel odometry and converts it to position and orientation """

        if self.fake_odom_position is None:
            # haven't updated yet
            return

        # TODO how to deal with covariances!!!

        odom = Odometry()
        odom.pose.pose.position = Point(*self.fake_odom_position)

        quat = Quaternion()
        orientation_quat = quat_from_rpy(0.0, 0.0, self.fake_odom_heading_angle)
        quat.x, quat.y, quat.z, quat.w = orientation_quat

        odom.pose.pose.orientation = quat

        self.fake_odom_publisher.publish(odom)

if __name__ == '__main__':
    rospy.init_node("FakeWheelOdometry")
    
    # wait until clock messages arrive
    # and hence simulation is ready
    while rospy.get_time() == 0:
        rospy.sleep(1)

    fake_odom = FakeWheelOdometry("/base_pose_ground_truth",
                                  "/wheel_odometry")

    rospy.spin()
