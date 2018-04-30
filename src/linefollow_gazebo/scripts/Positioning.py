import abc
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped



class Positioning(object):
    def __init__(self):
        self.last_odom = None
        self.last_msg_time = None

    def on_update(self, msg):
        self.last_odom = msg
        self.last_msg_time = msg.header.stamp 

    def get_odom(self):
        return self.last_odom 

    def get_odom_time(self):
        return self.last_msg_time

    def ready(self):
        return self.last_odom is not None

class TruePositioning(Positioning):
    def __init__(self, repub=False):
        super(TruePositioning, self).__init__()
        rospy.loginfo("Create True Positioning")
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.on_update)

        self.repub = repub
        if repub:
            self.repub = rospy.Publisher("/ground_truth_pose_msg", PoseWithCovarianceStamped, queue_size=3)

    def on_update(self, msg):
        self.last_odom = msg
        self.last_msg_time = msg.header.stamp
        
        if self.repub:
            self.republish_odom_as_pose()

    def republish_odom_as_pose(self):
        """ Republish Odom as PoseWithCovarianceStamped for RViz """
        pose = PoseWithCovarianceStamped()
        pose.header = self.last_odom.header
        pose.pose = self.last_odom.pose
        self.repub.publish(pose)


class EKFPositioning(Positioning):
    def __init__(self):
        super(EKFPositioning, self).__init__()

        rospy.Subscriber('/ekf_odom', Odometry , self.on_update)
        
        
