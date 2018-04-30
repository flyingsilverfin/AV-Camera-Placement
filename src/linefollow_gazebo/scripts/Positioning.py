import abc
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped

# TODO
# Implement the get_pose method
# in an abstracted method
# that returns position & velocity
# and/or orientation
# agnostic of the source
# TODO will require rewrite of LineFollowController



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
    def __init__(self):
        super(TruePositioning, self).__init__()
        rospy.loginfo("Create True Positioning")
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.on_update)



# Next steps...
class EKFPositioning(Positioning):
    def __init__(self):
        super(EKFPositioning, self).__init__()

        rospy.Subscriber('/odometry/filtered', Odometry, self.on_update)
        self.pose_estimate_repub = rospy.Publisher('/pose_estimate', PoseWithCovarianceStamped, queue_size=4)
        
    def on_update(self, msg):
        super(EKFPositioning, self).on_update(msg)

        p = PoseWithCovarianceStamped()
        p.header = self.last_odom.header
        p.pose = self.last_odom.pose
        self.pose_estimate_repub.publish(p)
        self.last_pose = p 



    def last_pose(self):
        return self.last_pose

    def get_position_variance(self):
        p = self.last_odom.pose
        cov = p.covariance

        # just get the diagonal?
        c = np.array(cov).reshape(6, 6)
        var = np.diag(c)
        return var
        
