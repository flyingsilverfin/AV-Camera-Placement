import abc
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# TODO
# Implement the get_pose method
# in an abstracted method
# that returns position & velocity
# and/or orientation
# agnostic of the source
# TODO will require rewrite of LineFollowController



class Positioning(object):
    def __init__(self):
        self.last_pose = None

    def on_update(self, msg):
        self.last_pose = msg

    @abc.abstractmethod
    def get_pose(self):
        pass

class TruePositioning(Positioning):
    def __init__(self):
        super(TruePositioning, self).__init__()
        rospy.loginfo("Create True Positioning")
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.on_update)


    def get_pose(self):
        # TODO made this source agnostic
        return self.last_pose

# Next steps...
class EKFPositioning(Positioning):
    def __init__(self):
        super(TruePositioning, self).__init__()
        pass
    
    def on_imu(self, imu_msg):
        rospy.loginfo_throttle(10, str(imu_msg.linear_acceleration))
        self.last_imu = imu_msg
        
