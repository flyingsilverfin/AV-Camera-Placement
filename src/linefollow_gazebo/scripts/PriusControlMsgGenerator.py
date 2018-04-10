import rospy
from prius_msgs.msg import Control


class PriusControlMsgGenerator(object):
    """PriusControlMsgGenerator stateless (except seq) prius Control message generator"""
    def __init__(self):
        self.seq = 0

    def populate_header(self, prius_msg):
        """populate_header

        :param prius_msg: a Control msg to populate header of
        """
        header = prius_msg.header

        # insert sequence number
        header.seq = self.seq
        self.seq += 1
       	
        # insert seconds and nanoseconds
        now = rospy.get_rostime()
        header.stamp.secs = now.secs
        header.stamp.nsecs = now.nsecs
        
        # TODO this should be changed to frame_id of base_link?
        header.frame_id = "" 

    def update_msg(self, msg):
        """update_msg takes an existing Control msg and updates seq number

        :param msg:
        """

        msg.header.seq += 1
        self.seq = msg.header.seq
    
    def forward(self, throttle, steer_angle):
        """forward generates a forward-moving Control msg with throttle and steering angle

        :param throttle: throttle in range [0.0, 1.0]
        :param steer_angle: steering angle in range [-1.0, 1.0]
        """
        assert throttle >= 0.0 and throttle <= 1.0, "Require throttle to be within range [0.0, 1.0]"
        assert steer_angle >= -1.0 and steer_angle <= 1.0, "Require steer angle to be in range [-1.0, 1.0]"
        msg = Control()
        self.populate_header(msg)
        msg.throttle = throttle	# throttle as given
        msg.steer = steer_angle	# steering angle as given
        msg.brake = 0.0	    # no brake	
        msg.shift_gears = 2 # forward gearing

        return msg
