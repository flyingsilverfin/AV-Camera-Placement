#!/usr/bin/env python

import math
import rospy

class Path(object):
    def __init__(self):
        pass


class Line_Follow(object):
    def __init__(self):
        rospy.loginfo("Create Line_Follow object!")

if __name__ == "__main__":
    rospy.init_node("line_follow_py")
    line_follow = Line_Follow()
    rospy.spin()
