import rospy

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class VisualizeMarker(object):

    def __init__(self, rate=1):
        
        self.topic = rospy.Publisher("visualization_marker", Marker, queue_size=5)

        self.next_id = 0
        self.last_msg = rospy.get_time()
        self.publish_period = 1.0/rate

    def draw_n_points(self, points, width=0.5, height=0.5, rgba='default', duration=10.0):
        now = rospy.get_time()
        if now - self.last_msg < self.publish_period:
            return

        msg = Marker()
        msg.header.frame_id = "odom"
        msg.header.stamp = rospy.get_rostime()

        msg.id = self.next_id
        self.next_id += 1

        msg.type = Marker.POINTS
        msg.action = Marker.ADD

        if rgba == 'default':
             if len(points) > 3:
                 rospy.loginfo("More than 3 points to plot a time, using red for all of them")
                 colors = [[1.0, 0.0, 0.0, 1.0]]
             else:
                 colors = [[1.0, 0.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0], [0.0, 0.0, 1.0, 1.0]]
        for index, pt in enumerate(points):
            msg.points.append(Point(*pt))
            c = colors[index%len(colors)]
            msg.colors.append(ColorRGBA(*c))
        
        # c = colors[index%len(colors)] 

        q = msg.pose.orientation
        q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0

        msg.lifetime=rospy.Duration(duration)

        # I think z doesn't need to be specified for points
        msg.scale.x, msg.scale.y, msg.scale.z = width, height, height

        self.topic.publish(msg)
        self.last_msg = now



