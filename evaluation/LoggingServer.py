import os
import rospy
import rosbag
import rostopic
import numpy as np
from PIL import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, CameraInfo

class Logger(object):
    """ Endpoint handling service requests to record N topic messages. Currently stays subscribed until told to unsubscribe """
    
    def __init__(self):
        # self.root_dir = root_log_dir
        self.active_logs = {}
        rospy.init_node("Logger")
        self.img_bridge = CvBridge()

        # new approach: use rosbag, much more sensible!!
        self.rosbags = {}

    def rosbag_topic(self, topic, save_path):
        if topic in self.rosbags:
            print("Already bagging: {0}".format(topic))
            return
        rb = rosbag.Bag(save_path, 'w')
        self.rosbags[topic] = rb
    
    def receive_msg_to_rosbag(self, msg, topic):
        rb = self.rosbags[topic]
        try:
            rb.write(msg)
        except Exception as e:
            print(e)
    
    def close_rosbag(self, topic):
        rb = self.rosbags[topic]
        rb.close()

    # ------- old methods -------

    def log_n_messages(self, topic, save_dir, num_msgs, callback):
        # num_msgs == -1 is a signal to log ALL messages

        if topic in self.active_logs:
            data = self.active_logs[topic]
            remaining = data['msgs_to_save']
            if save_dir != data['save_dir']:
                # changing target save directory
                if remaining > 0:
                    # issue a warning if overriding some awaited messages
                    print("\nWARNING: trying to save to a different subdirectory but still awaiting messages, overriding awaited ones")
                # change the msg dir
                data['counter'] = 0
                data['save_dir'] = save_dir 
                data['msgs_to_save'] = num_msgs
                data['callback'] = callback
            else:
                # save directory unchanged
                data['msgs_to_save'] += num_msgs # queue another N messages to save
        else:
            # create a new subscriber for the topic and save that many messages
            subscriber = rospy.Subscriber(topic, rostopic.get_topic_class(topic)[0], self.msg_received, callback_args=(topic))
            self.active_logs[topic] = {
                'subscriber': subscriber,
                'msgs_to_save': num_msgs,
                'save_dir': save_dir,
                'counter': 0,
                'callback': callback
            }
        

    def unsubscribe(self, topic):
        if topic in self.active_logs:
            subscriber = self.active_logs[topic]
            subscriber.unregister()
            del self.active_logs[topic]
            return "Successfully unsubscribed from {0}".format(topic)
        
        return "Already not subscribed to {0}".format(topic)

    def msg_received(self, msg, source_topic):
        logger = self.active_logs[source_topic]
        if logger['msgs_to_save'] == 0: # < 0 is store everything
            return


        # hardcode some message types and handling thereof
        if type(msg) == Image:

            try:
                cv_image = self.img_bridge.imgmsg_to_cv2(msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            filename = os.path.join(logger['save_dir'], 'image_{0}.png'.format(logger['counter']))
            cv2.imwrite(filename, cv_image)

        else:
            pass 
            

        logger['callback'](msg)  # pass control back to caller to do aggregate statistics etc if wanted
        logger['counter'] += 1
        if logger['msgs_to_save'] > 0:
            logger['msgs_to_save'] -= 1

    def finished_awaiting_msgs(self, topic):
        if topic in self.active_logs:
            return self.active_logs[topic]['msgs_to_save'] == 0

        print("Topic not being waited upon!")
        return True
