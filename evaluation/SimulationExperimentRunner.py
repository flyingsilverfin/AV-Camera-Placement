import rospy
import roslaunch
import subprocess
import numpy as np
import time
import signal
import sys
sys.path.append("/home/joshua/Documents/Uni/Year4/dissertation/catkin_ws/src/linefollow_gazebo/scripts")

from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from std_srvs.srv import Empty, SetBool
from std_msgs.msg import Header

class Runner(object):
    """ The main interface for running experiments using the Gazebo/ROS simulation """

    def __init__(self, port):
        self.set_model_state_proxy = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.launch = None

        self.port = port

    def launch_core(self):
        print("Lauching core")
        self.roscore = subprocess.Popen(["roscore", "--port", str(self.port)])
        time.sleep(1)

    def shutdown_core(self):
        # need to SIGINT not SIGKILL otherwise it doesn't kill master with it!
        self.roscore.send_signal(signal.SIGINT)
        print("Interrupted roscore")
        self.roscore.wait()
    
    def set_param(self, name, value):
        rospy.set_param(name, value)

    def launch_nodes(self, launch_params_dict):
        launch_params_list = [":=".join(pair) for pair in launch_params_dict.items()]
        cli_args = ['linefollow_gazebo', 'linefollow.launch']
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, [(roslaunch_file[0], launch_params_list)])
        self.launch.start()
        # can shutdown with self.launch.shutdown()

    def shutdown_nodes(self):
        self.launch.shutdown()
        time.sleep(1)

    def spin_for(self, t=1.0):
        start_time = time.time()
        running = True
        while running:
            # split this into two loops so don't spend too much time in loop header (premature opt)
            # for i in range(10):
                # running = self.launch.runner.spin_once()
                # if not running:
                    # return False
            running = self.launch.runner.spin_once()
            if time.time() - start_time > t:
                return running
            time.sleep(0.2) # otherwise the launcher will take up wayy to much CPU..

        return running

    def spin_until_finished(self, max_timeout):
        start_time = time.time()
        
        still_running = self.spin_for(max_timeout)
        if still_running:
            raise Exception("Launch not finished executing after {0} seconds".format(max_timeout))


    def do_service_call(self, service_point, msg):
        service = rospy.ServiceProxy(service_point, type(msg)) 
        service(msg)

    def start_track_path(self):
        start_track = rospy.ServiceProxy('/VehicleController/begin_path_tracking', SetBool)
        try:
            response = start_track(True)
            print(response)
            return True
        except Exception as e:
            print(e)
            return False
   
    def set_model_state(self, model_state_msg):
        if type(model_state_msg) != SetModelStateRequest:
            print("Cannot set model state without SetModelState message type")
            return False
        self.set_model_state_proxy(model_state_msg)



    def set_a_model_state(self, model_name, position, numpy_orientation_quat, linear_vel=np.array([0.0, 0, 0]), angular_vel=np.array([0.0, 0, 0]), reference_frame='map'):
        msg = SetModelStateRequest()
        vals = msg.model_state
        pos, orientation = vals.pose.position, vals.pose.orientation
        pos.x, pos.y, pos.z = position
        orientation.x, orientation.y, orientation.z, orientation.w = numpy_orientation_quat 
        lin_vel, angular = vals.twist.linear, vals.twist.angular
        lin_vel.x, lin_vel.y, lin_vel.z = linear_vel
        angular.x, angular.y, angular.z = angular_vel

        vals.model_name = model_name 
        vals.reference_frame = reference_frame

        self.set_model_state(msg)
