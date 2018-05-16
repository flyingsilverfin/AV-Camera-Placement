import time
import os
import multiprocessing
import subprocess
import argparse
import json
import time
import numpy as np
import tf_conversions
from ..LoggingServer import Logger 
from ..SimulationExperimentRunner import Runner 

from custom_messages.msg import SimulationDataMsg, CameraUpdate

def point_to_numpy(point):
    return np.array([point.x, point.y, point.z])

def quat_to_numpy(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    numpy_quat = np.array([x, y, z, w])
    return numpy_quat

def vec3_to_numpy(vec):
    return np.array([vec.x, vec.y, vec.z])

# def dfs_loader(path, config, runner):

    # for key in config:
        # child = config[key]
        # if key.endswith('_') or (type(child) != dict and type(child) != list):
            # name = '/'.join(path) + '/' + key
            # runner.set_param(name, child)
        # else:
            # dfs_loader(path + [key], child,runner)


def init_params(config, simulation_runner):
    for key in config:
        simulation_runner.set_param(key, config[key]) # addressable as /.../... by default! yay ROS

def run_n_times(runner, repeats, save_dir, rviz):

    def receive_sim_data(msg):
        time = msg.header.stamp.to_sec()

        true_odom = msg.true_odom
        true_position = point_to_numpy(true_odom.pose.pose.position)
        true_quat = quat_to_numpy(true_odom.pose.pose.orientation)
        true_linear_vel = vec3_to_numpy(true_odom.twist.twist.linear)
        true_angular_vel = vec3_to_numpy(true_odom.twist.twist.angular)
        true_heading = tf_conversions.transformations.euler_from_quaternion(true_quat)[2] # in radians

        ekf_state = list(msg.ekf_state)
        ekf_cov = list(msg.ekf_cov)

        sim_data_log.append({
            't': time,
            'true': {
                'pos': true_position.tolist(),
                'quat': true_quat.tolist(),
                'linear_vel': true_linear_vel.tolist(),
                'angular_vel': true_angular_vel.tolist(),
                'heading_radians': true_heading,
            },
            'ekf_state': ekf_state,
            'ekf_cov': ekf_cov
            })


    def receive_camera_update(msg):
        time = msg.header.stamp.to_sec()
        position = point_to_numpy(msg.position)
        covariance = np.array(msg.covariance)
        camera_updates_log.append({
            't': time,
            'pos': position.tolist(),
            'cov': covariance.tolist(),
            'camera_id': msg.source_camera_id
            })



    for n in range(repeats):
        sim_data_log = []
        camera_updates_log = []

        this_save_dir = os.path.join(save_dir, "run_{0}".format(n))
        try:
            os.makedirs(this_save_dir)
        except OSError as e:
            print(e)
        
        # need to put absolute path into param server
        cwd = os.getcwd()
        runner.set_param("/results_dir", os.path.join(cwd, this_save_dir))
        
        rviz_string = "true" if rviz else "false"
        runner.launch_nodes({'rviz': rviz_string}) 

        logger = Logger()
        # attach two listeners
        # tbh could just init this as a node but probably cleaner this way
        
        time.sleep(12) # sleep seconds to wait for nodes to launch
        logger.log_n_messages('/simulation_data', this_save_dir, -1, receive_sim_data)
        logger.log_n_messages('/camera_update', this_save_dir, -1, receive_camera_update)
        runner.start_track_path()
        runner.spin_until_finished()

        # should finish and shutdown nodes automatically
        # and go onto next loop!

        print("Finished execution run {0}, writing logs".format(n))
        with open(os.path.join(this_save_dir, "sim_data.json"), 'w') as f:
            json.dump(sim_data_log, f, indent=4)
        with open(os.path.join(this_save_dir, "camera_data.json"), 'w') as f:
            json.dump(camera_updates_log, f, indent=4)

         


if __name__ == "__main__":


    parser = argparse.ArgumentParser(description='Set up and launch single instances of simulation, with a specified config file')

    parser.add_argument("--port", required=True, type=int, help="Port to launch master and roscore on")
    parser.add_argument("--config", required=True, help="Config file (or folder if implemented) that defines experiment to launch")
    parser.add_argument("--repeats", type=int, help="Number of times to perform the experiment", required=True)
    parser.add_argument("--out", required=True, help="Directory to save simulation results/data to")

    args = parser.parse_args()
    
    port = args.port
    repeats = args.repeats

    print(port)
    config_file = args.config
    # open and parse the JSON config file
    config = json.load(open(config_file)) 

    save_dir = args.out
    try:
        os.makedirs(save_dir)
    except OSError as e:
        print(e)


    # instantiate a simulation runner
    sim_runner = Runner(port=port)
    sim_runner.launch_core()
    init_params(config, sim_runner)
    run_n_times(sim_runner, repeats, save_dir, rviz=config['rviz'])

    sim_runner.shutdown_core()

