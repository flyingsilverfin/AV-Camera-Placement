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
import analysis

from custom_messages.msg import SimulationDataMsg, CameraUpdate

def point_to_numpy(point):
    return np.array([point.x, point.y, point.z])

def quat_to_numpy(quat):
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    numpy_quat = np.array([x, y, z, w])
    return numpy_quat

def vec3_to_numpy(vec):
    return np.array([vec.x, vec.y, vec.z])

def init_params(config, simulation_runner):
    for key in config:
        simulation_runner.set_param(key, config[key]) # addressable as /.../... by default! yay ROS

def run_n_times(runner, repeats, save_dir, rviz, max_timeout=120, continue_exec=True):

    count = 0
    last_num = 0
    if continue_exec:
        # get the last number already executed in `this_save_dir`
        dirs = os.listdir(save_dir)
        nums = [int(x.split("_")[-1]) for x in dirs if os.path.isdir(os.path.join(save_dir,x))]
        nums.sort()
        if len(nums) > 0:
            last_num = nums[-1]+1

    while count < repeats:
        sim_data_log = []
        camera_updates_log = []

        this_save_dir = os.path.join(save_dir, "run_{0}".format(count + last_num))
        try:
            os.makedirs(this_save_dir)
        except OSError as e:
            print(e)
        
        # need to put absolute path into param server
        cwd = os.getcwd()
        runner.set_param("/results_dir", os.path.join(cwd, this_save_dir))
        
        rviz_string = "true" if rviz else "false"
        runner.launch_nodes({'rviz': rviz_string}) 
        print("rviz string: " + rviz_string)

        logger = Logger()
        # attach two listeners
        # tbh could just init this as a node but probably cleaner this way
        
        # keep trying to start the path tracking, minimized wait time
        start_time = time.time()
        time.sleep(5)
        try:
            runner.set_a_model_state('prius', np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0, 1.0]), reference_frame='map')

            while not runner.start_track_path():
                time.sleep(0.5)

            rosbag_file_path = os.path.join(this_save_dir, "sim_data.bag")
            logger.rosbag_topic('/simulation_data', rosbag_file_path, msg_type=SimulationDataMsg)
            runner.spin_until_finished(max_timeout=max_timeout)
        except Exception as e: # some other error...
            # also retry the loop
            print("Retrying loop: exception raised: {0}".format(e))
            if '/simulation_data' in logger.active_logs:
                logger.close_rosbag('/simulation_data')
            runner.shutdown_nodes()
            continue

        # should finish and shutdown nodes automatically
        # and go onto next loop!
        logger.close_rosbag('/simulation_data')

        print("Finished execution run {0} and writing bagfile to {1}".format(count, rosbag_file_path))
        count += 1
         


if __name__ == "__main__":


    parser = argparse.ArgumentParser(description='Set up and launch single instances of simulation, with a specified config file')

    parser.add_argument("--port", required=True, type=int, help="Port to launch master and roscore on")
    parser.add_argument("--config", required=True, help="Config file (or folder if implemented) that defines experiment to launch")
    parser.add_argument("--repeats", type=int, help="Number of times to perform the experiment", required=True)
    parser.add_argument("--out", required=True, help="Directory to save simulation results/data to")
    parser.add_argument('--rviz', dest='rviz', action='store_true')
    parser.add_argument('--no-rviz', dest='rviz', action='store_false')
    parser.add_argument('--continue', dest='continue_exec', action='store_true')
    parser.add_argument('--restart', dest='continue_exec', action='store_false')
    parser.add_argument('--compute_MI', dest='compute_MI', action='store_true')
    parser.add_argument('--no-compute_MI', dest='compute_MI', action='store_false')

    parser.set_defaults(compute_MI=True)
    parser.set_defaults(rviz=True)
    parser.set_defaults(continue_exec=True) # by default, don't overwrite existing runs

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

    config['rviz'] = args.rviz
    print("Single runner RVIZ: {0}".format(config['rviz']))

    if 'max_timeout' in config:
        timeout = config['max_timeout']
    else:
        timeout = 120.0

    # instantiate a simulation runner
    sim_runner = Runner(port=port)
    sim_runner.launch_core()
    init_params(config, sim_runner)
    run_n_times(sim_runner, repeats, save_dir, rviz=config['rviz'], max_timeout=timeout, continue_exec=args.continue_exec)

    sim_runner.shutdown_core()

    print("\n\nBeginning bagfile analysis for all bagfiles below: {0}".format(save_dir))

    # run analysis over the produced .bag files!
    # get all produced bagfiles for this runner
    bagfiles = analysis.get_all_bagfiles_below(save_dir)
    print("found bagfiles: {0}".format(bagfiles))
    definition = config
    max_steps = definition['analysis']['max_steps']
    metrics = analysis.compute_metrics(definition, bagfiles, max_steps, compute_MI=args.compute_MI) 
   
    
    # need to convert bigfloat mutual information into something JSON serializable
    # can be deseralized from strings again via bf.BigFloat()
    if args.compute_MI:
        metrics['mean mutual inf']['mutual inf'] = str(metrics['mean mutual inf']['mutual inf'])
        metrics['mean mutual inf']['variance'] = str(metrics['mean mutual inf']['variance'])

    with open(os.path.join(save_dir, "summary.json"), 'w') as f:
        json.dump(metrics, f)

