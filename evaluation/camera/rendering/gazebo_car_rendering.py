import os
import glob
import json
import time
import numpy as np
from ...LoggingServer import Logger 
from ...SimulationExperimentRunner import Runner 

from gazebo_msgs.srv import SetModelState
import sys
sys.path.append("/home/joshua/Documents/Uni/Year4/dissertation/catkin_ws/src/linefollow_gazebo/scripts")

import helper

def do_render_capture(vehicle_location, vehicle_heading_deg, save_to, runner, logger):

    orientation_quat = helper.quat_from_rpy(0.0, 0.0, vehicle_heading_deg)
    runner.set_a_model_state('prius', vehicle_location, orientation_quat)
  
    def empty_callback(msg):
        pass

    logger.log_n_messages('/camera/image_raw', save_to, 1, empty_callback)

    # can just block since we need to be synchronous anyway
    while not logger.finished_awaiting_msgs('/camera/image_raw'):
        time.sleep(0.1)

def camera_grid_eval(camera_save_root_dir, runner, logger, counter=0):
    """ This method is different to the corresponding modeled one in that it walks the subdirectories and uses data generated into 'description' to place the vehicle
    
    :param camera_save_root_dir: eg. rendering/gen/grid/camera_blabla
    
    """
    print("****Camera grid eval: {0}".format(camera_save_root_dir))

    for (full_path, subdirs, files) in os.walk(camera_save_root_dir):
        for f in files:
            if f == 'perspective_description.json':
                print("[{0}] Processing: {1}".format(counter, os.path.join(full_path, f)))
                experiment_description = json.load(open(os.path.join(full_path, f)))
                vehicle_loc = experiment_description['world_vehicle_center']
                vehicle_heading = experiment_description['world_vehicle_heading']
                do_render_capture(vehicle_loc, vehicle_heading, full_path, runner, logger)
                counter += 1
    return counter


def process_config(config, save_root_dir, runner, logger):
    print("Processing {0} in gazebo simulation".format(config))

    eval_type = config['type']
    camera_descriptions = config['camera']
    vehicle_desc = config['vehicle']
    
    for desc in camera_descriptions:
      
        # launch nodes camera parameters as required
        res_x, res_y = desc["resolution"]
        fov_w_deg, fov_h_deg =desc["fov_deg"]
        camera_loc = desc["location"]
        orientation_rpy_deg = desc["orientation_rpy_deg"]

    
        launch_params = {'horizontal_fov': str(np.deg2rad(fov_w_deg)),
                         'width_resolution': str(res_x), 
                         'height_resolution': str(res_y),
                         'rviz': "false"}

        runner.launch_nodes(launch_params) 
        time.sleep(6) # sleep seconds to wait for nodes to launch
      
        print("***Launched Nodes***")
        # once the nodes are running, set camera position and orientation
        runner.set_a_model_state('camera', camera_loc, helper.quat_from_rpy(*orientation_rpy_deg))
        print("*** Set camera position!***")

        # compute THIS camera's target save directory
        camera_config_name = "camera_{0}x{1}_{2}-{3}deg_height-{4}_pitch-{5}".format(res_x, res_y, fov_w_deg, fov_h_deg, camera_loc[2], orientation_rpy_deg[1])
        

        if eval_type == 'camera_grid':
            camera_grid_eval(os.path.join(save_root_dir, 'grid', camera_config_name), runner, logger)
        else:
            print("Unimplemented evaluation type: {0}".format(eval_type))





print(__name__)
if __name__ == "__main__":
    pkgdir = os.getcwd()
    script_dir = os.path.join(pkgdir, 'evaluation', 'camera', 'rendering')

    config_files = glob.glob(os.path.join(script_dir, 'defs', '*.json'))

    if len(config_files) == 0:
        print("no config files => assume no 'gen' files to iterate over, exiting")
        sys.exit()

    runner = Runner()
    runner.launch_core()

    # lauch logging node
    logger = Logger()

    try:
        for config_name in config_files:
            with open(config_name) as config_file:
                config = json.load(config_file)
                process_config(config, save_root_dir=os.path.join(script_dir, 'gen'), runner=runner, logger=logger) 
    except Exception:
        runner.shutdown_core()

