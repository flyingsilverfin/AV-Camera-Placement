"""

Evaluation code for camera model

should be run with 
`python -m evaluation.camera.rendering.camera_model_rendering_eval`


"""

import glob
import os
import scipy.misc
import numpy as np
import json

from skimage.draw import line_aa

import sys
sys.path.append("/home/joshua/Documents/Uni/Year4/dissertation/catkin_ws/src/linefollow_gazebo/scripts")

import helper as helper
import CameraModel as CameraModel

   

def do_eval(real_position, 
            real_orientation, 
            sigma_pos=0.03/2.0,
            sigma_orient=(0.5/2)*np.pi/180,
            alg_error=0.0,
            camera_res=(500,500),
            camera_fov=(60.0, 60.0),
            n_samples=10000):
    pass


if __name__ == "__main__":
    curdir = os.path.join(os.getcwd(), 'evaluation', 'camera', 'rendering')
    config_files = glob.glob(os.path.join(curdir, 'defs', '*.json'))
    print(config_files)
    for config_name in config_files:
        with open(config_name) as config_file:
            config = json.load(config_file)
            process_config(config, curdir, recompute=False)

