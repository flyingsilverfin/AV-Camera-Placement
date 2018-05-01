import glob
import os
import scipy.misc
import numpy as np

print(__name__)

from ....helper import quaternion_from_rpy, quat_mult_point
from ..CameraModel import CameraModel


def place_vehicle_box(position, heading_from_x_axis, length=5.0, width=2.0, height=2.0):
    """ Places a vehicle modeled as a box as 8 points in the world coordinate system about a position and orientation in degrees"""
    quat = quaternion_from_rpy(0.0, 0.0, heading_from_x_axis)
    
    # define 8 points as points of the rectangular prism
    l, w, h  = length/2.0, width/2.0, height
    base_points = np.array([[-l, -w, 0], [-l, w, 0], [l, w, 0], [l, -w, 0]])
    top_points = base_points + np.array([0,0, h])

    # rotate to match heading
    points = [quat_mult_point(quat, pt) for pt in base_points]
    points += [quat_mult_point(quat, pt) for pt in top_points]

    # shift into place
    points += position

    return np.array(points)


def camera_grid_generator(camera, vehicle_params):
    """ Takes an instantiated camera, its definition, and vehicle grid placement parameters
        and generates world center and orientation of a vehicle"""
    start_x, start_y = 0.0
    end_x, end_y = camera.R_x, camera.R_y
    step_x, step_y = vehicle_params["x_pixel_step"], vehicle_params["y_pixel_step"]
    start_heading = 0.0
    orientation_step = vehicle_params["orientation_step_deg"]

    for size in vehicle_sizes:
        # create grid of pixels to project through
        xs, ys = np.meshgrid(np.arange(start_x, end_x, step_x), np.arange(start_y, end_y, step_y))
        pairs = np.vstack(xs.ravel(), ys.ravel()).T
        counter = 0
        for (x_pix, y_pix) in pairs:
            plane_point = camera.pixel_to_plane(x_pix, y_pix) # car center point
            for orientation_heading in np.arange(0.0, 360.0, orientation_step):
                yield (counter, plane_point, orientation_heading)
            counter += 1



def camera_grid_eval(name, camera_description, vehicle_grid_params, model_types=['perspective', 'stereographic', 'equidistance']):
    # instantiate camera

    # iterate through camera_grid_generator
    # use world point and heading to place vehicle
    # re-project vehicle corners to get pixel plane coorindates (rounded to nearest pixel)
    
    # optional: color in the region? or connect with lines?
    # save pixel plane from camera into correct folder with name 'perspective' or etc.
    # also save a description which is used by simulation to generate renders


    camera_loc = camera_description["location"]
    orientation_rpy_deg = camera_description["orientation_rpy_deg"]
    res_x, res_y = camera_description["resolution"]
    fov = camera_description["fov_deg"]

    vehicle_sizes = vehiddcle_params["vehicle_box_model_sizes"]

    for model in model_types:

        camera = CameraModel(camera_loc, orientation_rpy_deg[1], orientation_rpy_deg[2], model=model)
        
        for (counter, vehicle_center, vehicle_orientation) in camera_grid_generator(camera, vehicle_grid_params):
            for size in vehicle_sizes:
                vehicle_outline = place_vehicle_box(vehicle_center, vehicle_orientation, length=size[0], width=size[1], height=size[2])
                
                image = np.ones((res_y, res_x), dtype=np.uint8)*255

                for world_point in vehicle_outline:
                    pix = camera.world_to_pixel(world_point)
                    image[pix[0], pix[1]] = 0


                description = {
                    "vehicle_size_string": "{0}m_{1}m_{2}m".format(size[0], size[1], size[2]),
                    "world_vehicle_center": vehicle_center,
                    "world_vehicle_heading": vehicle_orientation, 
                    "camera": camera_description,
                    "vehicle_world_outline": list(vehicle_outline)
                }

                target_dir = os.path.join('gen', 'grid', description["vehicle_size_string"], 
                                          "{0}_heading_{1}".format(counter, vehicle_orientation))

                if not os.path.exists(target_dir):
                    try:
                        os.makedirs(target_dir)
                    except OSError as exc: # Guard against race condition
                        if exc.errno != errno.EEXIST:
                            raise OSError

                filename = os.path.join(target_dir, "{0}.png".format(camera.model))
                scipy.misc.imsave(filename, image)

                json_desc_file = os.path.join(target_dir, "description.json")
                with open(json_desc_file, 'w') as outfile:
                    json.dump(description, outfile)



def process_config(config):

    eval_type = config['type']
    camera_descriptions = config['camera']
    vehicle_desc = config['vehicle']
    for desc in camera_descriptions:
        if eval_type == 'camera_grid':
            vehicle_grid_params = vehicle_desc[eval_type]
            camera_grid_eval(eval_type, desc, vehicle_grid_params)

    


config_files = glob.glob('defs/*.json')

for config in config_files:
    process_config(config)

