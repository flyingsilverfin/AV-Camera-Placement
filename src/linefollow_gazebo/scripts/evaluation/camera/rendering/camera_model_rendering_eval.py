import glob
import os
import scipy.misc
import numpy as np
import json

from skimage.draw import line_aa

import helper as helper
import CameraModel as CameraModel

curdir = os.getcwd()
rendering_eval_dir = os.path.join(curdir, 'evaluation', 'camera', 'rendering')
config_files = glob.glob(os.path.join(rendering_eval_dir, 'defs', '*.json'))

def place_vehicle_box(position, heading_from_x_axis, length=5.0, width=2.0, height=2.0):
    """ Places a vehicle modeled as a box as 8 points in the world coordinate system about a position and orientation in degrees"""
    quat = helper.quat_from_rpy(0.0, 0.0, heading_from_x_axis)
    
    # define 8 points as points of the rectangular prism
    l, w, h  = length/2.0, width/2.0, height
    base_points = np.array([[-l, -w, 0], [-l, w, 0], [l, w, 0], [l, -w, 0]])
    top_points = base_points + np.array([0,0, h])

    # rotate to match heading
    points = [helper.quat_mult_point(quat, pt) for pt in base_points]
    points += [helper.quat_mult_point(quat, pt) for pt in top_points]

    # shift into place
    points += position

    return np.array(points)


def camera_grid_generator(camera, vehicle_params):
    """ Takes an instantiated camera, its definition, and vehicle grid placement parameters and generates world center and orientation of a vehicle"""
    start_x, start_y = 0.0, 0.0
    end_x, end_y = camera.R_x, camera.R_y
    step_x, step_y = vehicle_params["x_pixel_step"], vehicle_params["y_pixel_step"]
    start_heading = 0.0
    orientation_step = vehicle_params["orientation_step_deg"]

    # create grid of pixels to project through
    xs, ys = np.meshgrid(np.arange(start_x, end_x, step_x), np.arange(start_y, end_y, step_y))
    pairs = np.vstack([xs.ravel(), ys.ravel()]).T
    counter = 0
    for (x_pix, y_pix) in pairs:
        plane_point = camera.pixel_to_plane(x_pix, y_pix) # car center point
        for orientation_heading in np.arange(0.0, 360.0, orientation_step):
            yield (counter, plane_point, orientation_heading)
        counter += 1

def draw_square(image, pixel, size=9):
    half = int(size/2.0)
    height = image.shape[0]
    # image[height - (pixel[1]-half) : height - (pixel[1]+half), pixel[0]-half:pixel[0]+half] = 0
    image[height-pixel[1]-half: height-pixel[1]+half, pixel[0]-half:pixel[0]+half] = 0


def connect_line(image, start, end, val=0):
    height, width = image.shape
    cc, rr, _ = line_aa(start[0], height-start[1], end[0], height-end[1])
    pixels = (rr >= 0) & (rr < height)
    pixels = pixels & (cc >= 0) & (cc < width)

    rr, cc = rr[pixels], cc[pixels]
    image[rr, cc] = val


def connect_pixels(image, pixels):
    # connect top and bottom edges
    connect_line(image, pixels[0], pixels[1])
    connect_line(image, pixels[1], pixels[2])
    connect_line(image, pixels[2], pixels[3])
    connect_line(image, pixels[3], pixels[0])
    connect_line(image, pixels[4], pixels[5])
    connect_line(image, pixels[5], pixels[6])
    connect_line(image, pixels[6], pixels[7])
    connect_line(image, pixels[7], pixels[4])

    # connect top and bottom corners
    connect_line(image, pixels[0], pixels[4])
    connect_line(image, pixels[1], pixels[5])
    connect_line(image, pixels[2], pixels[6])
    connect_line(image, pixels[3], pixels[7])

    # draw X on bottom face
    connect_line(image, pixels[0], pixels[2])
    connect_line(image, pixels[1], pixels[3])


def calculate_center(pixels):
    # calculates bounding box about the pixels
    # and takes the center as an approximation
    # to the true vehicle center including Z offset

    min_x, min_y = np.min(pixels, axis=0)
    max_x, max_y = np.max(pixels, axis=0)
    center = np.round([(min_x + max_x)/2.0, (min_y + max_y)/2.0]).astype(np.int64)

    return center

def apply_adjustment(camera, vehicle_center_pixel, vehicle_height, true_vehicle_center):
    """ Returns unadjusted and adjusted ground centers of the vehicle"""

    # project center pixel onto ground
    rough_vehicle_center = camera.pixel_to_plane(*vehicle_center_pixel)

    #rough_vehicle_center = camera.pixel_to_plane(*camera.world_to_pixel(*true_vehicle_center))

    # height of camera position
    camera_position = camera.position
    camera_height = camera_position[2]

    # get angle from Z axis (ie one camera is on)
    dist = np.linalg.norm(rough_vehicle_center[:2]) # ground plane distance
    direction_vec = helper.normalize(rough_vehicle_center)
    # overshoot = (vehicle_height/2)/(camera_height/dist)
    overshoot = dist*vehicle_height/(camera_height*2)
    adjusted_center_distance = dist - overshoot

    ground_plane_direction_vector = direction_vec.copy()
    ground_plane_direction_vector[2] = 0.0 # flatten onto ground plane
    ground_plane_direction_vector = helper.normalize(ground_plane_direction_vector)
    adjusted_center = ground_plane_direction_vector * adjusted_center_distance

    return (rough_vehicle_center, adjusted_center, overshoot) 

    
    


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

    vehicle_sizes = vehicle_grid_params["vehicle_box_model_sizes"]

    for model in model_types:

        camera = CameraModel.Camera(camera_loc, orientation_rpy_deg[1], orientation_rpy_deg[2], model=model)
        camera.set_fov(horizontal_deg=fov[0], vertical_deg=fov[1])
        camera.set_resolution(h=res_y, w=res_x)
        print(camera.w_fov, camera.h_fov)

        for (counter, vehicle_center, vehicle_orientation) in camera_grid_generator(camera, vehicle_grid_params):
            for size in vehicle_sizes:
                vehicle_outline = place_vehicle_box(vehicle_center, vehicle_orientation, length=size[0], width=size[1], height=size[2])
                # print("Counter: {0}, Pixel Vehicle Center: {1}".format(counter, vehicle_center))
                image = np.ones((res_y, res_x), dtype=np.uint8)*255

                pixels = []
                for world_point in vehicle_outline:
                    pixels.append(np.round(camera.world_to_pixel(*world_point)).astype(np.int64))
                pixels = np.array(pixels)


                calculated_center = calculate_center(pixels)
                # print("pixels: {0}, center: {1}".format(pixels, calculated_center))
                # print("ORIENTATION: {0}".format(vehicle_orientation))
                connect_pixels(image, pixels)


                draw_square(image, calculated_center, size=11)

                rough_center, adjusted_center, overshoot = apply_adjustment(camera, calculated_center, size[2], vehicle_center + np.array([0,0.0, size[2]/2.0]))
                rough_distance = np.linalg.norm(vehicle_center - rough_center)
                adjusted_distance = np.linalg.norm(vehicle_center - adjusted_center)

                description = {
                    "vehicle_size_string": "{0}m_{1}m_{2}m".format(size[0], size[1], size[2]),
                    "world_vehicle_center": vehicle_center.tolist(),
                    "world_vehicle_heading": vehicle_orientation, 
                    "camera": camera_description,
                    "vehicle_world_outline": vehicle_outline.tolist(),
                    "vehicle_camera_outline": pixels.tolist(),
                    "localization": {
                        "bounding_box_center_pixel": calculated_center.tolist(),
                        "rough_vehicle_center": rough_center.tolist(),
                        "adjusted_vehicle_center": adjusted_center.tolist(),
                        "rough_center_error": rough_distance,
                        "adjusted_center_error": adjusted_distance, 
                        "rough_center_overshoot": overshoot
                    }

                }

                target_dir = os.path.join(rendering_eval_dir, 'gen', 'grid', description["vehicle_size_string"], "{0}_heading_{1}".format(counter, vehicle_orientation))

                if not os.path.exists(target_dir):
                    try:
                        os.makedirs(target_dir)
                    except OSError as exc: # Guard against race condition
                        if exc.errno != errno.EEXIST:
                            raise OSError

                filename = os.path.join(target_dir, "{0}.png".format(camera.model))
                scipy.misc.imsave(filename, image)

                json_desc_file = os.path.join(target_dir, "{0}_description.json".format(model))
                with open(json_desc_file, 'w') as outfile:
                    json.dump(description, outfile, indent=4, sort_keys=True)



def process_config(config):
    print("Processing {0}".format(config))

    eval_type = config['type']
    camera_descriptions = config['camera']
    vehicle_desc = config['vehicle']
    for desc in camera_descriptions:
        if eval_type == 'camera_grid':
            vehicle_grid_params = vehicle_desc[eval_type]
            camera_grid_eval(eval_type, desc, vehicle_grid_params)

    


for config_name in config_files:
    with open(config_name) as config_file:
        config = json.load(config_file)
        process_config(config)

