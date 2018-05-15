import rospy
import numpy as np
import scipy.stats as st

from Tracer import *
from CameraModel import Camera
from RoadModel import Road
from Path import Path
import helper as h


class CameraPlacement(object):

    def __init__(self, position, pitch_deg, yaw_deg, model, resolution, fov):
        # this will encompass two Cameras
        # one true positioned
        # one with error

        # TODO get the offsets and sample them

        self.real_camera = Camera(position + pos_noise, pitch_deg + pitch_error, yaw_deg + yaw_error, model=model)
        self.ideal_camera = Camera(position, pitch_deg, yaw_deg, model=model)

        self.real_camera.set_resolution(*resolution)
        self.real_camera.set_fov(*fov)

        self.ideal_camera.set_resolution(*resolution)
        self.ideal_camera.set_fov(*fov)

    def get_catchment_corners(self):
        return self.real_camera.get_corners()

    #TODO will need other methods that allow this to have the same
    # interface as a single Camera

    # TODO encompass algorithmic error as well


    def get_location_and_error(self, pixel):

        location = self.ideal_camera.pixel_to_plane(*pixel)
        area = self.ideal_camera.plane_area_of_pixel(*pixel)
        ground_dist = np.linalg.norm((location - self.ideal_camera.position)[:2])

        error_radius = self.get_predicted_error_radius(ground_dist, area)

        return (location, error_radius)







    def get_predicted_error_radius(self,
                                   ground_distance, 
                                   pixel_area,
                                   sigma_pos=0.03/2.0,
                                   sigma_orient_deg=0.5/2,
                                   alg_error=4.0, # note: changed
                                   verbose=False,
                                   orientation_percent=0.9,
                                   pos_percent=0.9,
                                   alg_percent=0.9,
                                   use_err_xy=True,
                                   use_err_z=True,
                                   use_err_orient=True,
                                   use_err_pixel=True
                                  ):
        # TODO this is wrong implementation! Doesn't matter though really as long as use is consistent
        # but really it should be -1*norm.ppf((1-percent)/2.0)
        # as it is is an underestimate. Also not sure how using |abs| affects...

        camera_height = self.ideal_camera.position[2]
    
        stddevs_orientation = st.norm.ppf(orientation_percent)
        stddevs_position = st.norm.ppf(pos_percent)
        stddevs_alg = st.norm.ppf(alg_percent)
        
        xy_error = 2 * stddevs_position * sigma_pos if use_err_xy else 0
        z_error = (ground_distance/camera_height)*stddevs_position*sigma_pos if use_err_z else 0
        
        if use_err_orient:
            t = stddevs_orientation*np.deg2rad(sigma_orient_deg)
            alpha = np.arctan2(ground_distance, camera_height)
            beta = alpha + t
            if beta > np.pi/2: # causes tan to roll over into negatives...
                orientation_error = 9999999999
            else:
                orientation_error = np.tan(beta)*camera_height - ground_distance
        else:
            orientation_error = 0
        
        pixel_error = np.sqrt(2*pixel_area) if use_err_pixel else 0
        
    #     print("Z error: {0}".format(z_error))
    #     print("orien error: {0}".format(orientation_error))
    #     print("pixel_error: {0}".format(pixel_error))
        total = xy_error + z_error + orientation_error + (alg_error+1)/2.0 * stddevs_alg * pixel_error
        return total
    
    


def point_within_catchment(point, corners):
    #TODO determine if a point is contained witin the given 4 corners
    pass


def point_visible_from_camera(point, camera):
    
    dest = camera.position
    diff = dest - point
    dist_to_camera = np.linalg.norm(diff)
    unit_dir = h.normalize(dest - point)

    for obj in world_objects:
        dist_to_object = obj.time_to_intersection(point, unit_dir)
        if dist_to_object is None:
            continue
        if obj.time_to_intersection(point, unit_dir) < dist_to_camera:
            return False

    return True


def check_visibility(point):
  
    visible_from = []
    for camera in always_project:
        if point_visible_from_camera(point, camera):
            visible_from.append(camera)

    for i, corners in enumerate(catchment_areas):
        if point_within_catchment(point, corners):
            if point_visible_from_camera(point, catchment_cameras[i]):
                visible_from.append(catchment_cameras[i])

    return visible_from


if __name__ == "__main__":

    # 1. get road definition from param server, create paths [done]
    # 2. set up cameras [done]
    # 3. create planes and cylinders as required according to segments [done]
    # NOTE: current restriction: can't have paths curve INTO walls
    # will get blocked!
    # 4. create a publisher topic for camera updates of position
    # 5. create a ground truth pose listener, raytrace back to each camera
    # => if not intersect, apply pertubations etc

    # note: maintains two cameras per intended camera,
    # to simulate positional and orientational errors!


    # get road definition
    road_width = rospy.get_param('/road/width')
    side_offset = rospy.get_param('/road/side_offset')
    

    segments = rospy.get_param('/road/path/_segments')

    path = Path(loop=looping)
    for segment in segments:
        curvature = segment["curvature"]
        path.add_segment(curvature=segment["curvature"], length=segment["length"])

    road = Road(path, width=road_width, side_offset=side_offset)
   
    world_objects = []
    for segment in self.path.segments:
        # create the world objects which define boundaries
        if segment.curvature == 0.0:
            continue # skip, only curved boundaries for now
        midpoint_dist = segment.start_time + segment.get_length()/2.0
        midpoint = segment.get_point_at(midpoint_dist)
        normal = segment.get_normal_at(midpoint_dist)

        road_radius = np.abs(1.0/segment.curvature) 
        cylinder_radius = road_radius - road.halfwidth - road.side_offset 
        cylinder_center = midpoint + normal * road_radius
        cylinder_direction = np.array([0.0, 0.0, 1.0])
        
        world_objects.append(Cylinder(cylinder_center, cylinder_direction, cylinder_radius))

    # cameras and catchment areas
    catchment_areas = [] 
    catchment_cameras = []
    always_project = [] # cameras that have infinite FoV always need to be projected into, no shortcuts!
    
    camera_defs = rospy.get_param('/_cameras')
    for conf in camera_defs:

        placement = CameraPlacement(np.array(conf['position']), conf['pitch_degrees'], conf['yaw_degrees'], model=conf['model'], resolution=conf['resolution'], fov=conf['fov'])


        # get corners on ground plane
        corners = placmenet.get_catchment_corners()
        if None in corners:
            always_project.append(placement)
            continue

        # safe to assume all 4 corners are now valid ground points
        corners[:, 2] = 0 # just for niceness to look at
        catchment_areas.append(corners)
        catchment_cameras.append(placement)

    





