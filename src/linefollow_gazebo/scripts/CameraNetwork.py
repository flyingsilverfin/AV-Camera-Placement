#!/usr/bin/python

import rospy
import numpy as np
import os
import scipy.stats as st

from Tracer import *
from CameraModel import Camera
from RoadModel import Road
from Path import Path
import helper as h

from custom_messages.msg import CameraUpdate, SimulationDataMsg 
from nav_msgs.msg import Odometry

import cv2 # for drawing world and points

class CameraPlacement(object):

    def __init__(self, id, position, pitch_deg, yaw_deg, model, 
                 resolution, fov, 
                 positional_error_stddev=0.03/2.0,
                 orientation_error_stddev=0.5/2.0
                 ):
        self.id = id
        # this will encompass two Cameras
        # one true positioned
        # one with error

        offset_xyz = np.random.normal(loc=0.0, scale=positional_error_stddev, size=3)
        offset_rp = np.random.normal(loc=0.0, scale=orientation_error_stddev, size=2)

        # represents camera with errors
        self.real_camera = Camera(position=position+offset_xyz, 
                            orientation_pitch_deg=pitch_deg+offset_rp[0], 
                            orientation_yaw_deg=yaw_deg+offset_rp[1],
                            verbose=False,
                            model=model
                           ) 
        # perfectly positioned camera
        self.ideal_camera = Camera(position, pitch_deg, yaw_deg, model=model)

        self.real_camera.set_resolution(*resolution)
        self.real_camera.set_fov(*fov)

        self.ideal_camera.set_resolution(*resolution)
        self.ideal_camera.set_fov(*fov)

    def get_catchment_corners(self):
        corners = np.array(self.real_camera.get_corners())
        corners[:, 2] = 0
        return corners


    def get_camera_height(self):
        return self.ideal_camera.position[2]

    def get_id(self):
        return self.id



class ROSCameraNetwork(object):

    def __init__(self, network, ground_truth_odom_topic, camera_update_topic, update_rate=10.0):
        # how often to check for and possibly send updates at most
        self.update_rate = update_rate
        self.update_period = 1.0/update_rate
        self.last_update = rospy.get_rostime().to_sec()

        self.true_pos_topic = ground_truth_odom_topic
        self.listener = rospy.Subscriber(ground_truth_odom_topic, Odometry, self.update)
        self.camera_update_publisher = rospy.Publisher(camera_update_topic, CameraUpdate, queue_size=10)
        self.seq = 0


        self.error_elipse = rospy.get_param('/camera/use_error_elipse', default=True)
        network.set_use_ellipse(self.error_elipse)
        self.camera_network = network


    def update(self, odom_msg):
        """ Receives base pose ground truth and computes predicted vehicle location and associated error """

        now = rospy.get_rostime().to_sec()

        if now - self.last_update < self.update_period:
            return
        self.last_update = now

        true_position = h.get_as_numpy_position(odom_msg.pose.pose.position)
        
        placements_seeing_point = self.camera_network.check_visibility(true_position)
        
        if len(placements_seeing_point) == 0:
            # no cameras see the vehicle => no updates to send
            return
        # for each of the camera placements that need to send an upate,
        # project true position into real_camera, round to nearest pixel
        # calculate algorithmic pertubation
        # real camera => world pixel
        # get estimate of vehicle position
        
        for placement in placements_seeing_point:
            location, error = self.camera_network.get_location_and_error(placement, true_position)
            self.send_camera_update(placement.get_id(), location, error)
    
    def send_camera_update(self, camera_id, location, error):
        # convert the 99% error radius into a covariance matrix
        # error_radius == 3 stddevs
        # 3s^2 = radius
        # s = sqrt(radius/3)

        covariance = np.diag(np.zeros(2, dtype=np.float64)) # this gets handed off as R_k in the EKF
        if not self.error_elipse:
            variance = (error/3.0)**2
            covariance[0,0] = variance
            covariance[1,1] = variance
        else:
            major, minor, angle = error # these represent approximately 3 stddevs!
            major = major/3.0
            minor = minor/3.0
            maj_sq, minor_sq, cos_a_sq, sin_a_sq = major**2, minor**2, np.cos(angle)**2, np.sin(angle)**2
            varx = maj_sq*cos_a_sq + minor_sq*sin_a_sq
            vary = maj_sq*sin_a_sq + minor_sq*cos_a_sq
            covxy = (maj_sq - minor_sq) * cos_a_sq * sin_a_sq
            covariance[0,0] = varx
            covariance[1,0] = covxy
            covariance[0,1] = covxy
            covariance[1,1] = vary

        msg = CameraUpdate()
        pos = msg.position
        pos.x, pos.y, pos.z = location
        msg.covariance = covariance.ravel().tolist()

        msg.header.seq = self.seq
        self.seq += 1
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "map"

        msg.source_camera_id = camera_id

        self.camera_update_publisher.publish(msg)

class CameraNetwork(object):
    """ Represents a set of cameras, world objects, and tracks
        vehicle position, sending updates on a given topic as required """

    def __init__(self, update_rate=10.0, positional_error_stddev=0.03/2, orient_error_stddev=0.5/2, alg_error=4, error_ellipse=True):

        # world objects to do collisions with
        self.world_objects = []
        
        # cameras and catchment areas
        self.catchment_areas = [] 
        self.catchment_cameras = []
        self.always_project = [] # cameras that have infinite FoV always need to be projected into, no shortcuts!

        # store the error bounds to incorporate into error function
        self.alg_error = 4
        self.pos_error_stddev = positional_error_stddev
        self.orient_error_stddev = orient_error_stddev
        
        self.use_error_ellipse = error_ellipse 
  
    def get_placement_by_id(self, identifier):
        for placement in self.catchment_cameras:
            if placement.get_id() == identifier:
                return placement
        for placement in self.always_project:
            if placement.get_id() == identifier:
                return placement

    def set_use_ellipse(self, ellipse):
       self.use_error_ellipse = ellipse
        

    def add_placement(self, placement):
        # get corners on ground plane
        corners = placement.get_catchment_corners()
        if None in corners:
            self.always_project.append(placement)
            return

        # safe to assume all 4 corners are now valid ground points
        self.catchment_areas.append(corners)
        self.catchment_cameras.append(placement)

    def add_world_object(self, world_object):
        self.world_objects.append(world_object)


    def point_within_catchment(self, point, corners):

        # for now to operate on Z = 0 assumption
        # basic idea: check point is on RHS of all vectors between corners
        n = len(corners) 
        for i in range(n):
            vec = corners[(i+1)%n] - corners[i]
            # so my RHS function is flipped... gives LHS, but ok
            if not h.a_rhs_of_b(vec, point - corners[i]):
                return False
        return True
    
    
    def point_visible_from_camera(self, point, camera_placement):
        
        dest = camera_placement.real_camera.position
        diff = dest - point
        dist_to_camera = np.linalg.norm(diff)

        if dist_to_camera is None or dist_to_camera < 0:
            return False
        
        unit_dir = h.normalize(diff)
    
        for obj in self.world_objects:
            dist_to_object = obj.time_to_intersection(point, unit_dir)
            if dist_to_object is None:
                continue
            if obj.time_to_intersection(point, unit_dir) < dist_to_camera:
                return False
    
        return True
    
    
    def check_visibility(self, point):
      
        visible_from = []
        for placement in self.always_project:
            if self.point_visible_from_camera(point, placement):
                visible_from.append(placement)
    
        for i, corners in enumerate(self.catchment_areas):
            if self.point_within_catchment(point, corners):
                if self.point_visible_from_camera(point, self.catchment_cameras[i]):
                    visible_from.append(self.catchment_cameras[i])
    
        return visible_from


    def get_location_and_error(self, placement, true_world_position, ellipse=True):
        # at this point, guaranteed to have visibility of vehicle (tested before) 
        # 1. project true world position into real camera
        real_camera_pixel = placement.real_camera.world_to_pixel(*true_world_position)
        # 2. Round it to get pixel center/corner
        pixel = np.round(real_camera_pixel)

        # apply algorithmic error
        pixel_move_distance = np.random.normal(loc=0.0, scale=self.alg_error/2.0)
        direction = np.random.uniform(0.0, 2*np.pi)
        dx = pixel_move_distance * np.cos(direction)
        dy = pixel_move_distance * np.sin(direction)
        pixel += np.array([dx, dy])


        # 3. use it to estimate world position
        location = placement.ideal_camera.pixel_to_plane(*pixel)
        area = placement.ideal_camera.plane_area_of_pixel(*pixel)
        ground_dist = np.linalg.norm((location - placement.ideal_camera.position)[:2])
        # calculate error radius
        error_radius = self.get_predicted_error_radius(placement.get_camera_height(), ground_dist, area)
        
        if ellipse:
            # convert error radius into an ellipse!
            # need pixel direction, major axis length
            c = placement.real_camera.pixel_to_plane(*pixel) # target point, should be cached anyway
            dy = placement.real_camera.pixel_to_plane(pixel[0], pixel[1]+1) - c
            # dx = placement.real_camera.pixel_to_plane(pixel[0]+1, pixel[1]) - c
            camera_pos = placement.real_camera.position
            camera_pos[2] = 0.0
            direction, major_axis_length = c - camera_pos, np.linalg.norm(dy)
            minor_axis_length = area / major_axis_length
            # minor_axis_length = np.linalg.norm(dy)
            ratio = minor_axis_length/major_axis_length


            # *** This is often TOO oval and gets severely messed up by time differences in the simulation!
            # => make the ratio half close to a circle
            ratio = ratio + (1.0 - ratio)/2.0

            # convert circular error area into elliptical one of same area
            error_area = np.pi*error_radius**2
            major = np.sqrt(error_area/(ratio*np.pi))
            minor = error_area/(np.pi*major)
           
            # convert direction into an angle!
            angle = np.arctan2(direction[1], direction[0])
            
            return (location, (major, minor, angle))
        else:
            return (location, error_radius)
    

    def get_predicted_error_radius(self,
                                   camera_height,
                                   ground_distance, 
                                   pixel_area,
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
        # as it is is an underestimate. Also not sure how using |abs| affects..
        
    
        stddevs_orientation = st.norm.ppf(orientation_percent)
        stddevs_position = st.norm.ppf(pos_percent)
        stddevs_alg = st.norm.ppf(alg_percent)
        
        xy_error = 2 * stddevs_position * self.pos_error_stddev if use_err_xy else 0
        z_error = (ground_distance/camera_height)*self.pos_error_stddev * self.pos_error_stddev if use_err_z else 0
        
        if use_err_orient:
            t = stddevs_orientation*np.deg2rad(self.orient_error_stddev)
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
        total = xy_error + z_error + orientation_error + (self.alg_error+1)/2.0 * stddevs_alg * pixel_error
        return total



if __name__ == "__main__":

    rospy.init_node("camera_network")

    # 1. get road definition from param server, create paths 
    # 2. set up cameras 
    # 3. create planes and cylinders as required according to segments 
    # NOTE: current restriction: can't have paths curve INTO walls
    # will get blocked!
    # 4. create a publisher topic for camera updates of position
    # 5. create a ground truth pose listener, raytrace back to each camera
    # => if not intersect, apply pertubations etc

    # note: maintains two cameras per intended camera,
    # to simulate positional and orientational errors!

    # visual debugging!!
    # def draw_ellipse(img, center, xaxis, yaxis, rotation_deg, start_angle_deg, end_angle_deg):
        # # 0,0 is height/2, width/2
        # center = np.round(center).astype(np.int64).tolist()
        # center += ctr 
        # cv2.ellipse(img, center, np.array([xaxis, yaxis]), rotation_deg, start_angle_deg, end_angle_deg)

    def draw_line(img, start, end, color=(0,0,0), thickness=1):
        height = img.shape[0]
        start = np.round(start).astype(np.int64).tolist()
        start[0] = height - start[0]
        end = np.round(end).astype(np.int64).tolist()
        end[0] = height - end[0]
        cv2.line(img, tuple(start), tuple(end), color=color, thickness=thickness)

    def draw_circle(img, center, radius, thickness=3, color=(0,0,0)):
        height = img.shape[0]
        center = np.round(center).astype(np.int64).tolist()
        center[0] = height - center[0]
        cv2.circle(img, tuple(center), int(radius), thickness=thickness, color=color)

    resolution = 5 # pixels per meter
    height, width = 200, 200
    image = blank_image = np.ones((height*resolution,width*resolution,3), np.uint8)*255
    ctr = ctr_y, ctr_x = np.array([resolution*height/2.0, resolution*width/2.0]).astype(np.int64)
    blue, green, red, black = (255,0,0), (0,255,0), (0,0,255), (0,0,0)
    # draw some axes for reference
    draw_line(image, [0, ctr_y], [height*resolution, ctr_y], color=black, thickness=2)
    draw_line(image, [-ctr_x, 0], [width*resolution, 0], color=black, thickness=2)

    #TODO come back to this to create graphics!!


    # create a camera network
    # give it various types of error to account for
    errors = rospy.get_param('/cameras/errors')
    alg_error = errors['alg_error']
    pos_error_stddev = errors['pos_error_stddev']
    orient_error_stddev = errors['orient_error_stddev']
    
    # get camera update rate
    update_rate = rospy.get_param('/cameras/update_rate', default=10.0)

    camera_network = CameraNetwork(positional_error_stddev=pos_error_stddev,
                        orient_error_stddev=orient_error_stddev, 
                        alg_error=alg_error)


    # get road definition
    road_width = rospy.get_param('/road/width')
    side_offset = rospy.get_param('/road/side_offset')
    segments = rospy.get_param('/road/path/segments')
    looping = rospy.get_param('/road/path/loop')
    # instantiate path and road
    path = Path(loop=looping)
    for segment in segments:
        curvature = segment["curvature"]
        path.add_segment(curvature=segment["curvature"], length=segment["length"])

    # don't actually need this, mostly need width and side offset...
    # may be used further later though!
    road = Road(path, width=road_width, side_offset=side_offset)
  
   
    # draw path onto image 
    path_points = path.discretize_points() * resolution #scale from meters to resolution*meters
    print(path_points)
    for i in range(path_points.shape[0]-1):
        print("Connecting: {0}, {1}".format(path_points[i,:2], path_points[i+1,:2]))
        draw_line(image, path_points[i,:2], path_points[i+1,:2], color=green)



    for segment in path.segments:
        # create the world objects which define boundaries
        if segment.curvature == 0.0:
            continue # skip, only curved boundaries for now
        midpoint_dist = segment.start_time + segment.get_length()/2.0
        midpoint = segment.point_at(midpoint_dist)
        normal = segment.normal_at(midpoint_dist)

        road_radius = np.abs(1.0/segment.curvature) 
        cylinder_radius = road_radius - road.halfwidth - road.side_offset 
        cylinder_center = midpoint + normal * road_radius
        cylinder_direction = np.array([0.0, 0.0, 1.0])
        
        camera_network.add_world_object(Cylinder(cylinder_center, cylinder_direction, cylinder_radius))
        print("Cylinder center: {0}".format(cylinder_center))
        # draw this onto the image
        draw_circle(image, cylinder_center[:2]*resolution, 2, color=blue)
        draw_circle(image, cylinder_center[:2]*resolution, cylinder_radius*resolution, color=blue)

    
    camera_defs = rospy.get_param('/cameras/placements')
    for i, conf in enumerate(camera_defs):

        placement = CameraPlacement(i, np.array(conf['position']), 
                     conf['pitch_degrees'], 
                     conf['yaw_degrees'], 
                     model=conf['model'], 
                     resolution=conf['resolution'], 
                     fov=conf['fov'],
                     positional_error_stddev=pos_error_stddev,
                     orientation_error_stddev=orient_error_stddev
                 )


        camera_network.add_placement(placement)

        # draw camera outline onto image if possible
        corners = placement.get_catchment_corners()
        if None not in corners:
            corners = corners[:, :2]*resolution
            for i in range(corners.shape[0]):
                draw_line(image, corners[i][:2], corners[(i+1)%corners.shape[0]][:2], color=red)
        
        draw_circle(image, placement.ideal_camera.position[:2]*5, 3, thickness=-1, color=red)

    # save image to out dir
    save_dir = rospy.get_param('/results_dir')
    # TODO make this image work
    # cv2.imwrite(os.path.join(save_dir, "world.png"), image)


    ros_camera_network = ROSCameraNetwork(camera_network,
                                            "/base_pose_ground_truth", 
                                            "/camera_update", 
                                            update_rate=update_rate)

    rospy.spin()

