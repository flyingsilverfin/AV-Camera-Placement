
import json
import numpy as np
import tf_conversions

from helper import *

transforms = tf_conversions.transformations

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
def plot_vector(axes, start, direction, color='Blues'):
    X, Y, Z = start
    U, V, W =  direction
    ax.quiver(X, Y, Z, U, V, W, cmap=color)

def plot_points(axes, points, color='blue', size=2):
    if points.shape[0] == 0:
        return
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color=color, s=size) 

# some traffic camera information
# https://www.lumenera.com/media/wysiwyg/documents/casestudies/selecting-the-right-traffic-camera-solution-sheet.pdf
class Camera(object):
    def __init__(self, position, orientation_pitch_deg=0.0, orientation_yaw_deg=0.0, model='perspective'):
        
        self.model = model 

        self.position = np.array(position)
        self.orientation_rpy = np.deg2rad(np.array([-90.0, orientation_pitch_deg, orientation_yaw_deg]))
        self.orientation = transforms.quaternion_from_euler(*self.orientation_rpy)
        self.generate_transform()

        # initialize to some defaults
        self.set_focal_length()
        self.set_fov()
        self.set_resolution()
        self.set_target_plane()

        self.original_plane = None

        self.setup()

    def setup(self):
        self._compute_pixel_plane_boundaries()
        
        # focal-axis aligned vector
        self.orientation_vector = quat_mult_point(self.orientation, np.array([1.0, 0.0, 0.0]))
    
        # create new translation and rotation
        self.generate_transform()

        self.camera_sp_orientation_vector = quat_mult_point(self.rotation, self.orientation_vector)
        desired_camera_orientation = np.array([0.0, 0.0, 1.0])
        
        # it must be true that we are aligned with the Z-axis after rotating into camera space
        assert np.allclose(self.camera_sp_orientation_vector, desired_camera_orientation)

        # update target plane transformation
        self.transform_target_plane()


    def _compute_pixel_plane_boundaries(self):
        # pre-compute the x and y bounds of the scaled camera pixel plane
        self.max_x_pixel_plane = self.r(self.w_fov/2.0)
        self.max_y_pixel_plane = self.r(self.h_fov/2.0)

    def set_fov(self, horizontal_deg=45.0, vertical_deg=45.0):
        # TODO these /2 are here because I dropped a /2 in the math somewhere...

        self.w_fov = np.deg2rad(horizontal_deg)
        self.h_fov = np.deg2rad(vertical_deg)
        self._compute_pixel_plane_boundaries()

    def set_resolution(self, h=300, w=400):
        self.R_y = h
        self.R_x = w

    
    # 100mm focal length found online for a traffic camera somewhere
    def set_focal_length(self, f=0.1):
        self.f = f

    def set_position(self, position):
        self.position = position
        self.setup()


    def set_orientation(self, orientation_pitch_deg=0.0, orientation_yaw_deg=0.0):

        self.orientation_rpy = np.deg2rad(np.array([00.0, orientation_pitch_deg, orientation_yaw_deg]))
        self.orientation = transforms.quaternion_from_euler(*self.orientation_rpy)
        self.setup()
       
    def generate_transform(self):
        """ Creates translation and rotation to move world such that camera is at (0,0,0) and looking at (0,0,1)
        """
   
        target_position = np.array([0.0, 0.0, 0.0])
        self.translation = target_position - self.position
        print("World -> camera translation: {0}".format(self.translation))

        # in camera space, we want everything z-axis aligned ie. rotated 1,0,0 -> 0,0,1
        target_quaternion = np.array([0.0, 1.0, 0.0, -1.0]) # needs to be normalized
        target_quaternion = normalize(target_quaternion)

        #self.rotation = quat_inv_mult(target_quaternion, self.orientation) 
        self.rotation = quat_inv_mult(self.orientation, target_quaternion)
        self.inv_rotation =  transforms.quaternion_conjugate(self.rotation)
        print("World -> camera rotation: {0}".format(self.rotation))
        print("Camera -> world rotation: {0}".format(self.inv_rotation)) 

    def transform_to_camera_space(self, vector):
        v = vector + self.translation
        return quat_mult_point(self.rotation, v)

    def transform_from_camera_space(self, vector):
        r = quat_mult_point(self.inv_rotation, vector)
        return r - self.translation


    def set_target_plane(self, normal=np.array([0.0, 0.0, 1.0]), point=np.array([1.0, 1.0, 0.0])):
        """ Saves a target plane that will be intersected with and computed over, transforms into camera space """
        if np.array_equal(point, np.array([0.0, 0.0, 0.0])):
            print("Use a different plane point than 0,0,0")
            return


        self.original_plane = {
            'n': normal,
            'a': point
        }

        self.transform_target_plane()

    def transform_target_plane(self):
   
        if self.original_plane is None or self.rotation is None:
            return

        # normal doesn't need to be translated, but it needs to be rotated
        normal = quat_mult_point(self.rotation, self.original_plane['n'])

        # point needs to be rotated, then translated
        point = self.transform_to_camera_space(self.original_plane['a'])
        self.target_plane = {
            'n': normal,
            'a': point
        }



    # subsume all previous approaches under one mathematically unified implementation

    def r(self, theta):
        if self.model == 'perspective':
            if np.abs(theta) > np.pi/2:
                print("WARNING: attempting to use perspective model for angles greater than 90 degrees!")
            return self.f * np.tan(theta)
        elif self.model == 'equidistance':
            return self.f * theta
        elif self.model == 'stereographic':
            return 2*self.f*np.tan(theta/2.0)
        elif self.model == 'equisolid':
            return 2*self.f*np.sin(theta/2.0)
        else:
            raise Exception("Unknown camera model: {}".format(self.model))

    def theta(self, r):
        if self.model == 'perspective':
            return np.arctan2(r, self.f)
        elif self.model == 'equidistance':
            return r / self.f
        elif self.model == 'stereographic':
            return 2*np.arctan2(r, 2*self.f)
        elif self.model == 'equisolid':
            return 2*np.arcsin(r/2.0*self.f)
        else:
            raise Exception("Unknown camera model: {}".format(self.model))


    def _get_pixel_ray(self, x, y):
        """
        1. convert x,y into real pixel-plane coordinate using height and width of pixel plane
        2. compute r = sqrt(x'**2 + y'**2)
        3. compute phi = np.arctan2(y', x')
        4. compute theta = self.theta(r)
        5. at this point we have spherical angles about the Z axis, theta is from +Z axis, phi is from x=0
        6. convert spherical angles into a ray
        """
        
        # apply center shift
        x_ctr, y_ctr = x - self.R_x/2.0, self.R_y/2.0 - y
        # rescale based on pixels width in pixel plane
        x_width, y_width = self.max_x_pixel_plane/(self.R_x/2.0), self.max_y_pixel_plane/(self.R_y/2.0)
        x_scaled, y_scaled = x_ctr * x_width, y_ctr * y_width
        # compute r, phi, theta
        r = np.sqrt(x_scaled**2.0 + y_scaled**2.0)
        phi = np.arctan2(y_scaled, x_scaled)
        theta = self.theta(r) # outgoing angle depends on camera model

        # phi, theta define spherical polar angles, theta is from +Z axis, phi is from x=0 CCW
        x_ray, y_ray, z_ray = self._x_ray(theta, phi), self._y_ray(theta, phi), self._z_ray(theta, phi)
        return normalize(np.array([x_ray, y_ray, z_ray]))

    # theta is from +Z axis, phi is from +X axis CCW
    def _x_ray(self, theta, phi):
        return np.cos(phi) * np.sin(theta)

    def _y_ray(self, theta, phi):
        return np.sin(phi) * np.sin(theta)

    def _z_ray(self, theta, phi):
        return np.cos(theta)

    def pixel_to_plane(self, x, y, verbose=False):
        if (x < 0 or x >= self.R_x) and verbose:
            print("WARN: x pixel {0} is out of pixel space of [0, {1})".format(x, self.R_x))
        if (y < 0 or y >= self.R_y) and verbose:
            print("WARN: y pixel {0} is out of pixel space of [0, {1})".format(y, self.R_y))

        # ray going from (0,0,0) through image plane pixel position of (x,y)
        ray_vec = self._get_pixel_ray(x, y)

        t = np.dot(self.target_plane['a'], self.target_plane['n'])
        t /= np.dot(self.target_plane['n'], ray_vec)

        if t < 0:
            return None

        target_plane_point = t * ray_vec

        # apply inverse transforms
        return self.transform_from_camera_space(target_plane_point)
#        un_translated = target_plane_point + self.translation
#        point = quat_mult_point(self.inv_rotation, un_translated)


    def world_to_pixel(self, x, y, z):
        """ Computes inverse of pixel_to_plane """
        # get ray from camera origin to world coordinate
        ray = normalize(self.transform_to_camera_space(np.array([x,y,z])))

        # get angle to optical axis
        theta = np.arccos(np.dot(np.array([0, 0, 1.0]), ray))
        # get phi angle
        phi = np.arctan2(ray[1], ray[0])
        
        # compute r, which combined with phi gives x,y on pixel coordinate plane
        r = self.r(theta)
        x, y = r * np.cos(phi), r * np.sin(phi)
        
        #scale these by pixel size to find incident pixel
        x_pix, y_pix = x / (self.max_x_pixel_plane/(self.R_x/2.0)), y/(self.max_y_pixel_plane/(self.R_y/2.0))

        # shift these different origin
        x_pix = np.round(x_pix + self.R_x/2.0)
        y_pix = np.round(self.R_y/2.0 - y_pix) 

        return (x_pix, y_pix)



    def plane_area_of_pixel(self, x, y):
        # get three ground points: (x,y), (x+1, y), (x, y+1) and compute difference vectors
        c = self.pixel_to_plane(x,y)
        dx = self.pixel_to_plane(x+1, y) 
        dy = self.pixel_to_plane(x, y-1) 
        if c is None:
            return None

        dx -= c
        dy -= c
        # area of parallelogram is |cross product|
        cross = np.cross(dx, dy)
        ground_area = np.linalg.norm(cross)
        return ground_area


    
    # convenience feature for debugging
    def get_corners(self):
        w, h = self.R_x, self.R_y

        corners = []
        corners.append(self.pixel_to_plane(0,0))
        corners.append(self.pixel_to_plane(w, 0))
        corners.append(self.pixel_to_plane(w, h))
        corners.append(self.pixel_to_plane(0, h))

        return corners


if __name__ == "__main__":


    """
    VISUAL TESTS
    """
    
    plot_pixel_areas = False
    # camera pointing down onto XY plane at Z=5
    camera = Camera(position=np.array([0,0,6]), 
                    orientation_pitch_deg=90.0,
                    orientation_yaw_deg=0.0, 
                    model='perspective'
                    #model='equidistance'
                    #model='stereographic'
                   )
    camera.set_resolution(h=100,w=100)
    camera.set_fov(horizontal_deg=80.0, vertical_deg=50.0)
    camera.set_focal_length(0.1)


    # plane with normal pointing up along Z axis
    camera.set_target_plane(normal=np.array([0.0, 0.0, 1.0]),
                            point = np.array([1.0, 1.0, 0.0])
                           )


    # TEST: Pixel => Plane => Pixel ray trace should be very close to the same!!
    corners = np.array([[0.0, 0], [0, camera.R_y], [camera.R_x, camera.R_y], [camera.R_x, 0], [camera.R_x/2.0, camera.R_y/2.0]])
    print("\n\n Corners: {0}".format(corners))
    plane_corners = np.array([camera.pixel_to_plane(*pix) for pix in corners])
    print("Corners mapped to plane: {0}".format(plane_corners))
    pixel_corners = np.array([camera.world_to_pixel(*coord) for coord in plane_corners])
    print("Mapped back to pixels: {0}".format(pixel_corners))
    print("All close: {0}\n\n".format(np.allclose(corners, pixel_corners)))


    corners = np.array(camera.get_corners())

    print(corners)

    corners = np.array([c for c in corners if c is not None])

    others_x = np.arange(0, camera.R_x - 1, 5)
    others_y = np.arange(0, camera.R_y - 1, 5)
    xs, ys = np.meshgrid(others_x, others_y)
    xs, ys = xs.ravel(), ys.ravel()
    pts = []
    for x,y in zip(xs, ys):
        pt = camera.pixel_to_plane(x, y)
        if pt is not None:
            pts.append(pt)




    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([-5, 5])
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')


    pos = camera.position
    orientation = camera.orientation_vector

    plot_points(ax, np.array([pos]))
    plot_vector(ax, pos, orientation)


    camera_sp_orientation = camera.camera_sp_orientation_vector
    #plot_vector(ax, np.array([0,0,0]), camera_sp_orientation)

    plot_points(ax, corners)
    plot_points(ax, np.array(pts))

    a = camera.original_plane['a']
    n = camera.original_plane['n']
    #plot_vector(ax, a, n) 


    cam_sp_a = camera.target_plane['a']
    cam_sp_n = camera.target_plane['n']
    #plot_vector(ax, cam_sp_a, cam_sp_n)


    pix_0_0_ray = camera._get_pixel_ray(0.0, 0.0)
    x,y = camera.R_x, camera.R_y
    pix_0_Ry_ray = camera._get_pixel_ray(0.0, y)
    pix_Rx_0_ray = camera._get_pixel_ray(x, 0.0)
    pix_Rx_Ry_ray = camera._get_pixel_ray(x, y)
#    plot_vector(ax, np.array([0,0,0]), pix_0_0_ray)
#    plot_vector(ax, np.array([0,0,0]), pix_0_Ry_ray)
#    plot_vector(ax, np.array([0,0,0]), pix_Rx_0_ray)
#    plot_vector(ax, np.array([0,0,0]), pix_Rx_Ry_ray)

    p_1 = quat_mult_point(camera.inv_rotation, camera._get_pixel_ray(0.0, camera.R_y/2))
    p_2 = quat_mult_point(camera.inv_rotation, camera._get_pixel_ray(camera.R_x/2.0, 0.0))
    p_3 = quat_mult_point(camera.inv_rotation, camera._get_pixel_ray(camera.R_x, camera.R_y/2.0))
    p_4 = quat_mult_point(camera.inv_rotation, camera._get_pixel_ray(camera.R_x/2.0, camera.R_y))
   
    cam_sp_angle1 = np.dot(pix_0_0_ray, pix_Rx_0_ray)
    cam_sp_angle2 = np.dot(pix_0_0_ray, pix_0_Ry_ray)
    cam_sp_angle3 = np.dot(pix_0_Ry_ray, pix_Rx_Ry_ray)
    print("{0}, {1}, {2}, {3}".format(pix_0_0_ray, pix_0_Ry_ray, pix_Rx_Ry_ray, pix_Rx_0_ray))
    print("Cam space: Angle (0,0) -> (Rx, 0): {0}, (0,0)->(0,Ry): {1}, (0, Ry)->(Rx, Ry): {2}".format(cam_sp_angle1, cam_sp_angle2, cam_sp_angle3 ))

    p_0_0 = quat_mult_point(camera.inv_rotation, pix_0_0_ray)
    p_x_y = quat_mult_point(camera.inv_rotation, pix_Rx_Ry_ray)
    p_0_y = quat_mult_point(camera.inv_rotation, pix_0_Ry_ray)
    p_x_0 = quat_mult_point(camera.inv_rotation, pix_Rx_0_ray)
    angle1 = np.dot(p_0_0, p_x_0)
    angle2 = np.dot(p_0_0, p_0_y)
    angle3 = np.dot(p_0_y, p_x_y)
    print("{0}, {1}, {2}, {3}".format(p_0_0, p_0_y, p_x_y, p_x_0))
    print("Angle (0,0) -> (Rx, 0): {0}, (0,0)->(0,Ry): {1}, (0, Ry)->(Rx, Ry): {2}".format(angle1, angle2, angle3))
    plot_vector(ax, pos, p_0_0)
    plot_vector(ax, pos, p_0_y*2)
    plot_vector(ax, pos, p_x_y*3)
    plot_vector(ax, pos, p_x_0*4)
    plot_vector(ax, pos, p_1)
    plot_vector(ax, pos, p_2)
    plot_vector(ax, pos, p_3)
    plot_vector(ax, pos, p_4)



#    plt.scatter(corners[:, 0], corners[:, 1])
    plt.show()



    if plot_pixel_areas:
    
    
        xs = np.arange(0, camera.R_x)
        ys = np.arange(0, camera.R_y)
        xs, ys = np.meshgrid(xs,ys)
        areas = np.zeros_like(xs, dtype=np.float64)
        xs_, ys_ = xs.ravel(), ys.ravel()
        print(xs_.shape, xs.shape)
        pixels = zip(xs_, ys_)
    
        data = {
            'fov': "{0}, {1}".format(camera.h_fov, camera.w_fov),
            'resolution': "{0}, {1}".format(camera.R_x, camera.R_y),
            'orientation_rpy': "{}".format(camera.orientation_rpy),
            'position': "{}".format(camera.position),
            'pixels_to_area': {}
        }
    
        for p in pixels:
            pix_str = str(list(p))
            area = camera.plane_area_of_pixel(*p)
            data['pixels_to_area'][pix_str] = area 
        #    print(p)
            areas[p[1], p[0]] = area 
    
        # f = open('data/camera_pixel_areas.json','w')
        # j = json.dumps(data)
        # f.write(j)
        # f.close()
    
    #    print(areas)
    
        fig_areas = plt.figure()
        ax_areas = fig_areas.add_subplot('111', projection='3d')
        ax_areas.set_xlabel('X')
        ax_areas.set_ylabel('Y')
        ax_areas.set_zlabel('Z (cm^2)')
    
        bottom = np.zeros_like(xs_)
        width = depth = 1
   
        areas *= 10000 # convert to cm^2 from m^2

        camera_properties = np.array([camera.w_fov, camera.h_fov, camera.orientation_rpy[1], camera.orientation_rpy[2]])
        camera_properties = np.rad2deg(camera_properties)

        ax_areas.bar3d(xs_, ys_, bottom, width, depth, areas.ravel(), shade=True)
        plt.savefig('data/camera_pixel_areas_fov-{0:.1f}-{1:.1f}_pitch-{2:.1f}_yaw-{3:.1f}.eps'.format(*camera_properties))
        plt.clf()
        plt.cla()
        plt.close()
        #plt.show()
   
        m, median = np.min(areas), np.median(areas)
        contour_steps = 3.0
        step = (median - m)/contour_steps
        contour_values = np.arange(m, np.max(areas), step)

        contours = plt.contour(xs, ys, areas, levels=contour_values)
        plt.xlabel("X pixel")
        plt.ylabel("Y pixel")
        plt.clabel(contours, inline=1, fontsize=10)
        plt.savefig('data/camera_pixel_area_contours_fov-{0:.1f}-{1:.1f}_pitch-{2:.1f}_yaw-{3:.1f}.eps'.format(*camera_properties))
        #plt.show()

