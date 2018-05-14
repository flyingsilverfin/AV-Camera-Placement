
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
    axes.quiver(X, Y, Z, U, V, W, cmap=color)

def plot_points(axes, points, color='blue', size=2):
    if points.shape[0] == 0:
        return
    axes.scatter(points[:, 0], points[:, 1], points[:, 2], color=color, s=size) 

# some traffic camera information
# https://www.lumenera.com/media/wysiwyg/documents/casestudies/selecting-the-right-traffic-camera-solution-sheet.pdf
class Camera(object):
    def __init__(self, position, orientation_pitch_deg=0.0, orientation_yaw_deg=0.0, model='perspective', verbose=False):
        
        self.model = model 
        self.verbose = verbose

        self.position = np.array(position)
        self.orientation_rpy = np.deg2rad(np.array([-90.0, orientation_pitch_deg, orientation_yaw_deg]))
        self.orientation = transforms.quaternion_from_euler(*self.orientation_rpy)
        self.generate_transform()

        # initialize to some defaults
        self.set_resolution(setup=False)
        self.set_focal_length()
        self.set_fov(setup=False)
        self.set_target_plane()

        self.original_plane = None
        self.cached_plane_points = np.ones(shape=(self.R_y, self.R_x, 3))*-1

        self.original_cylinder = None

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


        self.plane_camera_center = self.pixel_to_plane(self.R_x/2.0, self.R_y/2.0)
        self.plane_camera_center_right_edge = self.pixel_to_plane(self.R_x, self.R_y/2.0)

        # one_up = self.pixel_to_plane(self.R_x/2.0, self.R_y/2.0 + 1)
        # one_up_right = self.pixel_to_plane(self.R_x/2.0 + 1.0, self.R_y/2.0 + + 1)

        # dx_lower = np.linalg.norm(self.plane_camera_center_right_edge - self.plane_camera_center) / (self.R_x/2.0)
        # dx_higher = np.linalg.norm(one_up_right - one_up)
        # self.dx_growth_per_y_pixel = dx_higher - dx_lower
        # # self.dx_growth_per_y_pixel = np.linalg.norm(one_up_right - one_up) - np.linalg.norm(self.plane_camera_center_right_edge - self.plane_camera_center)/(self.R_x/2.0)

        # self.plane_camera_bottom_middle = self.pixel_to_plane(self.R_x/2.0, 0)
        # self.plane_camera_bottom_right = self.pixel_to_plane(self.R_x, 0)
        
        # # # used for perspective camera
        # self.bottom_dx = np.linalg.norm(self.plane_camera_bottom_right - self.plane_camera_bottom_middle)/ ( self.R_x/2.0)
        # middle_dx = np.linalg.norm(self.plane_camera_center_right_edge - self.plane_camera_center) / (self.R_x/2.0)
        # #calculate increase in ground plane pixel width per increasing y pixel
        # self.dx_growth_per_y_pixel = (middle_dx - self.bottom_dx)/(self.R_y/2.0)

        self.cached_plane_points = np.ones(shape=(self.R_y, self.R_x, 3))*-1


    def _get_ground_plane_dx_at_y_pixel(self, y_pix):
        p1, p2 = self.pixel_to_plane(0, y_pix), self.pixel_to_plane(1, y_pix)
        return np.linalg.norm(p2 - p1)
        # return y_pix * self.dx_growth_per_y_pixel + self.bottom_dx


    def _compute_pixel_plane_boundaries(self):
        # pre-compute the x and y bounds of the scaled camera pixel plane
        self.max_x_pixel_plane = self.r(self.w_fov/2.0)
        self.max_y_pixel_plane = self.r(self.h_fov/2.0)


    def set_fov(self, horizontal_deg=45.0, vertical_deg=45.0, setup=True):
        # TODO these /2 are here because I dropped a /2 in the math somewhere...

        self.w_fov = np.deg2rad(horizontal_deg)
        self.h_fov = np.deg2rad(vertical_deg)
        self._compute_pixel_plane_boundaries()

        if setup:
            self.setup()

    def set_resolution(self, h=300, w=400, setup=True):
        self.R_y = h
        self.R_x = w

        if setup:
            self.setup()

    
    # 100mm focal length found online for a traffic camera somewhere
    def set_focal_length(self, f=0.1):
        self.f = f

    def set_position(self, position):
        self.position = position
        self.setup()


    def set_orientation(self, orientation_pitch_deg=0.0, orientation_yaw_deg=0.0):

        self.orientation_rpy = np.deg2rad(np.array([-90.0, orientation_pitch_deg, orientation_yaw_deg]))
        self.orientation = transforms.quaternion_from_euler(*self.orientation_rpy)
        self.setup()
       
    def generate_transform(self):
        """ Creates translation and rotation to move world such that camera is at (0,0,0) and looking at (0,0,1)
        """
   
        target_position = np.array([0.0, 0.0, 0.0])
        self.translation = target_position - self.position
        if self.verbose:
            print("World -> camera translation: {0}".format(self.translation))

        # in camera space, we want everything z-axis aligned ie. rotated 1,0,0 -> 0,0,1
        target_quaternion = np.array([0.0, 1.0, 0.0, -1.0]) # needs to be normalized
        target_quaternion = normalize(target_quaternion)

        #self.rotation = quat_inv_mult(target_quaternion, self.orientation) 
        self.rotation = quat_inv_mult(self.orientation, target_quaternion)
        self.inv_rotation =  transforms.quaternion_conjugate(self.rotation)
        # print("World -> camera rotation: {0}".format(self.rotation))
        # print("Camera -> world rotation: {0}".format(self.inv_rotation)) 

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


    def attach_cylinder(self, start_point, direction_vector, radius):
        vec = normalize(direction_vector)

        self.original_cylinder = {
            'v_a': direction_vector,
            'p_a': start_point,
            'r': radius
        }

        self._transform_cylinder()

    def _transform_cylinder(self):
        if self.original_cylinder is None or self.rotation is None:
            return

        # need to transform start_point into camera coords, then rotate direction vec
        p_a = self.transform_to_camera_space(self.original_cylinder['p_a'])
        v_a = quat_mult_point(self.rotation, self.original_cylinder['v_a'])
        self.target_cylinder = {
            'v_a': v_a,
            'p_a': p_a,
            'r': self.original_cylinder['r']
        }

    def get_cylinder_intersection(self, ray):
        """ Computes time of intersection from origin along `ray` to cylinder """
        # reduces to quadratic equation
        # At^2 + Bt + C = 0

        v_a = self.target_cylinder['v_a'] # cylinder axis vector
        p_a = -self.target_cylinder['p_a'] # cylinder start vector
        r = self.target_cylinder['r']
        q = ray

        dp_va_q = np.dot(v_a, q)
        dp_va_va = np.dot(v_a, v_a)
        dp_va_pa = np.dot(v_a, p_a)

        A = np.dot(q, q) - 2*dp_va_q**2 + dp_va_va*dp_va_q**2 
        B = 2 * (-np.dot(q, p_a) + 2*dp_va_pa * dp_va_q - dp_va_pa*dp_va_q*dp_va_va)
        C = np.dot(p_a, p_a) - 2*dp_va_pa**2 + dp_va_va*dp_va_pa**2 - r**2

        radical = B**2 - 4*A*C
        if radical < 0:
            # imaginary, no solutions
            return None

        t0 = -B + np.sqrt(radical)
        t0 /= (2*A*C)

        t1 = -B - np.sqrt(radical)
        t1 /= (2*A*C)

        print (t0, t1)
        # return lesser t
        if t0 < 0 and t1 < 0:
            return None
        elif t0 < 0:
            return t1
        elif t1 < 0:
            return t0
        else:
            return min(t0, t1) # first intersection is only one that interests us


    def r(self, theta):
        """ Theta = angle to optical axis. Converts this angle into a distance from optical axis in pixel plane
        Different amounts of bend depending on camera model """

        if self.model == 'perspective':
            if np.abs(theta) > np.pi/2 and self.verbose:
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


    def _get_r_phi(self, x, y):
        # apply center shift
        x_ctr, y_ctr = x - self.R_x/2.0, self.R_y/2.0 - y
        # rescale based on pixels width in pixel plane
        x_width, y_width = self.max_x_pixel_plane/(self.R_x/2.0), self.max_y_pixel_plane/(self.R_y/2.0)
        x_scaled, y_scaled = x_ctr * x_width, y_ctr * y_width
        # compute r, phi, theta
        r = np.sqrt(x_scaled**2.0 + y_scaled**2.0)
        phi = np.arctan2(y_scaled, x_scaled)
        return (r, phi)

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

        # NOTE the cache is stored y,x matching pixel coordinate conventions
        if type(x) == int and type(y) == int \
                and x >= 0 and x < self.R_x \
                and y >= 0 and y < self.R_y and \
                not np.array_equal(self.cached_plane_points[y, x], np.array([-1, -1, -1])): 
            # save recomputing loads of the same values
            return self.cached_plane_points[y,x]

        # ray going from (0,0,0) through image plane pixel position of (x,y)
        ray_vec = self._get_pixel_ray(x, y)

        # get intersection with pixel plane
        t = np.dot(self.target_plane['a'], self.target_plane['n'])
        t /= np.dot(self.target_plane['n'], ray_vec)



        if t < 0:
            return None

        # get intersection with cylinder if there is one
        if self.original_cylinder is not None:
            t_cyl = self.get_cylinder_intersection(ray_vec)

            if t_cyl is not None and t_cyl < t:
                # if intersect cylinder, no intersection with plane possible!
                return None

        target_plane_point = t * ray_vec

        # apply inverse transforms

        point = self.transform_from_camera_space(target_plane_point) 
       
        if type(x) == int and type(y) == int \
                and x >= 0 and x < self.R_x \
                and y >= 0 and y < self.R_y: 
            self.cached_plane_points[y, x] = point

        return point 
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


    def _plane_area_of_pixel_crossprod(self, x, y):
        # get three ground points: (x,y), (x+1, y), (x, y+1) and compute difference vectors
        c = self.pixel_to_plane(x,y)
        dx = self.pixel_to_plane(x+1, y) 
        dy = self.pixel_to_plane(x, y+1) 
        if c is None or dx is None or dy is None:
            return None

        dx -= c
        dy -= c
        # area of parallelogram is |cross product|
        cross = np.cross(dx, dy)
        ground_area = np.linalg.norm(cross)
        return ground_area

    def _plane_area_of_pixel_analytic(self, pixel_x, pixel_y):

        if self.model != 'perspective':
            if self.verbose:
                print("WARN: analytic does not work for non-perspective cameras")
            return -1

        """
        For perspective cameras:
        We compute the length of the diagonal of a pixel area
        using only basic measures: ground distance, 
        height of camera mount, and the change in angle corresponding to the diagonal of the pixel
        """


        p_centered = x_ctr, y_ctr = np.array([pixel_x - self.R_x/2.0, self.R_y/2.0 - pixel_y])
        x_width, y_width = self.max_x_pixel_plane/(self.R_x/2.0), self.max_y_pixel_plane/(self.R_y/2.0)
        p_scaled = x_scaled, y_scaled = np.array([x_ctr* x_width, y_ctr * y_width])

        camera_height = self.position[2]

        # only handles cameras at 0,0,z right now! TODO
        plane_point = x, y, z = self.pixel_to_plane(pixel_x, pixel_y)
        ground_dist = np.linalg.norm([x,y])
       
        r1 = np.linalg.norm(p_scaled)
        
        dr = np.linalg.norm([x_width, y_width])
        r2 = r1 + dr
        theta_1 = self.theta(r=r1)
        theta_2 = self.theta(r=r2)
        tan_d_theta_diag = np.tan(theta_2 - theta_1)

        tan_theta_vertical = ground_dist / camera_height
        length_diagonal = camera_height * (tan_theta_vertical - tan_d_theta_diag) / (1 - tan_theta_vertical * tan_d_theta_diag)
        length_diagonal -= ground_dist


        # a = (camera_height**2) / (camera_height - ground_dist * tan_d_theta_diag)
        # b = (ground_dist/camera_height) + tan_d_theta_diag + tan_d_theta_diag * ground_dist**2 - camera_height * ground_dist
        # length_diagonal = a * b

        
        # also need the base length, which is constant across any given y pixel
        # depends only on the y pixel and camera position/parameters

        # length of  pixel on the ground plane is constant for a  given depth
        # since we've disallowed ROLL
        # so we're just looking at the width of any pixel, projected onto the plane

        # pixel_width = self.pixel_plane_pixel_width
        # camera_to_pixel = np.array([pixel_x, pixel_y, self.f])
        # dist_to_pixel = np.linalg.norm(camera_to_pixel)
        # camera_to_plane_point = self.position - plane_point
        # dist_camera_to_plane_point = np.linalg.norm(camera_to_plane_point)
        # pixel_width_on_plane = dist_camera_to_plane_point * pixel_width / dist_to_pixel # simple ratio to find plane width
        # dx = pixel_width_on_plane
        dx = self._get_ground_plane_dx_at_y_pixel(pixel_y)

        # lastly need the plane angle to the principal point on the plane (ie. intersection of center and plane to the plane point)
        center = self.plane_camera_center # on-ground-plane position of principal vector intersection
        diff_vector = center - plane_point
        dist = np.linalg.norm(diff_vector)
        plane_horizontal_vector = self.plane_camera_center_right_edge - center

        if dist == 0: # we have the center pixel, in which case tha rea is a square with the diagonal given
            print("Center pixel ground diagonal length: {0}".format(length_diagonal))
            return length_diagonal**2 /2
        print("Distance plane_point to center: {0}, plane horizontal vector from principal plane point: {1}".format(dist, plane_horizontal_vector))
        dotprod = np.dot(diff_vector, plane_horizontal_vector) / (dist * np.linalg.norm(plane_horizontal_vector))
        angle = np.arccos(np.abs(dotprod))

        # area of p-gram given base, diagonal, and angle between the two is
        print("Angle: {0}, diagonal length: {1}, pixel ground width: {2}".format(angle, length_diagonal, dx))
        # area = abs(np.sin(angle) * length_diagonal * dx)
        area = abs(length_diagonal*dx)

        return area


        


    def plane_area_of_pixel(self, x, y, method="crossprod"):
        if method == 'crossprod':
            return self._plane_area_of_pixel_crossprod(x, y)
        elif method == 'analytic':
            return self._plane_area_of_pixel_analytic(x, y)
        else:
            return None

    
    # convenience feature for debugging
    def get_corners(self):
        w, h = self.R_x, self.R_y

        corners = []
        corners.append(self.pixel_to_plane(0,0))
        corners.append(self.pixel_to_plane(w, 0))
        corners.append(self.pixel_to_plane(w, h))
        corners.append(self.pixel_to_plane(0, h))

        return corners

    def get_height(self):
        return self.position[2]


    # ------ for evaluation ------


    def get_pixel_probabilities_for_road(self, road):
        
        pixel_probabilities = np.zeros(shape=(self.R_y, self.R_x))

        for (y,x),_ in np.ndenumerate(pixel_probabilities):
            ground_point = self.pixel_to_plane(x, y)
            if ground_point is None:
                pixel_probabilities[y,x] = 0
            else:   
                pixel_probabilities[y,x] = road.get_probability(ground_point)
        return pixel_probabilities

    

if __name__ == "__main__":


    """
    VISUAL TESTS
    """
    
    plot_pixel_areas = True 
    # camera pointing down onto XY plane at Z=5
    camera = Camera(position=np.array([0,0,6]), 
                    orientation_pitch_deg=45.0,
                    orientation_yaw_deg=0.0, 
                    model='perspective'
                    #model='equidistance'
                    # model='stereographic'
                   )
    camera.set_resolution(h=100,w=100)
    camera.set_fov(horizontal_deg=60.0, vertical_deg=60.0)
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

    # camera.set_orientation(60.0)

    # pts_2 = []
    # for (x,y) in zip(xs, ys):
        # pt = camera.pixel_to_plane(x,y)
        # if pt is not None:
            # pts_2.append(pt)

    # camera.set_orientation(45.0)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.set_xlim([-5, 5])
    ax.set_ylim([-5, 5])
    ax.set_zlim([-5, 5])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


    pos = camera.position
    orientation = camera.orientation_vector

    plot_points(ax, np.array([pos]))
    plot_vector(ax, pos, orientation)

    # plot_points(ax, np.array(pts_2))


    camera_sp_orientation = camera.camera_sp_orientation_vector
    #plot_vector(ax, np.array([0,0,0]), camera_sp_orientation)

    # plot_points(ax, corners)
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
    plot_vector(ax, pos, p_0_y)
    plot_vector(ax, pos, p_x_y)
    plot_vector(ax, pos, p_x_0)
    # plot_vector(ax, pos, p_1)
    # plot_vector(ax, pos, p_2)
    # plot_vector(ax, pos, p_3)
    # plot_vector(ax, pos, p_4)



#    plt.scatter(corners[:, 0], corners[:, 1])
    plt.savefig('data/camera_{0}.eps'.format(camera.model))
    plt.show()



    if plot_pixel_areas:
    
    
        xs = np.arange(0, camera.R_x)
        ys = np.arange(0, camera.R_y)
        xs, ys = np.meshgrid(xs,ys)
        areas = np.zeros_like(xs, dtype=np.float64)
        xs_, ys_ = xs.ravel(), ys.ravel()
        print(xs_.shape, xs.shape)
        pixels = zip(xs_, ys_)

        areas_2 = np.zeros_like(xs, dtype=np.float64)
    
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

            area2 = camera.plane_area_of_pixel(*p, method='analytic')
            print("Area of pixel {0}: {1}".format(p, area2))
            areas_2[p[1], p[0]] = area2
    
        # f = open('data/camera_pixel_areas.json','w')
        # j = json.dumps(data)
        # f.write(j)
        # f.close()
    
    #    print(areas)
    
        fig_areas = plt.figure()
        ax_areas = fig_areas.add_subplot('111', projection='3d')
        ax_areas.set_xlabel('X')
        ax_areas.set_ylabel('Y')
        ax_areas.set_zlabel('Z (m^2)')
    
        bottom = np.zeros_like(xs_)
        width = depth = 1
   

        camera_properties = np.array([camera.w_fov, camera.h_fov, camera.orientation_rpy[1], camera.orientation_rpy[2]])
        camera_properties = np.rad2deg(camera_properties)

        ax_areas.bar3d(xs_, ys_, bottom, width, depth, areas.ravel(), shade=True)
        plt.savefig('data/camera_pixel_areas_fov_crossprod-{0:.1f}-{1:.1f}_pitch-{2:.1f}_yaw-{3:.1f}.eps'.format(*camera_properties))
        plt.clf()
        plt.cla()

        fig_areas = plt.figure()
        ax_areas = fig_areas.add_subplot('111', projection='3d')
        ax_areas.set_xlabel('X')
        ax_areas.set_ylabel('Y')
        ax_areas.set_zlabel('Z (m^2)')
        ax_areas.bar3d(xs_, ys_, bottom, width, depth, areas_2.ravel(), shade=True)
        plt.savefig('data/camera_pixel_areas_fov_analytic-{0:.1f}-{1:.1f}_pitch-{2:.1f}_yaw-{3:.1f}.eps'.format(*camera_properties))
        plt.clf()
        plt.cla()
        plt.close()
        #plt.show()

        fig_areas = plt.figure()
        ax_areas = fig_areas.add_subplot('111', projection='3d')
        ax_areas.set_xlabel('X')
        ax_areas.set_ylabel('Y')
        ax_areas.set_zlabel('Z (m^2)')
        residuals = areas_2 - areas
        ax_areas.bar3d(xs_, ys_, bottom, width, depth, np.abs(residuals.ravel()), shade=True)
        plt.savefig('data/camera_pixel_areas_fov_residuals{0:.1f}-{1:.1f}_pitch-{2:.1f}_yaw-{3:.1f}.eps'.format(*camera_properties))
        plt.clf()
        plt.cla()
        plt.close()
   
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

