
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
    def __init__(self, position, orientation_pitch_deg=0.0, orientation_yaw_deg=0.0):
        
        self.position = np.array(position)
        self.orientation_rpy = np.deg2rad(np.array([00.0, orientation_pitch_deg, orientation_yaw_deg]))
        self.orientation = transforms.quaternion_from_euler(*self.orientation_rpy)
        self.generate_transform()

        # initialize to some defaults
        self.set_fov()
        self.set_resolution()
        self.set_focal_length()
        self.set_target_plane()

        self.original_plane = None

        self.setup()

    def setup(self):
        
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


    def set_fov(self, horizontal_deg=45.0, vertical_deg=45.0):
        # TODO these /2 are here because I dropped a /2 in the math somewhere...

        self.w_fov = np.deg2rad(horizontal_deg)/2
        self.h_fov = np.deg2rad(vertical_deg)/2

    def set_resolution(self, h=300, w=400):
        self.R_y = h
        self.R_x = w

    
    # 100mm focal length found online for a traffic camera somewhere
    def set_focal_length(self, f=0.1):
        self.f = f

    def set_position(self, position):
        self.position = position
        self.setup()


    def set_orientation(self, orientation_pitch_deg=0.0, orientation_pitch_yaw=0.0):

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


    def _x_ray(self, theta, phi):
        t = theta# - self.orientation_rpy[1]
        p = phi# - self.orientation_rpy[2]

        return self.f * np.cos(t) * np.sin(p)

    def _y_ray(self, theta, phi):
        t = theta# - self.orientation_rpy[1]
        p = phi# - self.orientation_rpy[2]

        return self.f * np.sin(t) * np.cos(t)

    def _z_ray(self, theta, phi):
        p = phi# - self.orientation_rpy[2]
        return self.f * np.cos(p)
    
    def _get_pixel_ray(self, x, y):
        theta = x * self.w_fov/self.R_x - self.w_fov/2
        phi = self.h_fov/2 - y*self.h_fov/self.R_y

        return normalize(np.array([self._x_ray(theta, phi), self._y_ray(theta, phi), self._z_ray(theta, phi)]))




#    def _x_ray(self, x):
#        return np.tan(self.w_fov) * self.f * -1 * (-1 + 2.0*x/self.R_x)
   
#    def _y_ray(self, y):
#        return np.tan(self.h_fov) * self.f * -1 * ( 1 - 2.0*y/self.R_y)

#    def _get_pixel_ray(self, x, y):
#        return normalize(np.array([ self._x_ray(x), self._y_ray(y), self.f ]))

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

        return point


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
    camera = Camera(position=np.array([0,0,1]), 
                    orientation_pitch_deg=45.0,
                    orientation_yaw_deg=45.0
                   )
    camera.set_resolution(h=150,w=150)
    camera.set_fov(horizontal_deg=135.0, vertical_deg=135.0)
    camera.set_focal_length(0.1)


    # plane with normal pointing up along Z axis
    camera.set_target_plane(normal=np.array([0.0, 0.0, 1.0]),
                            point = np.array([1.0, 1.0, 0.0])
                           )


    corners = np.array(camera.get_corners())

    print(corners)

    corners = np.array([c for c in corners if c is not None])

    others_x = np.arange(0, camera.R_x, 10)
    others_y = np.arange(0, camera.R_y, 10)
    xs, ys = np.meshgrid(others_x, others_y)
    xs, ys = xs.ravel(), ys.ravel()
    pts = []
    for x,y in zip(xs, ys):
        pts.append(camera.pixel_to_plane(x, y))




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



#    plt.scatter(corners[:, 0], corners[:, 1])
    plt.show()



    if plot_pixel_areas:
    
    
        xs = np.arange(0, camera.R_x-1)
        ys = np.arange(0, camera.R_y-1)
        xs, ys = np.meshgrid(xs,ys)
        areas = np.zeros_like(xs, dtype=np.float64)
        xs, ys = xs.ravel(), ys.ravel()
        pixels = zip(xs, ys)
    
        data = {
            'fov': "{0}, {1}".format(camera.h_fov, camera.w_fov),
            'resolution': "{0}, {1}".format(camera.R_x, camera.R_y),
            'orientation_rpy': "{}".format(camera.orientation_rpy),
            'position': "{}".format(camera.position),
            'pixels_to_area': {}
        }
    
        print(pixels)
        for p in pixels:
            pix_str = str(list(p))
            area = camera.plane_area_of_pixel(*p)
            data['pixels_to_area'][pix_str] = area 
        #    print(p)
            areas[p[1], p[0]] = area 
    
        f = open('data/camera_pixel_areas.json','w')
        j = json.dumps(data)
        f.write(j)
        f.close()
    
    #    print(areas)
    
        fig_areas = plt.figure()
        ax_areas = fig_areas.add_subplot('111', projection='3d')
        ax_areas.set_xlabel('X')
        ax_areas.set_ylabel('Y')
        ax_areas.set_zlabel('Z (cm^2)')
    
        bottom = np.zeros_like(xs)
        width = depth = 1
    
        ax_areas.bar3d(xs, ys, bottom, width, depth, areas.ravel()*10000, shade=True)
        plt.savefig('data/camera_pixel_areas.eps')
        plt.show()
    

