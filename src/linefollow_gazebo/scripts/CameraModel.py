
import numpy as np
import tf_conversions

transforms = tf_conversions.transformations

def quat_inv_mult(quat1, quat2):
    """  Returns quaternion transforming quat1 -> quat2 ??

    """

    inv = transforms.quaternion_inverse(quat1)
    return transforms.quaternion_multiply(inv, quat2)


def quat_mult_quat(quat1, quat2):
    return tf_conversions.transformations.quaternion_multiply(quat1, quat2)

def quat_mult_point(quat, vec):
    vec_quat = np.array([vec[0], vec[1], vec[2], 0])
    q_conj = tf_conversions.transformations.quaternion_conjugate(quat)

    # rotation = q*r*q_conj
    a = quat_mult_quat(quat, vec_quat)
    rotated = quat_mult_quat(a, q_conj)
    return rotated[:3]

def normalize(vec):
    return vec/np.linalg.norm(vec)

# some traffic camera information
# https://www.lumenera.com/media/wysiwyg/documents/casestudies/selecting-the-right-traffic-camera-solution-sheet.pdf
class Camera(object):
    def __init__(self, position, orientation_quaternion):
       
        self.position = np.array(position)

        self.orientation = orientation_quaternion


        # initialize to some defaults
        self.set_fov()
        self.set_resolution()
        self.set_focal_length()

        self.original_plane = None

        self.setup()

    def setup(self):
        
        # focal-axis aligned vector
        self.orientation_vector = quat_mult_point(self.orientation, np.array([1.0, 0.0, 0.0]))
    
        # create new translation and rotation
        self.generate_transform()

        # update target plane transformation
        self.transform_target_plane()


    def set_fov(self, horizontal=np.pi/3.5, vertical=np.pi/3):
        self.w_fov = horizontal
        self.h_fov = vertical

    def set_resolution(self, h=300, w=400):
        self.R_y = h
        self.R_x = w

    
    # 100mm focal length found online for a traffic camera somewhere
    def set_focal_length(self, f=0.1):
        self.f = f

    def set_position(self, position):
        self.position = position
        self.setup()


    def set_orientation(self, orientation_quaternion):
        self.orientation_quaternion = orientation_quaternion
        self.setup()

    def generate_transform(self):
        """ Creates translation and rotation to move world such that camera is at (0,0,0) and looking at (0,0,1)
        """
   
        target_position = np.array([0.0, 0.0, 0.0])
        self.translation = target_position - self.position


        target_quaternion = np.array([0.0, 1.0, 0.0, 1.0]) # needs to be normalized
        target_quaternion = normalize(target_quaternion)

        self.rotation = quat_inv_mult(target_quaternion, self.orientation) 
        self.inv_rotation =  transforms.quaternion_inverse(self.rotation)
        

    def set_target_plane(self, normal, point):
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
   
        if self.original_plane is None:
            return

        # normal doesn't need to be translated, but it needs to be rotated
        normal = quat_mult_point(self.rotation, self.original_plane['n'])

        # point needs to be rotated, then translated
        point = quat_mult_point(self.rotation, self.original_plane['a'])
        point = point + self.translation
        self.target_plane = {
            'n': normal,
            'a': point
        }


    def _x_ray(self, x):
        return np.tan(self.w_fov) * (self.f - 2*x/self.R_x)
   
    def _y_ray(self, y):
        return np.tan(self.h_fov) * (self.f - 2*y/self.R_y)


    def pixel_to_plane(self, x, y):
        if x < 0 or x > self.R_x:
            print("x pixel {0} is out of pixel space of [0, {1}]".format(x, self.R_x))
            return None
        if y < 0 or y > self.R_y:
            print("y pixel {0} is out of pixel space of [0, {1}]".format(y, self.R_y))

        # ray going from (0,0,0) through image plane pixel position of (x,y)
        ray_vec = np.array([ self._x_ray(x), self._y_ray(y), self.f ])
        
        pos = np.dot(self.target_plane['a'], self.target_plane['n'])
        pos /= np.dot(self.target_plane['n'], ray_vec)

        target_plane_point = pos * ray_vec

        # apply inverse transforms
        un_translated = target_plane_point - self.translation
       
        point = quat_mult_point(self.inv_rotation, un_translated)

        return point

    
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

    # camera pointing down onto XY plane at Z=5
    camera = Camera(position=np.array([0,0,1]), 
                    orientation_quaternion=normalize(np.array([0.0, 1.0, 0.0, 1.0]))
                   )


    # plane with normal pointing up along Z axis
    camera.set_target_plane(normal=np.array([0.0, 0.0, 1.0]),
                            point = np.array([1.0, 1.0, 0.0])
                           )


    corners = np.array(camera.get_corners())

    import matplotlib.pyplot as plt
    plt.scatter(corners[:, 0], corners[:, 1])
    plt.show()

    
    
