
from ..helper import quaternion_from_rpy, quat_mult_point
from ..CameraModel import Camera

def place_vehicle_box(position, heading_from_x_axis, length=5.0, width=2.0, height=2.0):
    """ Could be more efficient using matrices but not used too heavily anyway """
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

    return points


