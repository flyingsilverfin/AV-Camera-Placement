import numpy as np
import scipy.stats as st

class Road(object):

    def __init__(self, path, probability_distribution, width=3.0, side_offset=1.5):
        """ Define a road with a width, wide of road offset etc.
        
        Default to road width of 3m - common in cities
        Side offest = 1.5m is estimate of pavement width/building offsets
            Used for as a camera placement constraint
        """
        self.path = path

        # width on path
        self.width = width
        self.halfwidth = width/2.0

        # "pavement"
        self.side_offset = side_offset
    
        self.prob_dist = probability_distribution

        self.cameras = []

    def get_allowed_camera_positions_at_distance(self, distance):
        """ Returns positions at the side of the road, plus
        the direction toward the center of the road """
        
        center = self.path.get_point_at(distance)
        normal = self.path.get_normal_at(distance)
        
        # normal is a unit vector
        distance = self.halfwidth + self.side_offset
        left = center - distance*normal
        right = center + distance*normal

        return np.array([[left, normal], [right, -normal]])

    
    def _get_closest_curve_distance(self, point):
        """ computes and returns the `distance` and point corresponding
        to the closest point on the curve to the given world point.
        Should be normal to the tangent of the curve and (point-closest_point)"""

        closest_point_distance, dist, closest_point = -1, 99999999, None
        for segment in self.path.segments:
            # remember, time parametrized at v = 1.0 => time = distance
            segment_start_dist = segment.start_time 
            segment_end_dist = segment.end_time
            horizon = segment_end_dist - segment_start_dist

            closest_dist_on_segment = segment.closest_point_time(point, segment.start_time, horizon)  # search entire segment for closest point
            closest_point_on_segment = segment.point_at(closest_dist_on_segment)
            d = np.linalg.norm(closest_dist_on_segment - point)

            if d < dist:
                closest_point_distance = closest_dist_on_segment
                dist = d
                closest_point = closest_point_on_segment

        return closest_point_distance, closest_point 

    def get_perpendicular_distance_to_curve(self, point):
        path_distance, path_point = self._get_closest_curve_distance(point)
        vec = point - path_point
        dist = np.linalg.norm(vec)
        # sanity check
        # tangent =  self.path.get_tangent_at(path_distance)
        # assert np.dot(tangent, vec)/dist < 0.001, "Curve Tangent is not perpendicular to the (point-curve_point) found"

        return dist 

    def get_probability(self, point):
        perpendicular_curve_distance = self.get_perpendicular_distance_to_curve(point)
        
        prob = self.prob_dist.evaluate_point(perpendicular_curve_distance)
        return prob

    def attach_camera(self, camera):
        camera_position = camera.position
        self.cameras.append(camera)

        # TODO calculate bounding area on ground for this camera
        # although that's entirely an effiency boost




class NormalProbabilityDist(object):
    def __init__(self, two_stddev=1.0, d_stddev=0.01):
        """ Provide the 95% distance """
        self.stddev = two_stddev/2.0
        self.step = -1 * self.stddev * d_stddev # always step down a fraction of the stddev (? or just a step?)

    def evaluate_point(self, value):
        """ Computes an approximate probability of 
        taking the given value according to the distribution.
        uses a small step about that point to evaluate probability.
        d_stddev is the fraction of a standard deviation to step by"""

        # compute on the negative half of the dist
        if value > 0:
            value = value * -1

        cum_prob1 = st.norm.cdf(value, scale=self.stddev)
        cum_prob2 = st.norm.cdf(value + self.step, scale=self.stddev)

        prob = cum_prob1 - cum_prob2 
        return prob


        
