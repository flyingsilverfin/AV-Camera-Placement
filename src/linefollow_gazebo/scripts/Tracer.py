import numpy as np
import helper as h
import abc

class WorldObject(object):
    def __init__(self):
        pass

    @abc.abstractmethod
    def time_to_intersection(self, start, direction):
        pass

class Plane(WorldObject):
    def __init__(self, point, normal):
        self.p = point
        self.n = h.normalize(normal)

    def time_to_intersection(self, start, direction):
        """ Takes a start direction and unit direction vector"""
        t = np.dot(self.p, self.n)
        t -= np.dot(start, self.n)
        t /= np.dot(direction, self.n)
        if t < 0:
            return None
        return t


class Cylinder(WorldObject):
    def __init__(self, point, direction, radius):
        self.p_a = point
        self.v_a = direction
        self.r = radius

    def time_to_intersection(self, start, direction):
        
        # reduces to quadratic equation
        # At^2 + Bt + C = 0

        p_a = self.p_a - start # shift to start = 0, then use my formulas
        q = direction
        v_a = self.v_a
        r = self.r

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
        t0 /= -(2*A)

        t1 = -B - np.sqrt(radical)
        t1 /= -(2*A)

        # return lesser t
        if t0 < 0 and t1 < 0:
            return None
        elif t0 < 0:
            return t1
        elif t1 < 0:
            return t0
        else:
            return min(t0, t1) # first intersection is only one that interests us
