from shared.point import Point, Point3D
import numpy as np
from scipy.spatial.transform import Rotation as R

class Line3D:
    point3D: Point3D
    vector3D: Point3D

    def __init__(self, point3D, vector3D, isPositive=False):
        self.point3D = Point3D(point3D)
        self.vector3D = Point3D(vector3D)
        if (self.vector3D.z < 0 and isPositive):
            self.vector3D.x *= -1
            self.vector3D.y *= -1
            self.vector3D.z *= -1

    def rotate(self, rotvec):
        r = R.from_rotvec(rotvec)
        point = self.point3D.get_np()
        vec = self.vector3D.get_np()
        self.point3D.set_np(r.apply(point))
        self.vector3D.set_np(r.apply(vec))

    def translate(self, translate_vector):
        self.point3D.add_np(translate_vector)

    def scale(self, scale_factor):
        self.point3D.scale(scale_factor)

    def get_xyz(self):
        return self.point3D.x, self.point3D.y, self.point3D.z

    def get_vec(self):
        return self.vector3D.x, self.vector3D.y, self.vector3D.z


class Line:
    m: float
    c: float

    def __init__(self, p1: Point = None, p2: Point = None, m: float = None, c: float = None):
        self.m = m if m is not None else self.calculate_gradient(p1, p2)
        self.c = c if c is not None else self.calculate_y_intercept(p1)

    def calculate_gradient(self, p1, p2):
        return (p2.y - p1.y) / (p2.x - p1.x)


    def calculate_y_intercept(self, p1):
        return p1.y - self.m * p1.x

    def fx(self, x):
        return self.m * x + self.c

    def fy(self, y):
        return (y-self.c)/self.m

def fit_line(x_list: list, y_list: list):
    x = np.array(x_list)
    y = np.array(y_list)
    m, c = np.polyfit(x, y, 1)
    line = Line(m=m, c=c)
    return line
