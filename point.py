from math import sqrt
import numpy as np

class Point3D:
    x: float
    y: float
    z: float

    def __init__(self, point3d):
        self.x = point3d[0]
        self.y = point3d[1]
        self.z = point3d[2]

    def get_np(self):
        return np.array([self.x, self.y, self.z])

    def set_np(self, vec):
        self.x = vec[0]
        self.y = vec[1]
        self.z = vec[2]

    def add_np(self, add_vector):
        self.x += add_vector[0]
        self.y += add_vector[1]
        self.z += add_vector[2]

    def scale(self, scale_factor):
        self.x *= scale_factor
        self.y *= scale_factor
        self.z *= scale_factor

class Point:
    x: int
    y: int

    def __init__(self, x, y):
        self.x = x
        self.y = y

    def get_xy_str(self):
        return "("+str(self.x)+","+str(self.y)+")"


def distance_between_points(p1: Point, p2: Point):
    x_diff = p1.x - p2.x
    y_diff = p1.y - p2.y
    return sqrt(x_diff * x_diff + y_diff * y_diff)

def midpoint_between_points(p1: Point, p2: Point):
    new_x = p2.x + (p1.x-p2.x)/2
    new_y = p2.y + (p1.y-p2.y)/2
    return Point(new_x, new_y)