from math import sqrt

class Point:
    x: int
    y: int

    def __init__(self, x, y):
        self.x = x
        self.y = y


def distance_between_points(p1: Point, p2: Point):
    x_diff = p1.x - p2.x
    y_diff = p1.y - p2.y
    return sqrt(x_diff * x_diff + y_diff * y_diff)