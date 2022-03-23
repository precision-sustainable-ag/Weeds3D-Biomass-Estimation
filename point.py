from math import sqrt

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