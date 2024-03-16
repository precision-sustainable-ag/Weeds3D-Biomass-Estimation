from math import acos, sqrt, pi
from shared.point import Point
from shared import point
import cv2

class Circle:
    center: Point
    radius: int

    def __init__(self, center: Point, radius: int):
        self.center = center
        self.radius = radius

    def get_center(self):
        return (self.center.x, self.center.y)

    def get_area(self):
        return pi*self.radius*self.radius

    def get_circle_info(self):
        return "center: [" + str(self.center.x) + "," + str(self.center.y) + "], radius: " + str(self.radius)

def approx_same(c1: Circle, c2: Circle, percent: float = 0.05):
    r_perc = c1.radius/c2.radius
    dist = point.distance_between_points(c1.center, c2.center)
    dist_perc = dist/c1.radius
    return True if r_perc > 1-percent and r_perc < 1+percent and dist_perc < 2*percent else False

def min_enclosing_circle(cnt):
    (x, y), radius = cv2.minEnclosingCircle(cnt)
    center = Point(int(x), int(y))
    radius = int(radius)
    return Circle(center, radius)

def half_circle_overlap_area(r, d_part):
    return r*r*acos(float(d_part)/float(r)) - d_part*sqrt(r*r - d_part*d_part)

# Using formula from https://mathworld.wolfram.com/Circle-CircleIntersection.html
def are_two_circles_overlapped(c1: Circle, c2: Circle, iou_cutoff: float):
    d = point.distance_between_points(c1.center, c2.center)
    # if circles are not overlapping
    if d >= (c1.radius + c2.radius):
        return False

    # if two circles exactly overlap
    if d == 0 and c1.radius == c2.radius:
        return True

    # if one circle inside the other
    if d < abs(c1.radius - c2.radius):
        a1 = c1.get_area()
        a2 = c2.get_area()
        iou = a1/a2 if a1 < a2 else a2/a1
        return True if iou > iou_cutoff else False

    # if circles are overlapping
    x = (d * d - (c2.radius * c2.radius) + c1.radius * c1.radius) / (2 * d)
    intersection_area = half_circle_overlap_area(c1.radius, x) + half_circle_overlap_area(c2.radius, d - x)
    union_area = c1.get_area() + c2.get_area() - 2*intersection_area
    iou = intersection_area/union_area
    return True if iou > iou_cutoff else False

