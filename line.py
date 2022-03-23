from point import Point
import numpy as np

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
    m, b = np.polyfit(x, y, 1)
    line = Line(m=m, b=b)
    return line
