from point import Point

class Line:
    p1: Point
    p2: Point
    m: float
    c: int

    def __init__(self, p1: Point, p2: Point):
        self.p1 = p1
        self.p2 = p2
        self.m = self.calculate_gradient()
        self.c = self.calculate_y_intercept()

    def __init__(self, p1: Point, m: float, c: int):
        self.p1 = p1
        self.p2 = None
        self.m = m
        self.c = c

    def calculate_gradient(self):
        self.m = (self.p2.y - self.p1.y) / (self.p2.x - self.p1.x)

    def calculate_y_intercept(self):
        self.c = self.p1.y - self.m * self.p1.x