from point import Point

class BallOnPost:
    center: Point
    radius: int

    def __init__(self, center: Point, radius: int):
        self.center = center
        self.radius = radius