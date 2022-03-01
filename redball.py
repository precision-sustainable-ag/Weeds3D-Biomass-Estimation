from circle import Circle
import numpy as np

class RedBall:
    ball: Circle
    ball_id: int
    elevation: int

    def __init__(self, ball: Circle, image_height: int = None):
        self.ball = ball
        self.ball_id = None
        self.elevation = None
        if image_height is not None:
            self.calculate_elevation(image_height)

    # Calculate relative elevation between -1 and 1. -1 is bottom of image, +1 is top of image.
    def calculate_elevation(self, image_height):
        image_center_y = image_height/2
        x, y = self.ball.get_center()
        self.elevation = 1 - 2*y/image_height

    def __lt__(self, other):
        return self.elevation < other.elevation

    def __le__(self, other):
        return self.elevation <= other.elevation

    def __gt__(self, other):
        return self.elevation > other.elevation

    def __ge__(self, other):
        return self.elevation >= other.elevation

