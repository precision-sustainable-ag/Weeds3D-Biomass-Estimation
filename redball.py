from circle import Circle
import numpy as np
import point

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

def interpolate_red_ball(rb1: RedBall, rb2: RedBall):
    new_center = point.midpoint_between_points(rb1.ball.center, rb2.ball.center)
    new_radius = int((rb1.ball.radius+rb2.ball.radius)/2)
    new_elevation = (rb1.elevation + rb2.elevation)/2
    new_ball = RedBall(Circle(new_center, new_radius))
    new_ball.ball_id = rb1.ball_id
    new_ball.elevation = new_elevation
    return new_ball

