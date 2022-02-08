import numpy as np
import cv2


class BallOnPost:
    center: (int, int)
    radius: int
    gravity_vector: (float, float)
    base_of_post: (int, int)
    biomass_region: list

    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
        self.gravity_vector = None
        self.base_of_post = None
        self.biomass_region = None


class Frame:
    image: np.array
    markers: list
    elevation_angle: int

    def __init__(self, image):
        self.image = image
        self.markers = []
        self.elevation_angle = None

    def get_shape(self):
        return np.shape(self.image)

    def add_test_marker(self, center, radius):
        self.markers.append(BallOnPost(center, radius))

# Video:
#   - video filepath
#   - frames
#   - red ball center frames
class Video:
    filepath: str
    frames: list
    red_ball_center_frames = list

    def __init__(self, filepath):
        self.filepath = filepath
        self.frames = []
        self.red_ball_center_frames = []

    def add_test_frame(self, img):
        self.frames.append(Frame(img))

    def add_test_marker(self, frame_num, center, radius):
        self.frames[frame_num].add_test_marker(center, radius)

    def get_shape(self):
        if len(self.frames) > 0:
            return self.frames[0].get_shape()
        else:
            return 0, 0
        
# Testing video
print("Hello World")
vid = Video(".\\data\\Calib-test.MP4")
print(vid.get_shape())
vid.add_test_frame(np.zeros([2, 2], dtype='uint8'))
vid.add_test_frame(np.ones([2, 2], dtype='uint8'))
print(vid.get_shape())
vid.add_test_marker(0, (0, 0), 1)
vid.add_test_marker(1, (1, 1), 1)
