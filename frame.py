from ball_on_post import BallOnPost
import numpy as np
import cv2


class Frame:
    image: np.array
    markers: list
    elevation_angle: int

    def __init__(self, image):
        self.image = image
        self.markers = []
        self.elevation_angle = None

    def get_np_shape(self):
        return np.shape(self.image)

    def get_xy_shape(self):
        return np.shape(self.image)[1], np.shape(self.image)[0]

    def convert_to_contour(self, points):
        length = len(points)
        cnt = np.zeros([length, 1, 2])
        for i in range(length):
            cnt[i, 0, :] = points[i]
        return np.round(cnt).astype('int')

    def find_red_balls_in_image(self):
        pass

    def extract_red_ball_from_meta(self, meta):
        for shape in meta["shapes"]:
            if shape["label"] == "redball":
                cnt = self.convert_to_contour(shape["points"])
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)
                self.markers.append(BallOnPost(center, radius))

    def extract_meta(self, meta):
        self.extract_red_ball_from_meta(meta)