import numpy as np
import os
import cv2
import shutil
import glob
from math import sqrt
import utils

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
    interrow: [[int, int], [int, int]]

    def __init__(self, image):
        self.image = image
        self.markers = []
        self.elevation_angle = None
        self.interrow = None

    def get_np_shape(self):
        return np.shape(self.image)

    def get_xy_shape(self):
        return np.shape(self.image)[1], np.shape(self.image)[0]

    def add_test_marker(self, center, radius):
        self.markers.append(BallOnPost(center, radius))

    def convert_to_contour(self, points):
        length = len(points)
        cnt = np.zeros([length, 1, 2])
        for i in range(length):
            cnt[i, 0, :] = points[i]
        return np.round(cnt).astype('int')


    def extract_red_ball_from_meta(self, meta):
        for shape in meta["shapes"]:
            if shape["label"] == "redball":
                cnt = self.convert_to_contour(shape["points"])
                (x, y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x), int(y))
                radius = int(radius)
                self.markers.append(BallOnPost(center, radius))

    def check_close_and_below(self, p1, max_dist, p2):
        x_diff = p1[0]-p2[0]
        y_diff = p1[1]-p2[1]
        dist = sqrt(x_diff*x_diff + y_diff*y_diff)
        if (dist < max_dist) and (p1[1]<p2[1]):
            return True
        else:
            return False

    def extract_white_base_from_meta(self, meta):
        for shape in meta["shapes"]:
            if shape["label"] == "whitepost":
                post = shape["points"]
                for marker in self.markers:
                    if self.check_close_and_below(marker.center, 3 * marker.radius, post[0]):
                        white_base = post[1]
                        marker.base_of_post = (round(white_base[0]), round(white_base[1]))





    def extract_row_vector_from_meta(self, meta):
        rows = 0
        leftrow = None
        rightrow = None
        for shape in meta["shapes"]:
            if shape["label"] == "leftrow":
                rows += 1
                leftrow = shape["points"]
            if shape["label"] == "rightrow":
                rows += 1
                rightrow = shape["points"]
        if rows == 2:
            leftrow = self.extrapolate_line_to_vertical_image_boundaries(leftrow)
            rightrow = self.extrapolate_line_to_vertical_image_boundaries(rightrow)
            self.interrow = utils.find_average_of_two_lines(leftrow, rightrow)


    def extract_meta(self, meta):
        self.extract_red_ball_from_meta(meta)
        self.extract_row_vector_from_meta(meta)
        self.extract_white_base_from_meta(meta)

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

    def add_frame(self, img, meta=None):
        self.frames.append(Frame(img))
        if meta is not None:
            self.frames[-1].extract_meta(meta)

    def add_marker(self, frame_num, center, radius):
        self.frames[frame_num].add_test_marker(center, radius)

    def get_shape(self):
        if len(self.frames) > 0:
            return self.frames[0].get_shape()
        else:
            return 0, 0

    def extract_frames(self):
        cap = cv2.VideoCapture(self.filepath)
        while not cap.isOpened():
            cap = cv2.VideoCapture(self.filepath)
            cv2.waitKey(1000)
            print("Wait for the header")

        for i in range(int(cap.get(cv2.CAP_PROP_FRAME_COUNT))):
            ret, frame = cap.read()
            if ret:
                self.frames.append(frame)

    def get_frame_count(self):
        return len(self.frames)

    def check_frames_extracted(self):
        if self.get_frame_count() == 0:
            print("Extracting Frames...")
            self.extract_frames()



    def check_folder_empty(self, filepath):
        if self.check_folder_exists(filepath):
            shutil.rmtree(filepath)
        os.makedirs(filepath)





    def save_frames_to_file(self, subsample_rate: int = 1):
        # If folder exists delete it, then create empty folder.
        self.check_folder_empty(self.filepath[:-4])

        # Ensure frames have been extracted
        self.check_frames_extracted()

        # Save frames at subsample rate
        print("Saving frames")
        for i in range(0, self.get_frame_count(), subsample_rate):
            filename = os.path.join(self.filepath[:-4], str(i).zfill(4)+".png")
            cv2.imwrite(filename, self.frames[i])

    def load_frames_from_file(self, load_meta=False):
        # Check if folder exists
        if not self.check_folder_exists(self.filepath):
            print("Folder does not exist")
            quit()

        # Find and load all png files in folder
        files = glob.glob(os.path.join(self.filepath, "*.png"))
        for file in files:
            img = cv2.imread(file)
            if load_meta:
                meta = utils.load_json(file[:-3] + "json")
                self.add_frame(img, meta)
            else:
                self.add_frame(img)

        print(str(self.get_frame_count()) + " frames saved")

# Testing video
print("Hello World")
vid = Video(".\\data\\Test")
vid.load_frames_from_file(load_meta=True)
