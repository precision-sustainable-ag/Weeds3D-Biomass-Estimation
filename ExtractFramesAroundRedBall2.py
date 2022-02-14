import numpy as np
import cv2
import os
import utils
import glob


class BallOnPost:
    center: (int, int)
    radius: int

    def __init__(self, center, radius):
        self.center = center
        self.radius = radius

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
        print(str(self.get_frame_count()) + " frames extracted")

    def get_frame_count(self):
        return len(self.frames)

    def check_frames_extracted(self):
        if self.get_frame_count() == 0:
            print("Extracting Frames...")
            self.extract_frames()

    def save_frames_to_file(self, subsample_rate: int = 1):
        # If folder exists delete it, then create empty folder.
        utils.check_folder_empty(self.filepath[:-4])

        # Ensure frames have been extracted
        self.check_frames_extracted()

        # Save frames at subsample rate
        print("Saving frames")
        for i in range(0, self.get_frame_count(), subsample_rate):
            filename = os.path.join(self.filepath[:-4], str(i).zfill(4)+".png")
            cv2.imwrite(filename, self.frames[i])

    def load_frames_from_file(self, load_meta=False):
        # Check if folder exists
        if not utils.check_folder_exists(self.filepath):
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

    def find_red_balls_in_video(self):
        pass


# Testing video
print("Hello World")
vid = Video(".\\data\\Calib-test.MP4")
vid.extract_frames()