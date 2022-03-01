from frame import Frame
from evaluation import Evaluation
import numpy as np
import cv2
import os
import utils
import glob

class Video:
    filename: str
    folderpath: str
    frames: list
    red_ball_center_frames = list

    def __init__(self, path_to_folder, filename):
        self.filename = filename
        self.path_to_folder = path_to_folder
        self.frames = []
        self.red_ball_center_frames = []

    def add_frame(self, img, meta=None):
        self.frames.append(Frame(img))
        if meta is not None:
            self.frames[-1].extract_meta(meta)

    def get_shape(self):
        return self.frames[0].get_shape() if len(self.frames) > 0 else 0, 0

    def extract_frames(self):
        filepath = os.path.join(self.path_to_folder, self.filename)
        cap = cv2.VideoCapture(filepath)
        while not cap.isOpened():
            cap = cv2.VideoCapture(filepath)
            cv2.waitKey(1000)
            print("Wait for the header")

        for i in range(int(cap.get(cv2.CAP_PROP_FRAME_COUNT))):
            ret, image = cap.read()
            if ret:
                self.frames.append(Frame(image))
        print(str(self.get_frame_count()) + " frames extracted")

    def get_frame_count(self):
        return len(self.frames)

    def check_frames_extracted(self):
        if self.get_frame_count() == 0:
            print("Extracting Frames...")
            self.extract_frames()

    def save_frames_to_file(self, extension: str, subsample_rate: int = 1, start_num: int = 0):
        filepath = os.path.join(self.path_to_folder, extension)
        utils.check_folder_exists_create(filepath)

        # Ensure frames have been extracted
        self.check_frames_extracted()

        # Save frames at subsample rate
        for i in range(subsample_rate, self.get_frame_count(), subsample_rate):
            filename = os.path.join(filepath, str(start_num).zfill(4)+".png")
            print("saving " + filename)
            cv2.imwrite(filename, self.frames[i].image)
            start_num += subsample_rate

    def load_meta(self, file, meta_file_type):
        meta_file = file[:-3] + meta_file_type
        meta = None
        if meta_file_type == 'json':
            meta = utils.load_json(meta_file)
        elif meta_file_type == 'txt':
            data = utils.load_txt(meta_file)
            meta = {"BBox": []}
            for i in range(0, len(data), 5):
                meta["BBox"].append(data[i+1:i+5])
        return meta

    def load_frames_from_file(self, load_meta=False, meta_file_type='json'):
        # Check if folder exists
        if not utils.check_folder_exists_bool(self.path_to_folder):
            print("Folder does not exist")
            quit()

        # Find and load all png files in folder
        files = glob.glob(os.path.join(self.path_to_folder, "*.png"))
        for file in files:
            img = cv2.imread(file)
            if load_meta:
                meta = self.load_meta(file, meta_file_type)
                self.add_frame(img, meta)
            else:
                self.add_frame(img)

        print(str(self.get_frame_count()) + " frames saved")

    def find_red_balls_in_video(self):
        for f in self.frames:
            f.find_red_balls_in_image()

    def calculate_red_ball_elevation(self):
        red_ball_frames = self.list_of_frames_with_red_ball()
        for frame in red_ball_frames:
            frame.calculate_red_ball_elevation()

    def evaluate_red_ball_predictions(self, saveImages=False):
        results = Evaluation(0, 0, 0)
        for f in self.frames:
            results.add(f.evaluate_red_ball_predictions(saveImages=saveImages))
        results.print()
        if saveImages:
            self.save_frames_to_file("_labeled")

    def list_of_frames_with_red_ball(self):
        red_ball_list = []
        id = 0
        for frame in self.frames:
            if frame.has_red_ball():
                red_ball_list.append(id)
            id += 1
        return red_ball_list

    def red_ball_gaps_method(self, red_balls):
        gap_starts = utils.find_largest_gaps(red_balls, 2)
        for i in range(len(red_balls)):
            if i <= gap_starts[0]:
                self.frames[red_balls[i]].markers_pred[0].ball_id = 1
            elif i <= gap_starts[1]:
                self.frames[red_balls[i]].markers_pred[0].ball_id = 2
            else:
                self.frames[red_balls[i]].markers_pred[0].ball_id = 3

    def red_ball_elevation_method(self, red_balls):

        for i in range(len(red_balls)):
            pass

    def find_red_ball_ids(self, red_ball_function):
        # Ensure frames have been extracted
        self.check_frames_extracted()

        red_balls = self.list_of_frames_with_red_ball()
        red_ball_function(red_balls)






