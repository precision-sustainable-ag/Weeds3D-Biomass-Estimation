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

    def save_frames_to_file(self, extension: str, subsample_rate: int = 1):
        # If folder exists delete it, then create empty folder.
        filepath = self.path_to_folder+extension
        utils.check_folder_empty(filepath)

        # Ensure frames have been extracted
        self.check_frames_extracted()

        # Save frames at subsample rate
        print("Saving " + str(self.get_frame_count()) + " frames")
        for i in range(0, self.get_frame_count(), subsample_rate):
            filename = os.path.join(filepath, str(i).zfill(4)+".png")
            cv2.imwrite(filename, self.frames[i].image)

    def load_frames_from_file(self, load_meta=False):
        # Check if folder exists
        if not utils.check_folder_exists(self.path_to_folder):
            print("Folder does not exist")
            quit()

        # Find and load all png files in folder
        files = glob.glob(os.path.join(self.path_to_folder, "*.png"))
        for file in files:
            img = cv2.imread(file)
            if load_meta:
                meta = utils.load_json(file[:-3] + "json")
                self.add_frame(img, meta)
            else:
                self.add_frame(img)

        print(str(self.get_frame_count()) + " frames saved")

    def find_red_balls_in_video(self):
        for f in self.frames:
            f.find_red_balls_in_image()

    def evaluate_red_ball_predictions(self, saveImages=False):
        results = Evaluation(0, 0, 0)
        for f in self.frames:
            results.add(f.evaluate_red_ball_predictions(saveImages=saveImages))
        results.print()
        if saveImages:
            self.save_frames_to_file("_labeled")
