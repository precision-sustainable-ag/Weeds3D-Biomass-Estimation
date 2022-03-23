import redball
from frame import Frame
from evaluation import Evaluation
import numpy as np
import cv2
import os
import utils
import glob
from flipflop import FlipFlipCount
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from line import Line
import line
from point import Point

class Video:
    filename: str
    folderpath: str
    frames: list
    frame_cuts: list
    width_to_elevation_angle: Line
    elevation_angle_to_frames_be4_center: Line
    elevation_angle_to_num_of_frames: Line

    def __init__(self, path_to_folder, filename):
        self.filename = filename
        self.path_to_folder = path_to_folder
        self.frames = []
        self.frame_cuts = []
        self.width_to_elevation_angle = Line(m=0.2217, c=10.782)
        self.elevation_angle_to_frames_be4_center = Line(m=-3.5086, c=96.267)
        self.elevation_angle_to_frames_after_center = Line(m=-5.4423, c=92.381)

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

    def list_of_red_ball_ids(self):
        red_ball_ids_list = []
        for frame in self.frames:
            if frame.has_red_ball():
                red_ball_ids_list.append(frame.markers_pred[0].ball_id)
        return red_ball_ids_list

    def red_ball_gaps_method(self, red_balls):
        gap_starts = utils.find_largest_gaps(red_balls, 2)
        for i in range(len(red_balls)):
            if i <= gap_starts[0]:
                self.frames[red_balls[i]].markers_pred[0].ball_id = 0
            elif i <= gap_starts[1]:
                self.frames[red_balls[i]].markers_pred[0].ball_id = 1
            else:
                self.frames[red_balls[i]].markers_pred[0].ball_id = 2

    def red_ball_elevation_method(self, red_balls):
        # step through frames with ball
        red_ball_i_min = 0
        trailingEdgeDetection = FlipFlipCount(counter_target=2)
        for frame_id in red_balls:
            frame = self.frames[frame_id]

            # Should red_ball_i_min be increased?
            trailingEdgeDetection.update_input(frame.isABallInBottomHalf())
            #print("isABallInBottomHalf: " + str(frame.isABallInBottomHalf()) + ", falling edge: " + str(
            #    trailingEdgeDetection.falling_edge()))
            if trailingEdgeDetection.falling_edge():
                red_ball_i_min += 1

            # Assign ids
            id = red_ball_i_min
            for red_ball in frame.markers_pred:
                red_ball.ball_id = id
                id += 1

    def interpolate_red_ball_1_frame(self,first_frame_id):
        red_balls1, red_balls3 = self.frames[first_frame_id].markers_pred, self.frames[first_frame_id+2].markers_pred
        if len(red_balls1) == len(red_balls3):
            for i in range(len(red_balls1)):
                rb1 = red_balls1[i]
                rb3 = red_balls3[i]
                rb2 = redball.interpolate_red_ball(rb1, rb3)
                self.frames[first_frame_id+1].markers_pred.append(rb2)

    def fill_in_individual_dropouts(self):
        red_balls = self.list_of_frames_with_red_ball()
        for i in range(len(red_balls)-1):
            if red_balls[i+1]-red_balls[i] == 2:
                self.interpolate_red_ball_1_frame(red_balls[i])

    def find_red_ball_ids(self, red_ball_function):
        # Ensure frames have been extracted
        self.check_frames_extracted()

        red_balls = self.list_of_frames_with_red_ball()
        red_ball_function(red_balls)

    def num_frames_w_red_ball_id(self, ball_id):
        count = 0
        for frame in self.frames:
            if frame.has_red_ball() and frame.has_red_ball_with_id(ball_id):
                count += 1
        return count

    def extract_ball_trajectories(self):
        trajectories = []
        for id in range(3):
            traj = []
            for frame in self.frames:
                traj.append(frame.get_redball_elevation(id))
            trajectories.append(traj)
        return trajectories

    def fit_curve(self, x, y):
        rough_line = Line(p1=Point(x[0], y[0]), p2=Point(x[-1], y[-1]))
        xmin, xmax = rough_line.fy(0.9), rough_line.fy(-0.9)
        xcurve = np.arange(xmin, xmax)
        popt, pcov = curve_fit(utils.parabola_func, x, y)
        ycurve = utils.parabola_func(xcurve, *popt)
        return xcurve, ycurve

    def plot_red_ball_trajectories(self, trajectories):
        length = len(trajectories[0])
        plt.figure(figsize=[16,4])
        for i in range(3):
            num_balls = self.num_frames_w_red_ball_id(i)
            x = np.zeros([num_balls])
            y = np.zeros([num_balls])
            count = 0
            for j in range(length):
                if trajectories[i][j] is not None:
                    y[count] = trajectories[i][j]
                    x[count] = j
                    count += 1
            print("x: "+ str(x))
            print("y: " + str(y))
            plt.plot(x, y)
            xcurve, ycurve = self.fit_curve(x, y)
            plt.plot(xcurve, ycurve, '--')
        plt.axis([0, trajectories[2][-1], -1, 1])
        plt.show()

    def find_frame_just_above_0_elevation(self, ball_id: int, max_elevation: float):
        frame_id_above_0 = None
        for i in range(len(self.frames)):
            frame = self.frames[i]
            if frame.has_red_ball_with_id(ball_id) and frame.markers_pred[0].elevation > 0 and frame.markers_pred[0].elevation <= max_elevation:
                frame_id_above_0 = i
        return frame_id_above_0

    def extrapolate_to_0_elevation(self, ball_id):
        x = []
        y = []
        for i in range(len(self.frames)):
            if self.frames[i].isABallInBottomHalf():
                x.append(i)
                y.append(self.frames[i].markers_pred[0].elevation)
        fit_line = line.fit_line(x, y)
        center_frame_id = fit_line.fy(0)
        width1 = 2*self.frames[x[0]].markers_pred[0].ball.radius
        width2 = 2 * self.frames[x[1]].markers_pred[0].ball.radius
        width_line = Line(p1=Point(x[0], width1), p2=Point(x[1], width2))
        center_width = width_line.fx(center_frame_id)
        return center_frame_id, center_width

    def interpolate_to_0_elevation(self, frame_id_above_0):
        elev1 = self.frames[frame_id_above_0].markers_pred[0].elevation
        elev2 = self.frames[frame_id_above_0+1].markers_pred[0].elevation
        width1 = 2*self.frames[frame_id_above_0].markers_pred[0].ball.radius
        width2 = 2*self.frames[frame_id_above_0+1].markers_pred[0].ball.radius

        elevation_diff = elev1 - elev2
        center_ratio = elev1 / elevation_diff
        center_frame_id = frame_id_above_0 - center_ratio
        width_diff = width2-width1
        center_width = width1 + width_diff*center_ratio
        return center_frame_id, center_width

    def isolate_frames_around_red_balls(self):
        width_avg = 0
        centers = []
        for ball_id in range(3):
            frame_id_above_0 = self.find_frame_just_above_0_elevation(ball_id, max_elevation=0.2)
            if frame_id_above_0 is None:
                print("need to extrapolate")
                center_frame_id, center_width = self.extrapolate_to_0_elevation(ball_id)
            else:
                print("can interpolate")
                center_frame_id, center_width = self.interpolate_to_0_elevation(frame_id_above_0)
            print(center_width)
            centers.append(center_frame_id)
            width_avg += center_width
        width_avg /= 3
        elevation_est = self.width_to_elevation_angle.fx(width_avg)
        frames_be4_center = max(self.elevation_angle_to_frames_be4_center.fy(elevation_est), 16)
        frames_after_center = self.elevation_angle_to_frames_after_center.fy(elevation_est)

        print("centers: " + str(centers))
        print("width_avg: "+str(width_avg))
        print("elevation_est: "+str(elevation_est))
        print("frames_be4_center: "+str(frames_be4_center))
        print("frames_after_center: " + str(frames_after_center))
        for i in range(3):
            start_frame_id = int(round(centers[i] - frames_be4_center, 0))
            finish_frame_id = int(round(centers[i] + frames_after_center, 0))
            self.frame_cuts.append([start_frame_id, finish_frame_id])





    # Steps to TODO:
    # 1. Find red ball center frame ids
    # 1a. If frames redball detections exist on both sides of center, use them
    # 1b. If not, extrapolate a line from the lower detections
    # 2. Extract center frame widths
    # 3. Use calculated linear relationship between width and elevation angle to predict elevation angle. y = 0.2217x + 10.782
    # 4. Use calculated linear relationship between elevation angle and frame id to start from. y = -0.2755x + 107.69
    # 5. Use calculated linear relationship between elevation angle and number of frames to include. y = -0.3509x +96.267
    # 6. Store this in a useful way






