from video import Video
from circle import Circle
from point import Point
from frame import Frame
from redball import RedBall
import circle
import os
import glob
from math import ceil
import utils
import numpy as np


# Testing video
print("Hello World")
folderpath = '.\\data\\TX-COTTON1-1-CALIB-COTTON1-GX010031'
file = None
vid = Video(folderpath, file)
vid.load_frames_from_file(load_meta=True, meta_file_type='txt')
vid.fill_in_individual_dropouts()
vid.find_red_ball_ids(vid.red_ball_elevation_method)
vid.isolate_frames_around_red_balls()
print(vid.frame_cuts)

