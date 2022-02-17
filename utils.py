import os
import json
import shutil
import cv2
from math import sqrt
from point import Point
import point

def load_json(filepath):
    f = open(filepath)
    data = json.load(f)
    f.close()
    return data

def check_folder_exists(filepath):
    return True if os.path.isdir(filepath) else False

def check_folder_empty(filepath):
    if check_folder_exists(filepath):
        shutil.rmtree(filepath)
    os.makedirs(filepath)

def check_close_and_below(p1: Point, max_dist: int, p2: Point):
    dist = point.distance_between_points(p1, p2)
    return True if (dist < max_dist) and (p1.y < p2.y) else False
