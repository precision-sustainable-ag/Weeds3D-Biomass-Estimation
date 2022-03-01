import os
import json
import shutil
import cv2
from math import sqrt
from point import Point
import point
import numpy as np

def check_file_exists_bool(filepath):
    return True if os.path.isfile(filepath) else False

def check_folder_exists_bool(filepath):
    return True if os.path.isdir(filepath) else False

def check_folder_exists_create(filepath):
    if not check_folder_exists_bool(filepath):
        os.makedirs(filepath)

def check_folder_empty(filepath):
    if check_folder_exists_bool(filepath):
        shutil.rmtree(filepath)
    os.makedirs(filepath)

def check_close_and_below(p1: Point, max_dist: int, p2: Point):
    dist = point.distance_between_points(p1, p2)
    return True if (dist < max_dist) and (p1.y < p2.y) else False

def load_json(filepath):
    data = []
    with open(filepath) as file:
        data = json.load(file)
        file.close()
    return data

def load_txt(filepath):
    lines = []
    if check_file_exists_bool(filepath):
        with open(filepath) as file:
            lines = [float(i) for line in file for i in line.split(' ') if i.strip()]
    return lines

def find_largest_gaps(points:list, num_clusters:int):
    gaps = np.zeros(len(points)-1)
    for i in range(len(points)-1):
        gaps[i] = points[i+1]-points[i]  # Calculate gaps
    sort_index = np.argsort(gaps)  # Identify num_clusters-1 largest gaps
    return np.sort(sort_index[-num_clusters:])
