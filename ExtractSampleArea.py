import open3d as o3d
import numpy as np
from math import radians
from scipy.spatial.transform import Rotation as R
from pointcloud import PointCloud
from skimage.measure import LineModelND, ransac
import matplotlib.pyplot as plt


ESTIMATED_ELEV_DEG = 45


pcloud = PointCloud(".\\data\\Point_Clouds\\pmvs_options1.txt.ply")  # load point cloud
pcloud.rotate(radians(ESTIMATED_ELEV_DEG) * np.array([1, 0, 0]))  # Rotate to approximately flat
pcloud.translate_to_z_gt_0()
pcloud.scale_to_z_equals_1()
pcloud.find_post(min_h=0.75, max_h=0.95)
pcloud.move_post_vertical()
pcloud.center_origin_point()
sample_points = pcloud.vertical_cutout_of_points(xmin=-0.381, xmax=0.381, ymin=-0.656, ymax=0.656)
#pcloud.rotate(np.array([np.pi/2, 0, 0]))  # Rotate 90 degrees

pcloud.show_o3d(sample_points)
#pcloud.show_np(sample_points)

# Roughly scale image

