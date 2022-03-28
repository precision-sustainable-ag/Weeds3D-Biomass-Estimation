import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
from skimage.measure import LineModelND, ransac
from line import Line3D
import matplotlib.pyplot as plt
from math import asin

class PointCloud:
    data: o3d.geometry.PointCloud
    points_backup: o3d.geometry.PointCloud.points
    white_post: Line3D

    def __init__(self, filepath=None):
        self.white_post = None
        if filepath is None:
            self.data = None
            self.points_backup = None
        else:
            self.load(filepath)
            self.points_backup = self.get_points()

    def load(self, filepath):
        self.data = o3d.io.read_point_cloud(filepath)

    def get_points(self):
        return np.asarray(self.data.points)

    def set_points(self, xyz):
        self.data.points = o3d.utility.Vector3dVector(xyz)

    def reset_points(self):
        self.set_points(self.points_backup)

    def rotate(self, rotvec):
        r = R.from_rotvec(rotvec)
        xyz = self.get_points()
        xyz2 = r.apply(xyz)
        self.set_points(xyz2)
        if self.white_post is not None:
            self.white_post.rotate(rotvec)

    def translate(self, translate_vector):
        xyz = self.get_points()
        xyz2 = xyz + translate_vector
        self.set_points(xyz2)
        if self.white_post is not None:
            self.white_post.translate(translate_vector)

    def scale(self, scale_factor):
        xyz = self.get_points()
        xyz2 = xyz*scale_factor
        self.set_points(xyz2)
        if self.white_post is not None:
            self.white_post.scale(scale_factor)

    def translate_to_z_gt_0(self):
        xyz = self.get_points()
        min_h = np.min(xyz[:, 2])
        xyz[:, 2] -= min_h
        self.set_points(xyz)
        if self.white_post is not None:
            self.white_post.translate(-1*min_h)

    def scale_to_z_equals_1(self):
        xyz = self.get_points()
        max_h = np.max(xyz[:, 2])
        min_h = np.min(xyz[:, 2])
        h_diff = max_h - min_h
        xyz2 = xyz / h_diff
        self.set_points(xyz2)

    def show_o3d(self, sample_points=None):
        if sample_points is None:
            o3d.visualization.draw_geometries([self.data])
        else:
            colors = np.asarray(self.data.colors)
            points = self.get_points()
            sampled_pcloud = o3d.geometry.PointCloud()
            sampled_pcloud.colors = o3d.utility.Vector3dVector(colors[sample_points])
            sampled_pcloud.points = o3d.utility.Vector3dVector(points[sample_points])
            o3d.visualization.draw_geometries([sampled_pcloud])

    def show_np(self, sample_points=None):
        xyz = self.get_points()
        pole_x = xyz[:, 0]
        pole_y = xyz[:, 1]
        pole_z = xyz[:, 2]
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(pole_x, pole_y, pole_z, c=pole_z) if sample_points is None else ax.scatter(pole_x[sample_points], pole_y[sample_points], pole_z[sample_points], c=pole_z[sample_points])
        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        plt.show()

    def find_post(self, min_h, max_h):
        xyz = self.get_points()
        pole_x = xyz[:, 0]
        pole_y = xyz[:, 1]
        pole_z = xyz[:, 2]
        pole_points1 = pole_z > min_h
        pole_points2 = pole_z < max_h
        pole_points = np.bitwise_and(pole_points1, pole_points2)

        shorter_array = np.array([pole_x[pole_points], pole_y[pole_points], pole_z[pole_points]])
        model_robust, inliers = ransac(shorter_array.transpose(), LineModelND, min_samples=50, residual_threshold=0.05,
                                       max_trials=1000)
        self.white_post = Line3D(model_robust.params[0], model_robust.params[1])
        print(model_robust.params[1])

    def vertical_cutout_of_points(self, xmin, xmax, ymin, ymax):
        xyz = self.get_points()
        xyz_ge_xmin = xyz[:, 0] >= xmin
        xyz_le_xmax = xyz[:, 0] <= xmax
        xyz_ge_ymin = xyz[:, 1] >= ymin
        xyz_le_ymax = xyz[:, 1] <= ymax
        xyz_x_range = np.bitwise_and(xyz_ge_xmin, xyz_le_xmax)
        xyz_y_range = np.bitwise_and(xyz_ge_ymin, xyz_le_ymax)
        xyz_xy_range = np.bitwise_and(xyz_x_range, xyz_y_range)
        return xyz_xy_range

    def move_post_vertical(self):
        post_vector = self.white_post.vector3D
        self.rotate(np.array([post_vector.y, -1*post_vector.x, 0]))
        post_x, post_y, post_z = self.white_post.get_xyz()
        cutout_size = 0.1
        xyz_xy_range = self.vertical_cutout_of_points(post_x-cutout_size, post_x+cutout_size, post_y-cutout_size, post_y+cutout_size)
        xyz = self.get_points()
        max_h = np.max(xyz[xyz_xy_range, 2])
        min_h = np.min(xyz[xyz_xy_range, 2])
        diff_h = max_h-min_h
        print(max_h)
        print(min_h)
        self.translate(np.array([0, 0, -1*min_h]))
        self.scale(1/diff_h)
        xyz = self.get_points()
        pole_z = xyz[:, 2]
        print(np.max(pole_z))
        print(np.min(pole_z))

    def center_origin_point(self):
        post_x, post_y, post_z = self.white_post.get_xyz()
        self.translate(np.array([-1*post_x, -1*post_y, 0]))
