import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
from skimage.measure import LineModelND, ransac
import matplotlib.pyplot as plt


def custom_draw_geometry(pcd):
    # The following code achieves the same effect as:
    # o3d.visualization.draw_geometries([pcd])
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    cam = vis.get_view_control()
    vis.run()
    vis.destroy_window()

def find_pole_vector(points, pcnt):
    pole_points1 = points[:, 2] > pcnt
    pole_points2 = points[:, 2] < 0.95
    pole_points = np.bitwise_and(pole_points1, pole_points2)
    pole_x = points[:, 0]
    pole_y = points[:, 1]
    pole_z = points[:, 2]

    shorter_array = np.array([pole_x[pole_points],pole_y[pole_points], pole_z[pole_points]])
    print(shorter_array.transpose().shape)
    model_robust, inliers = ransac(shorter_array.transpose(), LineModelND, min_samples=50, residual_threshold=0.05, max_trials=1000)
    outliers = inliers == False
    print(model_robust.params)
    line_center = model_robust.params[0]
    line_vector = model_robust.params[1]
    line_a = line_center + line_vector*0.3
    line_b = line_center - line_vector*0.3

    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(pole_x, pole_y, pole_z, c=pole_z)
    ax.plot([line_a[0], line_b[0]], [line_a[1], line_b[1]], [line_a[2], line_b[2]])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()


pcd_load = o3d.io.read_point_cloud(".\\data\\Point_Clouds\\GX010075-pmvs_options1.txt.ply", format='ply')
xyz_load = np.asarray(pcd_load.points)

r = R.from_rotvec(np.pi/4 * np.array([1, 0, 0]))
xyz2 = r.apply(xyz_load)
max_h = np.max(xyz2[:, 2])
min_h = np.min(xyz2[:, 2])
print(max_h)
print(min_h)
h_diff = max_h - min_h
print(h_diff)
xyz2[:, 2] -= min_h
max_h = np.max(xyz2[:, 2])
min_h = np.min(xyz2[:, 2])
print(max_h)
print(min_h)
scaled_xyz = xyz2/h_diff
max_h = np.max(scaled_xyz[:, 2])
min_h = np.min(scaled_xyz[:, 2])
print(max_h)
print(min_h)
find_pole_vector(scaled_xyz, 0.75)
pcd_load.points = o3d.utility.Vector3dVector(xyz2)
o3d.visualization.draw_geometries([pcd_load])
custom_draw_geometry(xyz_load)