from SfM.pointcloud import PointCloud
from SfM.camera import Camera
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import numpy as np

def undistort(p, k1, k2):
    p_norm = np.linalg.norm(p)
    p_norm_sq = p_norm * p_norm
    return 1 + k1 * p_norm_sq + k2 * p_norm_sq * p_norm_sq

Rot = R.from_euler('zyx', [-45, 0, -54.732], degrees=True)
t = np.array([0, -0.816, -4.039])
cam_vec = Rot.as_matrix().T @ np.array([0, 0, -1]).T

print(cam_vec)

cam_pos = -1 * Rot.as_matrix().T @ t

print(cam_pos)
pcloud = PointCloud("./pointcloudtest.txt.ply")
P_o2p_wrt_w = pcloud.get_points()
num_points = pcloud.get_num_points()
P_c2p_wrt_c = np.zeros([num_points,3])
P_c2p_wrt_c_znorm = np.zeros([num_points,3])
P_c2p_wrt_c_pix = np.zeros([num_points,3])

f = 1000
k1 = 0 # -0.126
k2 = 0 # 0.004

for i in range(num_points):
    X = P_o2p_wrt_w[i, :]
    P_c2p_wrt_c[i, :] = Rot.as_matrix() @ X.T + t
    P_c2p_wrt_c_znorm[i, :] = -P_c2p_wrt_c[i, :] / P_c2p_wrt_c[i, 2]  # Perspective Division by -z axis
    P_c2p_wrt_c_pix[i, :] = f * undistort(P_c2p_wrt_c_znorm[i, :], k1, k2) * P_c2p_wrt_c_znorm[i, :]

pcloud.set_points(P_c2p_wrt_c_pix)

vis = o3d.visualization.Visualizer()
vis.create_window(width=3840, height=2160)
vis.add_geometry(pcloud.data)
ctr = vis.get_view_control()
camera = Camera(translation=np.array([0, 0, 0]), rotation=R.from_euler('zyx', [0, 0, 180], degrees=True))
ctr.convert_from_pinhole_camera_parameters(camera.PinholeCameraParameters, allow_arbitrary=True)
vis.run()

