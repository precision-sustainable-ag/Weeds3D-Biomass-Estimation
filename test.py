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

P_o2p_wrt_w = np.array([[0, 0, 0],
                  [0, 3, 0],
                  [2, 3, 0],
                  [2, 0, 0],
                  [0, 0, 1],
                  [0, 3, 1],
                  [2, 3, 1],
                  [2, 0, 1]])

P_c2p_wrt_c = np.zeros([8,3])
P_c2p_wrt_c_znorm = np.zeros([8,3])
P_c2p_wrt_c_pix = np.zeros([8,3])

f = 1000
k1 = 0 # -0.126
k2 = 0 # 0.004

for i in range(8):
    X = P_o2p_wrt_w[i, :]
    P_c2p_wrt_c[i, :] = Rot.as_matrix() @ X.T + t
    P_c2p_wrt_c_znorm[i, :] = -P_c2p_wrt_c[i, :] / P_c2p_wrt_c[i, 2]  # Perspective Division by -z axis
    P_c2p_wrt_c_pix[i, :] = f * undistort(P_c2p_wrt_c_znorm[i, :], k1, k2) * P_c2p_wrt_c_znorm[i, :]

print(P_c2p_wrt_c_pix)

