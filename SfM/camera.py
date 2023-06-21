# Standard Libraries
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation
from shared.line import Line3D

class Camera:
    PinholeCameraParameters: o3d.camera.PinholeCameraParameters

    def __init__(self, translation: np.array, rotation: Rotation):
        extrinsic = np.zeros([4,4])
        extrinsic[0:3,0:3] = rotation.as_matrix()
        extrinsic[0:3,3] = translation
        extrinsic[3,3] = 1
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width=3840, height=2160, fx=997.5123, fy=999.109885, cx=953.45, cy=543.51542)
        self.PinholeCameraParameters = o3d.camera.PinholeCameraParameters()
        self.PinholeCameraParameters.intrinsic = intrinsic
        self.PinholeCameraParameters.extrinsic = extrinsic

    def get_position(self) -> np.array:
        extrinsic = self.PinholeCameraParameters.extrinsic
        return np.asarray(extrinsic[0:3, 3])

    def get_translation(self) -> np.array:
        location = self.get_position()
        translation = -1 * np.array([location[0], location[1], location[2]])
        return translation

    def get_R(self) -> np.array:
        return np.asarray(self.PinholeCameraParameters.extrinsic[0:3, 0:3]).copy()

    def set_R(self, rot_np:np.array) -> None:
        H = np.asarray(self.PinholeCameraParameters.extrinsic).copy()
        H[0:3, 0:3] = rot_np
        self.PinholeCameraParameters.extrinsic = H

    def get_viewing_line(self) -> Line3D:
        R = self.get_R()
        pos = self.get_position()
        line = Line3D(pos, R.T @ np.array([0, 0, -1]).T)
        return line

    def translate_to(self, location):
        extrinsic = np.asarray(self.PinholeCameraParameters.extrinsic).copy()
        extrinsic[0:3,3] = location
        self.PinholeCameraParameters.extrinsic = extrinsic

    def rotate_cameras(self, rot_change):
        rot_new = np.matmul(self.get_R(), rot_change)
        self.set_R(rot_new)

    def as_euler(self, seq:str, degrees:bool):
        rot = Rotation.from_matrix(self.get_R())
        return rot.as_euler(seq=seq, degrees=degrees)

    def undistort(self, r: float, k1: float, k2: float) -> float:
        return 1 + k1*r*r + k2*r*r*r*r

    def distort(self, r: float, k1: float, k2: float) -> float:
        # Compute powers
        k1_2 = k1 * k1
        k1_3 = k1_2 * k1
        k1_4 = k1_3 * k1
        k1_5 = k1_4 * k1
        k1_6 = k1_5 * k1
        k1_7 = k1_6 * k1
        k1_8 = k1_7 * k1
        k1_9 = k1_8 * k1

        k2_2 = k2 * k2
        k2_3 = k2_2 * k2
        k2_4 = k2_3 * k2

        b1 = -k1
        b2 = 3.0 * k1_2 - k2
        b3 = -12.0 * k1_3 + 8.0 * k1 * k2
        b4 = 55.0 * k1_4 - 55.0 * k1_2 * k2 + 5.0 * k2_2
        b5 = -273.0 * k1_5 + 364.0 * k1_3 * k2 - 78.0 * k1 * k2_2
        b6 = 1428.0 * k1_6 - 2380.0 * k1_4 * k2 + 840.0 * k1_2 * k2_2 - 35.0 * k2_3
        b7 = -7752.0 * k1_7 + 15504.0 * k1_5 * k2 - 7752.0 * k1_3 * k2_2 + 816.0 * k1 * k2_3
        b8 = 43263.0 * k1_8 - 100947.0 * k1_6 * k2 + 65835.0 * k1_4 * k2_2 - 11970.0 * k1_2 * k2_3 + 285.0 * k2_4
        b9 = -246675.0 * k1_9 + 657800.0 * k1_7 * k2 - 531300.0 * k1_5 * k2_2 + 141680.0 * k1_3 * k2_3 - 8855.0 * k1 * k2_4

        # Perform the undistortion
        r2 = r * r
        r4 = r2 * r2
        r8 = r4 * r4
        r16 = r8 * r8
        return r*(1.0 + b1 * r2 + b2 * r4 + b3 * r4 * r2 + b4 * r8 + b5 * r8 * r2 + b6 * r8 * r4 + b7 * r8 * r4 * r2 + b8 * r16 + b9 * r16 * r2)
