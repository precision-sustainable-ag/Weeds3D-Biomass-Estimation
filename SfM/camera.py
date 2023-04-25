# Standard Libraries
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation

class Camera:
    PinholeCameraParameters: o3d.camera.PinholeCameraParameters

    def __init__(self, translation: np.array, rotation: Rotation):
        extrinsic = np.zeros([4,4])
        extrinsic[0:3,0:3] = rotation.as_matrix()
        extrinsic[0:3,3] = translation
        extrinsic[3,3] = 1
        intrinsic = o3d.camera.PinholeCameraIntrinsic(width=1920, height=1080, fx=997.5123, fy=999.109885, cx=953.45, cy=543.51542)
        self.PinholeCameraParameters = o3d.camera.PinholeCameraParameters()
        self.PinholeCameraParameters.intrinsic = intrinsic
        self.PinholeCameraParameters.extrinsic = extrinsic

    def get_translation(self):
        extrinsic = self.PinholeCameraParameters.extrinsic
        return np.asarray(extrinsic[0:3,3])

    def translate_to(self, location):
        extrinsic = np.asarray(self.PinholeCameraParameters.extrinsic).copy()
        extrinsic[0:3,3] = location
        self.PinholeCameraParameters.extrinsic = extrinsic