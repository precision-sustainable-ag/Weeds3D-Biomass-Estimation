# Standard Libraries
import open3d as o3d
import numpy as np

class Camera:
    PinholeCameraParameters: o3d.camera.PinholeCameraParameters

    def __init__(self, translation, rotation):
        extrinsic_np = np.zeros([4,4])
        extrinsic_np[0:3,0:3] = rotation.as_matrix()
        extrinsic_np[0:3,3] = translation
        extrinsic_np[3,3] = 1
        extrinsic = extrinsic_np
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