from unittest import TestCase
from SfM.pointcloud import PointCloud
from SfM.camera import Camera
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import numpy as np

class TestPointCloud(TestCase):
    def setUp(self):

        self.pcloud = PointCloud("./pointcloudtest.txt.ply")
        camera = Camera(translation=np.array([0, 0.816, 4.039]),
                         rotation=R.from_euler('zyx', [-45, 0, 125.268], degrees=True))
        self.pcloud.cameras.append(camera)
        k1 = -0.126
        k2 = 0.004
        self.pcloud.lens_distort_point_cloud(cam_num=0)
        self.pcloud.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd=self.pcloud.data,
                                                                                           radii=o3d.utility.DoubleVector(
                                                                                               [0.8]))
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=3840, height=2160)
        vis.add_geometry(self.pcloud.mesh)
        ctr = vis.get_view_control()
        ctr.convert_from_pinhole_camera_parameters(camera.PinholeCameraParameters, allow_arbitrary=True)
        vis.run()

    def test_mesh(self):
        self.assertEqual(1, 1)
