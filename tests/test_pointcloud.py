from unittest import TestCase
from SfM.pointcloud import PointCloud
from SfM.pointcloud import Camera
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import numpy as np

class TestPointCloud(TestCase):
    def setUp(self):
        self.pcloud = PointCloud("./pointcloudtest.txt.ply")
        self.pcloud.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd=self.pcloud.data,
                                                                                           radii=o3d.utility.DoubleVector(
                                                                                               [0.8]))
        camera1 = Camera(translation=np.array([-3, 1.5, 0.5]), rotation=R.from_euler('zyx', [0, 0, 0], degrees=True))
        camera2 = Camera(translation=np.array([1, -3, 0.5]), rotation=R.from_euler('zyx', [90, 0, 0], degrees=True))
        camera3 = Camera(translation=np.array([1, 1.5, 4]), rotation=R.from_euler('zyx', [0, 90, 0], degrees=True))
        self.pcloud.cameras.append(camera1)
        self.pcloud.cameras.append(camera2)
        self.pcloud.cameras.append(camera3)
        # vis = o3d.visualization.Visualizer()
        # vis.create_window()
        # vis.add_geometry(self.pcloud.mesh)
        # vis.run()

    def test_mesh(self):
        self.assertEqual(1, 1)
