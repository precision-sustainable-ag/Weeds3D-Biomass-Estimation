from unittest import TestCase
from SfM.pointcloud import PointCloud
import open3d as o3d
import numpy as np

class TestPointCloud(TestCase):
    def setUp(self):
        self.pcloud = PointCloud("./pointcloudtest.txt.ply")
        self.pcloud.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd=self.pcloud.data,
                                                                                           radii=o3d.utility.DoubleVector(
                                                                                               [0.8]))

    def test_mesh(self):
        self.pcloud.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd=self.pcloud.data, radii=o3d.utility.DoubleVector([0.8]))
        #vis = o3d.visualization.Visualizer()
        #vis.create_window()
        #vis.add_geometry(self.pcloud.mesh)
        #vis.run()
