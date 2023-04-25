from unittest import TestCase
from SfM.pointcloud import PointCloud
from SfM.camera import Camera
import open3d as o3d
from scipy.spatial.transform import Rotation as R
import numpy as np

class TestPointCloud(TestCase):
    def setUp(self):

        self.pcloud = PointCloud("./pointcloudtest.txt.ply")
        self.pcloud.mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd=self.pcloud.data,
                                                                                           radii=o3d.utility.DoubleVector(
                                                                                               [0.8]))
        #self.pcloud.rotate(np.pi / 2 * np.array([1, 0, 0]))
        Rot = self.pcloud.mesh.get_rotation_matrix_from_xyz((np.pi / 2, 0, 0))
        self.pcloud.mesh.rotate(Rot, center=(0, 0, 0))

        camera1 = Camera(translation=np.array([-1, 0.5, 3]), rotation=R.from_euler('zyx', [0, 0, 0], degrees=True)) # 2x1 viewing towards +y, up +z, p @ top left
        camera2 = Camera(translation=np.array([-1.5, -1, 4]), rotation=R.from_euler('zyx', [90, 90, 0], degrees=True)) # 3x2 viewing towards -z, up -x, p @ top left
        camera3 = Camera(translation=np.array([1.5, 0.5, 3]), rotation=R.from_euler('zyx', [0, -90, 0], degrees=True)) # 3x1 viewing towards +x, up +z, p @ top right
        self.pcloud.cameras.append(camera1)
        self.pcloud.cameras.append(camera2)
        self.pcloud.cameras.append(camera3)
        vis = o3d.visualization.Visualizer()
        vis.create_window(width=1920, height=1080)
        vis.add_geometry(self.pcloud.mesh)
        ctr = vis.get_view_control()
        ctr.convert_from_pinhole_camera_parameters(camera3.PinholeCameraParameters, allow_arbitrary=True)
        vis.run()

    def test_mesh(self):
        self.assertEqual(1, 1)
