# Standard Libraries
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
from skimage.measure import LineModelND, ransac
import matplotlib.pyplot as plt
from SurfaceReconstruction import SurfaceReconstruction, PoissonSurfaceReconstruction

# Custom Libraries
from SfM.camera import Camera
from shared.line import Line3D

class PointCloud:
    data: o3d.geometry.PointCloud
    points_backup: o3d.geometry.PointCloud.points
    white_post: Line3D
    cameras: list
    surface_reconstruction_method: SurfaceReconstruction
    mesh: o3d.geometry.TriangleMesh

    def __init__(self, filepath=None, surface_reconstruction_method: SurfaceReconstruction = PoissonSurfaceReconstruction):
        self.white_post = None
        self.cameras = []
        self.surface_reconstruction_method = surface_reconstruction_method()
        self.mesh = None
        if filepath is None:
            self.data = None
            self.points_backup = None
        else:
            self.load(filepath)
            self.points_backup = self.get_points()

    def load(self, filepath):
        self.data = o3d.io.read_point_cloud(filepath)

    def get_points(self):
        return np.asarray(self.data.points)

    def set_points(self, xyz):
        self.data.points = o3d.utility.Vector3dVector(xyz)

    def set_colors(self, rgb):
        self.data.colors = o3d.utility.Vector3dVector(rgb)

    def reset_points(self):
        self.set_points(self.points_backup)

    def get_cam_locations(self):
        num_cams = len(self.cameras)
        points = np.zeros([num_cams, 3])
        for i in range(num_cams):
            points[i, :] = self.cameras[i].get_translation()
        return points

    def set_cam_locations(self, cam_pos):
        num_cams = len(self.cameras)
        for i in range(num_cams):
            #self.cameras[i].translation = cam_pos[i, :]
            self.cameras[i].translate_to(cam_pos[i, :])

    def rotate(self, rotvec):
        r = R.from_rotvec(rotvec)
        xyz = self.get_points()
        xyz2 = r.apply(xyz)
        self.set_points(xyz2)
        cam_xyz = self.get_cam_locations()
        self.set_cam_locations(r.apply(cam_xyz))
        if self.white_post is not None:
            self.white_post.rotate(rotvec)

    def translate(self, translate_vector):
        xyz = self.get_points()
        cam_xyz = self.get_cam_locations()
        xyz2 = xyz + translate_vector
        cam_xyz2 = cam_xyz + translate_vector
        self.set_points(xyz2)
        self.set_cam_locations(cam_xyz2)
        if self.white_post is not None:
            self.white_post.translate(translate_vector)

    def scale(self, scale_factor):
        xyz = self.get_points()
        cam_xyz = self.get_cam_locations()
        xyz2 = xyz*scale_factor
        cam_xyz2 = cam_xyz*scale_factor
        self.set_points(xyz2)
        self.set_cam_locations(cam_xyz2)
        if self.white_post is not None:
            self.white_post.scale(scale_factor)

    def translate_to_z_gt_0(self):
        xyz = self.get_points()
        cam_xyz = self.get_cam_locations()
        min_h = np.min(xyz[:, 2])
        xyz[:, 2] -= min_h
        cam_xyz[:, 2] -= min_h
        self.set_points(xyz)
        self.set_cam_locations(cam_xyz)
        if self.white_post is not None:
            self.white_post.translate(-1*min_h)

    def scale_to_z_equals_1(self):
        xyz = self.get_points()
        cam_xyz = self.get_cam_locations()
        max_h = np.max(xyz[:, 2])
        min_h = np.min(xyz[:, 2])
        h_diff = max_h - min_h
        xyz2 = xyz / h_diff
        cam_xyz2 = cam_xyz / h_diff
        self.set_points(xyz2)
        self.set_cam_locations(cam_xyz2)

    def show_o3d(self, sample_points=None, cam_pos=None):
        #vis = o3d.visualization.Visualizer()
        #vis.create_window(visible=False)
        #vis.add_geometry(self.mesh)
        #vis.update_geometry(self.mesh)
        #vis.poll_events()
        #vis.update_renderer()
        #cam = vis.get_view_control()
        #cam.rotate(10.0, 0)
        #vis.capture_screen_image('test.png')
        #vis.destroy_window()

        if sample_points is None:
            if cam_pos is None:
                o3d.visualization.draw_geometries([self.data])
            else:
                o3d.visualization.draw_geometries([self.data],
                                                  zoom=0.3,
                                                  front=[0, -1, 0],
                                                  lookat=[0, 0, 0],
                                                  up=[0, 0, 1])
        else:
            colors = np.asarray(self.data.colors)
            points = self.get_points()
            sampled_pcloud = o3d.geometry.PointCloud()
            sampled_pcloud.colors = o3d.utility.Vector3dVector(colors[sample_points])
            sampled_pcloud.points = o3d.utility.Vector3dVector(points[sample_points])
            o3d.visualization.draw_geometries([sampled_pcloud])

    def crop_mesh(self, bbox):
        pcd = o3d.geometry.PointCloud()
        # Bounding Box points from numpy to Open3D
        pcd.points = o3d.utility.Vector3dVector(bbox)

        # Create Bounding Box
        boundingbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd.points)

        # Crop mesh
        self.mesh = self.mesh.crop(boundingbox)


    def show_mesh(self):
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(self.mesh)
        ctr = vis.get_view_control()
        print(ctr.convert_to_pinhole_camera_parameters().intrinsic)
        ctr.convert_from_pinhole_camera_parameters(self.cameras[0].PinholeCameraParameters)

        #vis.run()
        #o3d.visualization.draw_geometries([self.mesh],
        #                                  zoom=1,
        #                                  front=[0, -1, 0],
        #                                  lookat=[0, 0, 0],
        #                                  up=[0, 0, 1])

    def show_np(self, sample_points=None, show_cameras=False):
        xyz = self.get_points()
        pole_x = xyz[:, 0]
        pole_y = xyz[:, 1]
        pole_z = xyz[:, 2]
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(pole_x, pole_y, pole_z, c=pole_z) if sample_points is None else ax.scatter(pole_x[sample_points], pole_y[sample_points], pole_z[sample_points], c=pole_z[sample_points])
        if show_cameras:
            cam_points = self.get_cam_locations()
            ax.scatter(cam_points[:, 0], cam_points[:, 1], cam_points[:, 2])

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        plt.show()

    def find_post(self, min_h, max_h):
        xyz = self.get_points()
        pole_x = xyz[:, 0]
        pole_y = xyz[:, 1]
        pole_z = xyz[:, 2]
        pole_points1 = pole_z > min_h
        pole_points2 = pole_z < max_h
        pole_points = np.bitwise_and(pole_points1, pole_points2)

        shorter_array = np.array([pole_x[pole_points], pole_y[pole_points], pole_z[pole_points]])
        model_robust, inliers = ransac(shorter_array.transpose(), LineModelND, min_samples=50, residual_threshold=0.05,
                                       max_trials=1000)
        self.white_post = Line3D(model_robust.params[0], model_robust.params[1])

    def vertical_cutout_of_points(self, xmin, xmax, ymin, ymax):
        xyz = self.get_points()
        xyz_ge_xmin = xyz[:, 0] >= xmin
        xyz_le_xmax = xyz[:, 0] <= xmax
        xyz_ge_ymin = xyz[:, 1] >= ymin
        xyz_le_ymax = xyz[:, 1] <= ymax
        xyz_x_range = np.bitwise_and(xyz_ge_xmin, xyz_le_xmax)
        xyz_y_range = np.bitwise_and(xyz_ge_ymin, xyz_le_ymax)
        xyz_xy_range = np.bitwise_and(xyz_x_range, xyz_y_range)
        return xyz_xy_range

    def vertical_cutout_of_mesh(self, xmin, xmax, ymin, ymax, zmin=-10, zmax=10):
        pcd = o3d.geometry.PointCloud()
        boundingBox = np.array([[xmin, ymin, zmin],
                               [xmin, ymin, zmax],
                               [xmin, ymax, zmin],
                               [xmin, ymax, zmax],
                               [xmax, ymin, zmin],
                               [xmax, ymin, zmax],
                               [xmax, ymax, zmin],
                               [xmax, ymax, zmax]])
        # Bounding Box points from numpy to Open3D
        pcd.points =o3d.utility.Vector3dVector(boundingBox)

        # Create Bounding Box
        return o3d.geometry.AxisAlignedBoundingBox.create_from_points(pcd.points)


    def move_post_vertical(self):
        post_vector = self.white_post.vector3D
        self.rotate(np.array([post_vector.y, -1*post_vector.x, 0]))
        post_x, post_y, post_z = self.white_post.get_xyz()
        cutout_size = 0.1
        xyz_xy_range = self.vertical_cutout_of_points(post_x-cutout_size, post_x+cutout_size, post_y-cutout_size, post_y+cutout_size)
        xyz = self.get_points()
        max_h = np.max(xyz[xyz_xy_range, 2])
        min_h = np.min(xyz[xyz_xy_range, 2])
        diff_h = max_h-min_h
        self.translate(np.array([0, 0, -1*min_h]))
        self.scale(1/diff_h)
        xyz = self.get_points()
        pole_z = xyz[:, 2]

    def center_origin_point(self):
        post_x, post_y, post_z = self.white_post.get_xyz()
        self.translate(np.array([-1*post_x, -1*post_y, 0]))

    def extract_cameras(self, filepath):
        file = open(filepath, 'r')
        Lines = file.readlines()
        num_cameras = int(Lines[1].split()[0])
        for i in range(2, num_cameras*5+2, 5):
            line1 = Lines[i+1].split()
            line2 = Lines[i+2].split()
            line3 = Lines[i+3].split()
            line4 = Lines[i+4].split()
            rot_np = np.array([[float(line1[0]), float(line1[1]), float(line1[2])],[float(line2[0]), float(line2[1]), float(line2[2])],[float(line3[0]), float(line3[1]), float(line3[2])]])
            rot = R.from_matrix(rot_np)
            trans = np.array([float(line4[0]), float(line4[1]), float(line4[2])])
            xyz = -1 * rot.as_matrix().T @ trans.T
            self.cameras.append(Camera(xyz, rot))

    def move_camera(self, rotation,translation):
        pass

    def move_to_camera_position(self, position):
        camera = self.cameras[position]
        rot = camera.rotation
        trans = camera.translation
        self.move_camera(rot, trans)

    def surface_reconstruction(self):
        self.mesh = self.surface_reconstruction_method.reconstruct_surface(pcd=self.data)

