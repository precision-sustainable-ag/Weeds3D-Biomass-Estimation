# Standard Libraries
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
from skimage.measure import LineModelND, ransac
import matplotlib.pyplot as plt


# Custom Libraries
from SfM.camera import Camera
from shared.line import Line3D
from SurfaceReconstruction import SurfaceReconstruction, PoissonSurfaceReconstruction

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

    def get_num_points(self):
        return np.shape(self.get_points())[0]
        
    def set_colors(self, rgb):
        self.data.colors = o3d.utility.Vector3dVector(rgb)

    def reset_points(self):
        self.set_points(self.points_backup)

    def get_cam_locations(self):
        num_cams = len(self.cameras)
        points = np.zeros([num_cams, 3])
        for i in range(num_cams):
            points[i, :] = self.cameras[i].get_position()
        return points

    def get_cam_viewing_lines(self) -> list:
        num_cams = len(self.cameras)
        lines = []
        for i in range(num_cams):
            lines.append(self.cameras[i].get_viewing_line())
        return lines

    def set_cam_locations(self, cam_pos):
        num_cams = len(self.cameras)
        for i in range(num_cams):
            #self.cameras[i].translation = cam_pos[i, :]
            self.cameras[i].translate_to(cam_pos[i, :])

    def set_cam_rotations(self, cam_rot):
        num_cams = len(self.cameras)
        for i in range(num_cams):
            self.cameras[i].rotate_cameras(cam_rot)

    def rotate(self, rotvec):
        r = R.from_rotvec(rotvec)
        xyz = self.get_points()
        xyz2 = r.apply(xyz)
        self.set_points(xyz2)
        cam_xyz = self.get_cam_locations()
        self.set_cam_locations(r.apply(cam_xyz))
        r_inv = r.inv()
        self.set_cam_rotations(r_inv.as_matrix())
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
        if sample_points is None:
            if cam_pos is None:
                vis = o3d.visualization.Visualizer()
                vis.create_window(width=1920, height=1080)
                vis.add_geometry(self.data)
                ctr = vis.get_view_control()
                camera1 = Camera(translation=np.array([0, 0, 0]), rotation=R.from_euler('zyx', [0, 0, 90],
                                                                                        degrees=True))
                ctr.convert_from_pinhole_camera_parameters(camera1.PinholeCameraParameters, allow_arbitrary=True)
                vis.run()

            else:
                #Rot = R.from_euler('zyx', [np.radians(-0.02335), np.radians(6.8225), np.radians(180-39.59621)])
                #Rot = R.from_euler('zyx', [np.radians(0), np.radians(0), np.radians(-39.59621)])
                #Rot = R.from_euler('zyx', [0, 0, np.radians(140)])
                #self.data.rotate(Rot.as_matrix(), center=(0, 0, 0))
                #self.data.translate(translation=np.matmul(Rot.as_matrix(), np.array([-0.0607, 1.0531, -1.3717])))
                #self.data.translate(translation=np.array([-0.0611, -0.3838, 1.6860]))
                vis = o3d.visualization.Visualizer()
                vis.create_window(width=1920, height=1080)
                vis.add_geometry(self.data)
                ctr = vis.get_view_control()
                #camera = Camera(translation=self.cameras[cam_pos-1].get_translation(), rotation=R.from_euler('zyx', [0, 0, 90],
                #                                                                        degrees=True))
                #camera0 = Camera(translation=np.array([0, 0, 0]), rotation=R.from_euler('zyx', [0, 0, 0], degrees=True))
                X = np.array([0.0545, 0.3869, -1.3510])  # cam 6: [0.0607, 1.0531, -1.3717]
                rot = R.from_euler('zyx', [np.radians(2.65489), np.radians(6.57612), np.radians(180-38.64658)])
                theta_x = np.radians(180-38.64658)
                rot_x = np.array([[1, 0, 0],[0, np.cos(theta_x), -1*np.sin(theta_x)],[0, np.sin(theta_x), np.cos(theta_x)]])
                X_dash = np.matmul(rot.as_matrix(), X.transpose())
                camera1 = Camera(translation=X_dash, rotation=rot)
                ctr.convert_from_pinhole_camera_parameters(camera1.PinholeCameraParameters, allow_arbitrary=True)
                vis.run()

        else:
            colors = np.asarray(self.data.colors)
            points = self.get_points()
            sampled_pcloud = o3d.geometry.PointCloud()
            sampled_pcloud.colors = o3d.utility.Vector3dVector(colors[sample_points])
            sampled_pcloud.points = o3d.utility.Vector3dVector(points[sample_points])
            vis = o3d.visualization.Visualizer()
            vis.create_window(width=1920, height=1080)
            vis.add_geometry(sampled_pcloud)
            ctr = vis.get_view_control()
            camera1 = Camera(translation=np.array([0.062, 1.058, 1.368]), rotation=R.from_euler('zyx', [0, 0, 180],
                                                                                       degrees=True))
            ctr.convert_from_pinhole_camera_parameters(camera1.PinholeCameraParameters, allow_arbitrary=True)
            vis.run()

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
        #camera = Camera(translation=np.array([0.062, 1.368, 1.055]),
        #                rotation=R.from_euler('zyx', [0, 0, 135], degrees=True))
        camera = Camera(translation=np.array([0, 1, 0]),
                        rotation=R.from_euler('zyx', [0, 0, 135], degrees=True))
        ctr.convert_from_pinhole_camera_parameters(camera.PinholeCameraParameters, allow_arbitrary=True)


        #o3d.visualization.draw_geometries([self.mesh],
        #                                  zoom=1,
        #                                  #front=[0.062, 1.058, 1.368],
        #                                  front=[0, -1, 0],
        #                                  lookat=[0, 1, 0],
        #                                  up=[0, 0, 1])
        vis.run()

    def show_np(self, sample_points=None, show_cameras=False, show_camera_lines=False, show_post_line=False):
        xyz = self.get_points()
        pole_x = xyz[:, 0]
        pole_y = xyz[:, 1]
        pole_z = xyz[:, 2]
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')

        x_diff = np.max(xyz[sample_points, 0]) - np.min(xyz[sample_points, 0])
        y_diff = np.max(xyz[sample_points, 1]) - np.min(xyz[sample_points, 1])
        z_diff = np.max(xyz[sample_points, 2]) - np.min(xyz[sample_points, 2])
        x2z_ratio = x_diff / z_diff
        y2z_ratio = y_diff / z_diff

        ax.scatter(pole_x, pole_y, pole_z, c=pole_z) if sample_points is None else ax.scatter(pole_x[sample_points], pole_y[sample_points], pole_z[sample_points], c=pole_z[sample_points])
        if show_cameras:
            cam_points = self.get_cam_locations()
            #ax.scatter(cam_points[5, 0], cam_points[5, 1], cam_points[5, 2])
            ax.scatter(cam_points[:, 0], cam_points[:, 1], cam_points[:, 2])

        if show_camera_lines:
            cam_lines = self.get_cam_viewing_lines()
            for cam_line in cam_lines:
                xpoints = np.array([cam_line.point3D.x, cam_line.point3D.x + 0.3*cam_line.vector3D.x])
                ypoints = np.array([cam_line.point3D.y, cam_line.point3D.y + 0.3*cam_line.vector3D.y])
                zpoints = np.array([cam_line.point3D.z, cam_line.point3D.z + 0.3*cam_line.vector3D.z])
                ax.plot3D(xpoints, ypoints, zpoints)

        if show_post_line:
            xpoints = np.array([self.white_post.point3D.x, self.white_post.point3D.x+self.white_post.vector3D.x])
            ypoints = np.array([self.white_post.point3D.y, self.white_post.point3D.y + self.white_post.vector3D.y])
            zpoints = np.array([self.white_post.point3D.z, self.white_post.point3D.z + self.white_post.vector3D.z])
            ax.plot3D(xpoints, ypoints, zpoints, 'black')

        ax.set_xlabel('X Label')
        ax.set_ylabel('Y Label')
        ax.set_zlabel('Z Label')
        #ax.set_box_aspect([x2z_ratio, y2z_ratio, 1])
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
        self.white_post = Line3D(model_robust.params[0], model_robust.params[1], isPositive=True)

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
        post_point = self.white_post.point3D
        self.rotate(np.array([post_vector.y, -1*post_vector.x, 0]))
        post_x, post_y, post_z = self.white_post.get_xyz()
        post_vector = self.white_post.vector3D
        post_point = self.white_post.point3D
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

    def lens_distort_point_cloud(self, cam_num: int) -> None:
        xyz = self.get_points()
        camera = self.cameras[cam_num]
        camera_line = camera.get_viewing_line()
        print(camera_line.get_xyz())
        print(camera_line.get_vec())
        # 1. For each point, convert p_w into p_c_3d
        #for i in range(np.shape(xyz)[0]):
        #    p = xyz[i, :]

        # 2. calculate f_scaled r_u

        # 3. Convert r_u to r_d

        # 4. project distorted p_c_2d into p_c_3d

