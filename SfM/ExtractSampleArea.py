import numpy as np
from math import radians
from pointcloud import PointCloud


ESTIMATED_ELEV_DEG = 45


pcloud = PointCloud("./data/Point_Clouds/GX010075-pmvs_options1.txt.ply")  # load point cloud
pcloud.extract_cameras("./data/GX010075-1-bundle.out") # load camera data
pcloud.rotate(radians(ESTIMATED_ELEV_DEG) * np.array([1, 0, 0]))  # Rotate to approximately flat
pcloud.translate_to_z_gt_0()
pcloud.scale_to_z_equals_1()
pcloud.find_post(min_h=0.75, max_h=0.95)
pcloud.move_post_vertical()
pcloud.center_origin_point()
sample_points = pcloud.vertical_cutout_of_points(xmin=-0.381, xmax=0.381, ymin=-0.656, ymax=0.656)
#pcloud.surface_reconstruction()
#pcloud.show_o3d(cam_pos=1)
#pcloud.show_o3d(sample_points)
#pcloud.show_mesh()
xmin=-0.5
xmax=0.5
ymin=-0.8
ymax=0.8
zmin = -10
zmax = 10
boundingBox = np.array([[xmin, ymin, zmin],[xmin, ymin, zmax],[xmin, ymax, zmin],[xmin, ymax, zmax],
                        [xmax, ymin, zmin],[xmax, ymin, zmax],[xmax, ymax, zmin],[xmax, ymax, zmax]])
#pcloud.crop_mesh(boundingBox)
#pcloud.show_mesh()
#o3d.io.write_triangle_mesh(".\\data\\Meshes\\GX010075-1-mesh.ply", pcloud.mesh, write_ascii=True)
pcloud.show_np(sample_points, show_cameras=True)
#pcloud.show_np(show_cameras=True)
#print(np.asarray(pcloud.mesh.vertices))


# Roughly scale image

