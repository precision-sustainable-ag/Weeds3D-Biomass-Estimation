# Standard Libraries
import numpy as np
from math import radians

# Custom Libraries
from SfM.pointcloud import PointCloud

# Config file
import SfM.config as config

pcloud = PointCloud("./data/Point_Clouds/GX010075-pmvs_options1.txt.ply")  # load point cloud
pcloud.extract_cameras("./data/GX010075-1-bundle.out") # load camera data
pcloud.rotate(radians(config.ESTIMATED_ELEV_DEG) * np.array([1, 0, 0]))  # Rotate to approximately flat
pcloud.translate_to_z_gt_0()
pcloud.scale_to_z_equals_1()
pcloud.find_post(min_h=config.min_h, max_h=config.max_h)
pcloud.move_post_vertical()
pcloud.center_origin_point()
sample_points = pcloud.vertical_cutout_of_points(xmin=config.xmin, xmax=config.xmax, ymin=config.ymin, ymax=config.ymax)
#pcloud.surface_reconstruction()
#pcloud.show_o3d(cam_pos=1)
#pcloud.show_o3d(sample_points)
#pcloud.show_mesh()
boundingBox = np.array([[config.bbox_xmin, config.bbox_ymin, config.bbox_zmin],[config.bbox_xmin, config.bbox_ymin, config.bbox_zmax],[config.bbox_xmin, config.bbox_ymax, config.bbox_zmin],[config.bbox_xmin, config.bbox_ymax, config.bbox_zmax],
                        [config.bbox_xmax, config.bbox_ymin, config.bbox_zmin],[config.bbox_xmax, config.bbox_ymin, config.bbox_zmax],[config.bbox_xmax, config.bbox_ymax, config.bbox_zmin],[config.bbox_xmax, config.bbox_ymax, config.bbox_zmax]])
#pcloud.crop_mesh(boundingBox)
#pcloud.show_mesh()
#o3d.io.write_triangle_mesh(".\\data\\Meshes\\GX010075-1-mesh.ply", pcloud.mesh, write_ascii=True)
pcloud.show_np(sample_points, show_cameras=True)
#pcloud.show_np(show_cameras=True)
#print(np.asarray(pcloud.mesh.vertices))


# Roughly scale image

