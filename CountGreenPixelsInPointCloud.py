from pointcloud import PointCloud
import numpy as np
from skimage import color

HUE_MIN = 80
HUE_MAX = 170
ExG_CUTOFF = 50
file = ".\\data\\Point_Clouds\\GX010075-pmvs_options1.txt.ply"

pcloud_hsv = PointCloud(file)  # load point cloud
pcloud_ExG = PointCloud(file)
rgb_colors = np.asarray(pcloud_hsv.data.colors)
hsv_colors = color.rgb2hsv(rgb_colors)
count_pix = np.shape(rgb_colors)[0]
count_hue = count_pix
count_ExG = count_pix
for i in range(np.shape(hsv_colors)[0]):
    if hsv_colors[i, 0] < (HUE_MIN/360) or hsv_colors[i, 0] > (HUE_MAX/360):
        count_hue -= 1
        hsv_colors[i, 0] = 300/360
        hsv_colors[i, 1] = 1.0
        hsv_colors[i, 2] = 1.0

    if (2*rgb_colors[i, 1] - rgb_colors[i, 0] - rgb_colors[i, 2]) < ExG_CUTOFF:
        count_ExG -= 1
        rgb_colors[i, 0] = 1.0
        rgb_colors[i, 1] = 0.0
        rgb_colors[i, 2] = 1.0

pcloud_hsv.set_colors(color.hsv2rgb(hsv_colors))
pcloud_hsv.show_o3d()
pcloud_ExG.set_colors(rgb_colors)
pcloud_ExG.show_o3d()
print("Number of green pixels (" + str(HUE_MIN) + " < Hue < " + str(HUE_MAX) + "): " + str(count_hue))
print("Number of green pixels (ExG > " + str(ExG_CUTOFF) + "): " + str(count_ExG))
print("total pixels: " + str(np.shape(rgb_colors)[0]))


