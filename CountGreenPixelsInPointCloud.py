from pointcloud import PointCloud
import numpy as np
from skimage import color

HUE_MIN = 80
HUE_MAX = 170
ExG_CUTOFF = 0

pcloud = PointCloud(".\\data\\Point_Clouds\\GX010075-pmvs_options1.txt.ply")  # load point cloud
rgb_colors = np.asarray(pcloud.data.colors)
hsv_colors = color.rgb2hsv(rgb_colors)
count_hue = 0
count_ExG = 0
for i in range(np.shape(hsv_colors)[0]):
    if hsv_colors[i,0] > (HUE_MIN/360) and hsv_colors[i,0] < (HUE_MAX/360):
        count_hue += 1

    if (2*rgb_colors[i,1] - rgb_colors[i,0] - rgb_colors[i,2]) > ExG_CUTOFF:
        count_ExG += 1


print("Number of green pixels (" + str(HUE_MIN) + " < Hue < " + str(HUE_MAX) + "): " + str(count_hue))
print("Number of green pixels (ExG > " + str(ExG_CUTOFF) + "): " + str(count_ExG))


