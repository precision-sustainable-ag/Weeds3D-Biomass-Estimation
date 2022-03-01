import utils
import glob
import os
import numpy as np
import cv2


def convert_to_contour(points):
    length = len(points)
    cnt = np.zeros([length, 1, 2])
    for i in range(length):
        cnt[i, 0, :] = points[i]
    return np.round(cnt).astype('int')

folder = "..\\Red_balls_yolo\\train\\labels"
files = glob.glob(os.path.join(folder, "*.json"))
WIDTH = 3840
HEIGHT = 2160
for file in files:
    meta = utils.load_json(file)
    print(file[-9:-5])
    txtfile = open("..\\Red_balls_yolo\\train\\labels\\"+file[-9:-5]+".txt", "w")
    for shape in meta["shapes"]:
        if shape["label"] == "redball":
            cnt = convert_to_contour(shape["points"])
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w/2
            cy = y + h/2
            cx_unit = cx/WIDTH
            cy_unit = cy/HEIGHT
            w_unit = w/WIDTH
            h_unit = h/HEIGHT
            #print("0 %0.6f %0.6f %0.6f %0.6f\n" % (cx_unit, cy_unit, w_unit, h_unit))
            txtfile.write("0 %0.6f %0.6f %0.6f %0.6f\n" % (cx_unit, cy_unit, w_unit, h_unit))
    txtfile.close()
