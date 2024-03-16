import json
import glob
import numpy as np
import cv2

json_files = glob.glob(".\\data\\DE-C4D-1S-CALIB-FIELD14SOY2ND-GX010075\\*.json")
color_label_key = {'Morning Glory': (255, 0, 0), 'Ragweed': (255, 255, 0), 'Foxtail': (255, 0, 255), 'White Post': (255, 255, 255), 'Soybeans': (0, 255, 0), 'Red Ball': (0, 0, 255)}
label_key = {'Morning Glory': 4, 'Ragweed': 5, 'Foxtail': 6, 'White Post': 2, 'Soybeans': 3, 'Red Ball': 1}

for file in json_files:
    image_file = file[:-4]+'png'
    image = cv2.imread(image_file)
    label_np = np.zeros([2160, 3840, 1])
    color_label_np = np.zeros([2160, 3840, 3])
    f = open(file)
    data = json.load(f)
    shapes = data['shapes']
    for shape in shapes:
        label = shape['label']
        points = shape['points']
        points = np.int0(points)
        cv2.drawContours(color_label_np, [points], 0, color_label_key[label], -1)
        cv2.drawContours(label_np, [points], 0, label_key[label], -1)

    color_label_np = cv2.resize(color_label_np, (1920, 1080))
    label_np = cv2.resize(label_np, (1920, 1080), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite(file[:-5]+'-color.jpg', color_label_np)
    cv2.imwrite(file[:-5]+'-label.png', label_np)
f.close()
