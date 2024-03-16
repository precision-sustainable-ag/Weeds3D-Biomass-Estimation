import os
import cv2
import numpy as np


file1 = open('.\\data\\GX010075-1-bundle.out', 'r')
folder = '.\\data\\DE-C4D-1S-CALIB-FIELD14SOY2ND-GX010075'
start_image = 150
color_label_key = {'Morning Glory': (255, 0, 0), 'Ragweed': (255, 255, 0), 'Foxtail': (255, 0, 255), 'White Post': (255, 255, 255), 'Soybeans': (0, 255, 0), 'Red Ball': (0, 0, 255)}
color_label_map = [[0, 0, 0], [0, 0, 255], [255, 255, 255], [0, 255, 0], [255, 0, 0], [255, 255, 0], [255, 0, 255]]
label_key = {'Morning Glory': 4, 'Ragweed': 5, 'Foxtail': 6, 'White Post': 2, 'Soybeans': 3, 'Red Ball': 1}
Lines = file1.readlines()
num_cameras = int(Lines[1].split()[0])
seg_images = []
for k in range(start_image, start_image+num_cameras*10, 10):
    image_filepath = os.path.join(folder, str(k).zfill(4)+'-label.png')
    image = cv2.imread(image_filepath, cv2.IMREAD_GRAYSCALE)
    seg_images.append(image)

num_points = int(Lines[1].split()[1])
start_line = num_cameras*5+2

# Load beginning lines into outfile
fileout = open('.\\data\\GX010075-1-bundle-seg.out', 'w')
for k in range(start_line):
    fileout.write(Lines[k])

for i in range(start_line, start_line+num_points*3, 3):
    #i = start_line
    fileout.write(Lines[i])
    line_split1 = Lines[i].split()
    xyz = [float(line_split1[0]), float(line_split1[1]), float(line_split1[2])]
    line_split2 = Lines[i+1].split()
    color = [line_split2[0], line_split2[1], line_split2[2]]
    line_split3 = Lines[i+2].split()
    num_matches = int(line_split3[0])
    label_votes = np.zeros([7])
    for j in range(1, 4*num_matches, 4):
        cam_num = int(line_split3[j])
        xy_centered = [float(line_split3[j+2]), float(line_split3[j+3])]
        xy_corrected = [round(960+xy_centered[0]), round(540-xy_centered[1])]
        lab = seg_images[cam_num][xy_corrected[1]][xy_corrected[0]]
        label_votes[lab] += 1

    if np.sum(label_votes[1:]) > 1:
        final_label = np.argmax(label_votes[1:]) + 1
    else:
        final_label = 0
    #final_label = np.argmax(label_votes)


    fileout.write(str(color_label_map[final_label][0])+" "+str(color_label_map[final_label][1])+" "+str(color_label_map[final_label][2])+"\n")
    fileout.write(Lines[i+2])
fileout.close()
file1.close()




