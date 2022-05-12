import os
import cv2
import numpy as np
import matplotlib.pyplot as plt

file1 = open('.\\data\\GX010075-1-bundle.out', 'r')
Lines = file1.readlines()
num_cameras = int(Lines[1].split()[0])
num_points = int(Lines[1].split()[1])
print(num_points)
color_label_key = {'Morning Glory': (255, 0, 0), 'Ragweed': (255, 255, 0), 'Foxtail': (255, 0, 255), 'White Post': (255, 255, 255), 'Soybeans': (0, 255, 0), 'Red Ball': (0, 0, 255)}
label_key = {'Morning Glory': 4, 'Ragweed': 5, 'Foxtail': 6, 'White Post': 2, 'Soybeans': 3, 'Red Ball': 1}
start_line = num_cameras*5+2
xs = np.zeros([num_points])
ys = np.zeros([num_points])
zs = np.zeros([num_points])
count = 0
fig = plt.figure()
ax = fig.add_subplot(projection='3d')
for i in range(start_line, start_line+num_points*3, 3):
    #i = start_line
    line_split1 = Lines[i].split()
    xyz = [float(line_split1[0]), float(line_split1[1]), float(line_split1[2])]
    if xyz[2] > -5:
        xs[count] = xyz[0]
        ys[count] = xyz[1]
        zs[count] = xyz[2]
    line_split2 = Lines[i+1].split()
    color = [int(line_split2[0]), int(line_split2[1]), int(line_split2[2])]
    count += 1

print(xs)
print(ys)
print(zs)
ax.scatter(xs, ys, zs)
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
plt.show()
