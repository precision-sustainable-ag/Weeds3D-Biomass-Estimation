#!/usr/bin/env python3
#!/home/azureuser/.venv/python3-cv/lib/python3.6/site-packages

import sys
sys.path.append("/home/azureuser/.venv/python3-cv/lib/python3.6/site-packages")

import cv2
import numpy as np
import os

f = open('/home/azureuser/data/video/DE/DE-C4D-1S-CALIB-FIELD14SOY2ND-GX010075/clustering/1/bundle/bundle-seg.out', 'r')
Lines = f.readlines()
num_cameras = int(Lines[1].split()[0])

for i in range(2, 2+num_cameras*5, 5):
    focal_length = float(Lines[i].split()[0])
    k0 = float(Lines[i].split()[1])
    k1 = float(Lines[i].split()[2])
    rotation = np.zeros([3, 3])
    rotation[0, :] = [float(Lines[i+1].split()[0]), float(Lines[i+1].split()[1]), float(Lines[i+1].split()[2])]
    rotation[1, :] = [float(Lines[i+2].split()[0]), float(Lines[i+2].split()[1]), float(Lines[i+2].split()[2])]
    rotation[2, :] = [float(Lines[i+3].split()[0]), float(Lines[i+3].split()[1]), float(Lines[i+3].split()[2])]
    translation = np.array([float(Lines[i+4].split()[0]), float(Lines[i+4].split()[1]), float(Lines[i+4].split()[2])])
    print("focal length: "+str(focal_length))
    print(rotation)

f.close()
