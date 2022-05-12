import os
import cv2
import numpy as np

class CamPoint:
    x: float
    y: float
    label: int
    id: int

    def __init__(self, id, label, x, y):
        self.x = x
        self.y = y
        self.label = label
        self.id = id


class Vertex:
    x: float
    y: float
    z: float
    r: int
    g: int
    b: int
    cam_points: list

    def __init__(self, x, y, z, r, g, b):
        self.x = x
        self.y = y
        self.z = z
        self.r = r
        self.g = g
        self.b = b
        self.cam_points = []

    def add_campoint(self, campoint):
        self.cam_points.append(campoint)

    def isMatch(self, other):
        if (abs(self.x - other.x) < 0.00001) and (abs(self.y - other.y) < 0.00001) and (abs(self.z - other.z) < 0.00001):
            return True
        else:
            return False

    def return_labels(self):
        num = len(self.cam_points)
        labels = np.zeros([num])
        for i in range(num):
            labels[i] = self.cam_points[i].label
        return labels

    def return_coords(self):
        num = len(self.cam_points)
        coords = []
        for i in range(num):
            coords.append([self.cam_points[i].x, self.cam_points[i].y])
        return coords


class SfM:
    vertecies: list

    def __init__(self):
        self.vertecies = []

    def add(self, vertex: Vertex):
        self.vertecies.append(vertex)

    def find_match(self, vertex: Vertex):
        num = len(self.vertecies)
        for i in range(num):
            if self.vertecies[i].isMatch(vertex):
                return i
        return -1

    def return_labels(self, index):
        return self.vertecies[index].return_labels()

    def return_coords(self, index):
        return self.vertecies[index].return_coords()







file1 = open('.\\data\\GX010075-1-bundle.rd.out', 'r')

start_image = 150
Lines = file1.readlines()
num_cameras = int(Lines[1].split()[0])
num_vertecies = int(Lines[1].split()[1])
seg_sfm = SfM()
folder = '.\\data\\DE-C4D-1S-CALIB-FIELD14SOY2ND-GX010075'
seg_images = []
for k in range(start_image, start_image+num_cameras*10, 10):
    image_filepath = os.path.join(folder, str(k).zfill(4)+'-label.png')
    image = cv2.imread(image_filepath, cv2.IMREAD_GRAYSCALE)
    seg_images.append(image)


print(Lines[num_cameras*5+2])
for i in range(num_cameras*5+2, num_cameras*5+2+num_vertecies*3, 3):
    linexyz = Lines[i].split()
    linergb = Lines[i+1].split()
    linematches = Lines[i+2].split()
    num_matches = int(linematches[0])
    x = float(linexyz[0])
    y = float(linexyz[1])
    z = float(linexyz[2])
    r = int(linergb[0])
    g = int(linergb[1])
    b = int(linergb[2])
    v = Vertex(x, y, z, r, g, b)
    for j in range(1, 4*num_matches, 4):
        cam_num = int(linematches[j])
        xy_centered = [float(linematches[j+2]), float(linematches[j+3])]
        xy_corrected = [round(960+xy_centered[0]), round(540-xy_centered[1])]
        label = seg_images[cam_num][xy_corrected[1]][xy_corrected[0]]
        camPoint = CamPoint(cam_num, label, xy_corrected[0], xy_corrected[1])
        v.add_campoint(camPoint)

    seg_sfm.add(v)

file2 = open('.\\data\\GX010075-1-bundle1a.ply', 'r')
Lines = file2.readlines()
num_vertecies = int(Lines[3].split()[2])
for i in range(14, num_vertecies+14):
    line = Lines[i].split()
    x = float(line[0])
    y = float(line[1])
    z = float(line[2])
    r = int(line[3])
    g = int(line[4])
    b = int(line[5])
    v = Vertex(x, y, z, r, g, b)
    k = seg_sfm.find_match(v)
    print(seg_sfm.return_labels(k))


