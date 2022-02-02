import os
import glob
import numpy as np

folder = "D:\\Weeds3D\\data\\"
files = glob.glob(folder+"*.MP4")
print(str(len(files))+" videos found")

with open("videos.csv", 'w') as myfile:
    #wr = csv.writer(myfile)
    #wr.writerows(files)

    for file in files:
        myfile.write(file + '\n')

