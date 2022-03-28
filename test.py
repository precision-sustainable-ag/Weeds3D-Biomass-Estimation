import numpy as np

data = np.load(".\\data\\GP51451840-CALIB-01-GX010001.npz")
for k in data.keys():
    print(k)

print(data['focalLength'])
print(data['distCoeff'])
print(data['intrinsic_matrix'])
