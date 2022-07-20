import numpy as np

data = np.load(".\\data\\GP24667519-CALIB-02-GX010170.npz")
for k in data.keys():
    print(k)
print(data['focalLength'])
print(data['distCoeff'])
print(data['intrinsic_matrix'])
focalLength = float(data['focalLength'])
scaledFocalLength = focalLength * 1920 / 6.17
print(scaledFocalLength)
