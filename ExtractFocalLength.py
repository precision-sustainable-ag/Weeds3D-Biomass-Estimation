import numpy as np

data = np.load(".\\data\\GP51457457-CALIB-01-GX010001.npz")
focalLength = float(data['focalLength'])
scaledFocalLength = focalLength * 1920 / 6.17
print(scaledFocalLength)
