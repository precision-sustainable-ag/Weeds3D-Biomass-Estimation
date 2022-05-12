import matplotlib.pyplot as plt
import matplotlib.image as mpimg
img = mpimg.imread('.\\data\\DE-C4D-1S-CALIB-FIELD14SOY2ND-GX010075\\0170-color.jpg')
imgplot = plt.imshow(img)
plt.show()
