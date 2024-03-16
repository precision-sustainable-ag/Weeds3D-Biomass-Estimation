import numpy as np
import cv2

def red_hue_mask(image):
    HLS = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    posRedHueMask = (HLS[:, :, 0] < 10).astype('uint8')
    negRedHueMask = (HLS[:, :, 0] > 170).astype('uint8')
    redHueMask = np.bitwise_or(posRedHueMask, negRedHueMask).astype('uint8')
    return redHueMask

def HLS_white_mask(image):
    HLS = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    lightMask = (HLS[:, :, 1] > 240).astype('uint8')
    SaturationMask = (HLS[:, :, 2] < 20).astype('uint8')
    whiteMask = np.bitwise_or(lightMask, SaturationMask).astype('uint8')
    return whiteMask


def excess_red_mask(image):
    b = image[:, :, 0]
    g = image[:, :, 1]
    r = image[:, :, 2]
    ExR = 2 * r.astype('int') - g.astype('int') - b.astype('int')
    ExR_mask = (ExR > 80).astype('uint8')
    return ExR_mask

def two_method_mask(image, filter1, filter2):
    mask1 = filter1(image)
    mask2 = filter2(image)
    combined_mask = np.bitwise_and(mask1, mask2).astype('uint8')
    return combined_mask
