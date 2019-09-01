import cv2
import numpy as np

CHOCOLATE_MIN = np.array([10,0,0])
CHOCOLATE_MAX = np.array([20,200,150])

<<<<<<< HEAD
LEMON_MIN = np.array([22,60,0])
=======
LEMON_MIN = np.array([20,0,0])
>>>>>>> master
LEMON_MAX = np.array([37,255,255])

class ColorFilter:
    def __init__(self, flavor):

        self.filter_low = CHOCOLATE_MIN if flavor == "C" else LEMON_MIN
        self.filter_high = CHOCOLATE_MAX if flavor == "C" else LEMON_MAX

    def filter(self, img, ret_mask=False):

        mask = cv2.inRange(img, self.filter_low, self.filter_high)
        if ret_mask:
            return mask

        return cv2.bitwise_and(img, img, mask=mask)

