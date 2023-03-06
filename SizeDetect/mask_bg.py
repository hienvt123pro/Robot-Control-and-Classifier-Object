""" create a mask of green conveyor background """
import cv2
import numpy as np
import joblib

kernel_mean = 1 / 9 * np.array([[1, 1, 1],
                                [1, 1, 1],
                                [1, 1, 1]])

img = cv2.imread("green_bg.jpg")
img = cv2.resize(img, (640, 480))
img = cv2.filter2D(img, -1, kernel_mean)
cv2.imshow('bg', img)
cv2.waitKey()
joblib.dump(img, 'output_models/bg_mask.save')
