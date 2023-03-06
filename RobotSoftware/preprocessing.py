"""
Preprocessing image after detect object from yolov5 model
Method: - mask the area of bounding-box object into green conveyor background
        - use some image processing algorithm: otsu, canny, ... to threshold tho object
        - condition: Brightness of environment is not too dark or too bright, then result of
        preprocessing will be good
"""

import cv2
import numpy as np
import joblib

kernel_mean = 1 / 9 * np.array([[1, 1, 1],
                                [1, 1, 1],
                                [1, 1, 1]])

CONVEYOR_BG_MASK = joblib.load('data/bg_mask.save')  # 640x480


def preprocessing_img(img, bb: tuple, half_size=(320, 240)):
    # resize the image
    bb = (int(bb[0] * half_size[0] / img.shape[1]),
          int(bb[1] * half_size[1] / img.shape[0]),
          int(bb[2] * half_size[0] / img.shape[1]),
          int(bb[3] * half_size[1] / img.shape[0]))

    img = cv2.resize(img, half_size)
    img_org = img.copy()

    # mask the detected image to the bg mask
    mask = np.zeros_like(img)
    cv2.rectangle(mask, (bb[0], bb[1]), (bb[2], bb[3]), (255, 255, 255), -1)
    masked_img = cv2.bitwise_and(img, mask)  # cut the obj from main img
    mask_inv = cv2.bitwise_not(mask)
    masked_conveyor_bg = cv2.bitwise_and(cv2.resize(CONVEYOR_BG_MASK, half_size), mask_inv)  # cut the bg
    conveyor_with_obj = cv2.add(masked_conveyor_bg, masked_img)

    # smoothing new masked image
    conveyor_with_obj = cv2.filter2D(conveyor_with_obj, -1, kernel_mean)

    # convert to gray image
    img_gray = cv2.cvtColor(conveyor_with_obj, cv2.COLOR_BGR2GRAY)

    # threshold otsu and noise filter to split the object
    _, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    thresh = cv2.medianBlur(thresh, 37)

    # apply a Gaussian blur
    blur = cv2.GaussianBlur(img_gray, (3, 3), 0)

    # apply Canny edge detection to find the edge of the object
    edges = cv2.Canny(blur, 50, 200)

    # combine thresh and edges and apply some filters to complete the output image
    output_Step1 = cv2.bitwise_xor(thresh, edges)
    output_Step2 = cv2.filter2D(output_Step1, -1, kernel_mean)
    _, output_Step3 = cv2.threshold(output_Step2, 240, 255, cv2.THRESH_BINARY)
    output_Step4 = cv2.medianBlur(output_Step3, 5)

    return img_org, output_Step4
