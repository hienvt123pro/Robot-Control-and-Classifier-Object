import cv2
import numpy as np
import imutils

# laplacian kernel
kernel_laplacian = np.array([[0, 2, 0],
                             [2, -8, 2],
                             [0, 2, 0]])

kernel_mean = 1 / 9 * np.array([[1, 1, 1],
                                [1, 1, 1],
                                [1, 1, 1]])


def preprocessing_img(img):
    # resize image
    img = imutils.resize(img, width=440, height=590)
    img_org = img

    # convert img RGB to gray
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # median filtering
    img_median = cv2.medianBlur(img_gray, 19)

    # histogram equalization
    img_equ = cv2.equalizeHist(img_median)
    clahe = cv2.createCLAHE(clipLimit=3.2, tileGridSize=(2, 1))
    img_high_contrast = clahe.apply(img_equ)
    img_high_contrast = cv2.medianBlur(img_high_contrast, 25)

    # use laplacian to find detect edge
    image_sharp = cv2.filter2D(src=img_median, ddepth=-1, kernel=kernel_laplacian)

    # threshold edge
    _, thresh_sharp = cv2.threshold(image_sharp, 30, 255, cv2.THRESH_BINARY)

    # threshold object
    _, thresh_object = cv2.threshold(img_high_contrast, 180, 255, cv2.THRESH_BINARY)

    # full object image threshold
    img_contour = thresh_object + thresh_sharp

    # mean filtering
    img_contour = cv2.filter2D(img_contour, -1, kernel_mean)

    # completed threshold object
    _, img_contour = cv2.threshold(img_contour, 240, 255, cv2.THRESH_BINARY)

    return img_org, img_contour
