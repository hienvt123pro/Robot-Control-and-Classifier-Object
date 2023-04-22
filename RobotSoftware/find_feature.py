"""
Find center of object, as the pick-place for robot end-effector
Find features (d1, d2, d3) to input into the model size ANN
Find features of logo (location, direction)
"""

import cv2
import numpy as np
from scipy.spatial.distance import cdist
import math


class FindFeatures:
    def __init__(self):
        self.cor_part1 = []
        self.cor_part2 = []
        self.cor_part3 = []
        self.vec_logo = []
        self.center_obj = np.zeros((1, 2))
        self.center_logo = [0, 0]
        self.directObject = ''
        self.y_lines = [0, 0]
        self.x_lines = [0, 0]
        self.isShowSizeFeats = True
        self.isShowLogoFeats = False

    def find_size_features(self, img_org, img_contour):
        img_origin = cv2.resize(img_org, (640, 480))
        del img_org

        self.center_obj = np.zeros((1, 2))
        d1, d2, d3 = 0, 0, 0

        # find and draw contours
        contours, _ = cv2.findContours(cv2.resize(img_contour, (640, 480)), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        del img_contour
        try:
            for ct in contours:
                # choose the best contour, which is suitable for the object
                if 6500 < cv2.contourArea(ct[0:round(len(ct)/2), :]) < 35000 and 230 < len(ct) < 480:
                    # smoothing the contour
                    best_contour = cv2.convexHull(ct)

                    # take points of contour
                    cor_parts = best_contour[:, 0, :]

                    # check object and split 3 part of object
                    x, y, w, h = cv2.boundingRect(best_contour)
                    parts = [[] for _ in range(3)]
                    if h > w and 1 <= h/w <= 2.2:
                        self.directObject = "vertical"
                        split_lines = round(h / 3, 2)
                        lines = [y + split_lines, y + 2 * split_lines]
                        self.y_lines = lines
                        for p in cor_parts:
                            i = 0 if p[1] <= lines[0] else 1 if p[1] <= lines[1] else 2
                            parts[i].append(p)
                    elif h < w and 1 <= w/h <= 2.2:
                        self.directObject = "horizontal"
                        split_lines = round(w / 3, 2)
                        lines = [x + split_lines, x + 2 * split_lines]
                        self.x_lines = lines
                        for p in cor_parts:
                            i = 0 if p[0] <= lines[0] else 1 if p[0] <= lines[1] else 2
                            parts[i].append(p)
                    else:
                        break

                    # set of contour points of 3 parts of object
                    self.cor_part1, self.cor_part2, self.cor_part3 = map(np.array, parts)
                    if self.cor_part2 is None or self.cor_part1 is None or self.cor_part3 is None:
                        del parts
                        break

                    # draw the best contour
                    approx = cv2.approxPolyDP(best_contour, 0.001 * cv2.arcLength(best_contour, True), True)
                    cv2.drawContours(img_origin, [approx], 0, (255, 0, 0), 3)

                    # find center for object from the best contour
                    M = cv2.moments(best_contour)
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    cv2.circle(img_origin, (cX, cY), 8, (255, 0, 0), -1)
                    self.center_obj[:, 0] = cX
                    self.center_obj[:, 1] = cY

                    # find peak of 3 parts of object
                    # part 1: find the longest point
                    x1, y1 = self.cal_dist(self.center_obj, self.cor_part1, option="max")
                    # part 2: find the closest point
                    x2, y2 = self.cal_dist(self.center_obj, self.cor_part2, option="min")
                    # part 3: find the longest point
                    x3, y3 = self.cal_dist(self.center_obj, self.cor_part3, option="max")

                    # calculate for 3 features
                    d1 = math.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2)
                    d2 = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                    d3 = math.sqrt((x2 - x3) ** 2 + (y2 - y3) ** 2)

                    if self.isShowSizeFeats:
                        cv2.line(img_origin, (x1, y1), (x3, y3), (0, 0, 255), thickness=2)
                        cv2.line(img_origin, (x1, y1), (x2, y2), (0, 0, 255), thickness=2)
                        cv2.line(img_origin, (x2, y2), (x3, y3), (0, 0, 255), thickness=2)

                    if self.isShowLogoFeats:
                        cv2.circle(img_origin, (self.center_logo[0], self.center_logo[1]), 8, (255, 0, 0), -1)
                        cv2.line(img_origin, (cX, cY), (self.center_logo[0], self.center_logo[1]), (0, 255, 0), thickness=2)

        except Exception as e:
            print(e, '-find feature')

        return img_origin, self.center_obj, d1, d2, d3

    # calculate distance from center point to peak point
    @staticmethod
    def cal_dist(center, contours, option):
        D = cdist(contours, center)
        index = np.argmax(D) if option == 'max' else np.argmin(D)
        point = contours[index]
        return tuple(point)

    def find_logo_locate_features(self):
        if len(self.cor_part1) > 0:
            areas = map(cv2.contourArea, [self.cor_part1, self.cor_part3])
            area_part1, area_part3 = areas

            # find bigger part of object, if logo locate at this part, then it's error object
            if area_part1 < area_part3:
                bigger_part_obj = "part3"
            else:
                bigger_part_obj = "part1"

            if bigger_part_obj == "part3":
                if self.directObject == "vertical" and self.center_logo[1] > self.y_lines[1]:
                    return 'error'
                elif self.directObject == "horizontal" and self.center_logo[0] > self.x_lines[1]:
                    return 'error'
                return 'not error'
            else:
                if self.directObject == "vertical" and self.center_logo[1] < self.y_lines[0]:
                    return 'error'
                elif self.directObject == "horizontal" and self.center_logo[0] < self.x_lines[0]:
                    return 'error'
                return 'not error'

    def find_logo_direct_features(self, lines):
        # filter on lines to find the desired line => vecto of logo
        if lines is not None and len(lines) > 0:
            desired_lines = []
            for i, line in enumerate(lines):
                x1, y1, x2, y2 = line[0]
                slope = abs((y2 - y1) / (x2 - x1))
                if slope < 0.5:
                    desired_lines.append([x1, y1, x2, y2])
            for line in desired_lines:
                x1, y1, x2, y2 = line
                self.vec_logo = [(x2 - x1), (y2 - y1)]
                return self.vec_logo, x1, y1, x2, y2
        return [0, 0], 0, 0, 0, 0


ffeats = FindFeatures()
