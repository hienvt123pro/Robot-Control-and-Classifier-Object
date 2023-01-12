import cv2
import numpy as np
from scipy.spatial.distance import cdist
import math


def find_features(img_org, img_contour):
    center = np.zeros((1, 2))
    d1, d2, d3 = 0, 0, 0

    # find and draw contours
    contours, _ = cv2.findContours(img_contour, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    try:
        # find the best contour
        for ct in contours:
            if 4000 < cv2.contourArea(ct[0:round(len(ct) / 2), :]) < 30000 and 200 < len(ct) < 400:
                # smoothing contour
                best_contour = cv2.convexHull(ct)
                cor_parts = best_contour[:, 0, :]

                # check object and split 3 part of object
                cor_part1, cor_part2, cor_part3 = [], [], []
                x, y, w, h = cv2.boundingRect(best_contour)
                if h > w and 1 <= h/w <= 2.2:
                    split_lines = h / 3
                    yline1 = y + split_lines
                    yline2 = y + 2 * split_lines
                    for p in cor_parts:
                        if p[1] <= int(yline1):
                            cor_part1.append(p)
                        elif p[1] <= int(yline2):
                            cor_part2.append(p)
                        else:
                            cor_part3.append(p)
                elif h < w and 1 <= w/h <= 2.2:
                    split_lines = w / 3
                    xline1 = x + split_lines
                    xline2 = x + 2 * split_lines
                    for p in cor_parts:
                        if p[0] <= int(xline1):
                            cor_part1.append(p)
                        elif p[0] <= int(xline2):
                            cor_part2.append(p)
                        else:
                            cor_part3.append(p)
                else:
                    break

                # draw the best contour
                approx = cv2.approxPolyDP(best_contour, 0.001 * cv2.arcLength(best_contour, True), True)
                cv2.drawContours(img_org, [approx], 0, (255, 0, 0), 2)

                # find center of object from the best contour
                M = cv2.moments(best_contour)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                cv2.circle(img_org, (cX, cY), 4, (255, 0, 0), -1)
                center[:, 0] = cX
                center[:, 1] = cY

                # contour of 3 parts of object
                cor_part1 = np.array(cor_part1)
                cor_part2 = np.array(cor_part2)
                cor_part3 = np.array(cor_part3)

                # find peak of 3 parts of object
                # part 1
                x1, y1 = cal_dist(center, cor_part1, choose="max")
                # part 2
                x2, y2 = cal_dist(center, cor_part2, choose="min")
                # part 3
                x3, y3 = cal_dist(center, cor_part3, choose="max")

                # 3 features
                d1 = math.sqrt((x3 - x1) ** 2 + (y3 - y1) ** 2)
                d2 = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                d3 = math.sqrt((x2 - x3) ** 2 + (y2 - y3) ** 2)

                cv2.line(img_org, (x1, y1), (x3, y3), (0, 0, 255), thickness=1)
                cv2.line(img_org, (x1, y1), (x2, y2), (0, 0, 255), thickness=1)
                cv2.line(img_org, (x2, y2), (x3, y3), (0, 0, 255), thickness=1)

    except Exception as e:
        print(e, '- undefined object!')
        pass

    return img_org, center, d1, d2, d3


def cal_dist(center, contours, choose):
    if choose == "max":
        D = cdist(contours, center)
        maxp = np.argmax(D)
        point = contours[maxp, :]
        x = point[0]
        y = point[1]
        return x, y
    elif choose == "min":
        D = cdist(contours, center)
        minp = np.argmin(D)
        point = contours[minp, :]
        x = point[0]
        y = point[1]
        return x, y
