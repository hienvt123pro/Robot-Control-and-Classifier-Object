"""
Provide functions, classes for processing models (yolov5, size ANN, color CNN, logo ANN depend on HoG feature)
"""

from find_feature import ffeats
from preprocessing import preprocessing_obj
# from preprocessing import preprocessing_logo
from size_uncertainty import SizeProbabilistic
import load_model_json
import joblib
import cv2
import numpy as np
import torch
import math


# -----------------------------------------------------------
# 1. Detect object process
class ObjectProcess:
    def __init__(self):
        self.obj_model = torch.hub.load('ultralytics/yolov5', 'custom', path='models/obj.pt', force_reload=True)
        self.obj_model.iou = 0.5
        self.obj_model.conf = 0.84
        self.obj_classes = self.obj_model.names
        self.waited_capture = cv2.imread("GUI/load_image.png")

    @staticmethod
    def yolov5_detect(I, model):
        result = model([I], size=320)
        # cordinates is: x1, y2, x2, y2, confidence
        labels, cordinates = result.xyxyn[0][:, -1], result.xyxyn[0][:, :-1]
        return labels, cordinates

    def yolo_bb_obj(self, result, frame, classes):
        labels, cord = result
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        cropped_logo = self.waited_capture
        bbox = (0, 0, 0, 0)
        center_logo = [0, 0]
        for i in range(len(labels)):
            row = cord[i]
            x1, y1, x2, y2 = int(row[0] * x_shape), int(row[1] * y_shape), int(row[2] * x_shape), int(row[3] * y_shape)
            cls = classes[int(labels[i])]
            if cls == 'logo':
                cropped_logo = frame[y1 + 3:y2 - 3, x1 + 3:x2 - 3]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f" {round(float(row[4]), 2)}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 2)
                center_logo = [int((x1 + x2) / 2), int((y1 + y2) / 2)]
            elif cls == 'dep':
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, f" {round(float(row[4]), 2)}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 2)
                bbox = (x1, y1, x2, y2)
        return frame, cropped_logo, bbox, center_logo

    def obj_detector_process(self, input_queue):
        try:
            img_org = input_queue.copy()
            results = self.yolov5_detect(cv2.cvtColor(input_queue, cv2.COLOR_BGR2RGB), model=self.obj_model)

            # detect both of logo and obj: {15:obj, 16:logo}
            if 15 in results[0] and 16 in results[0]:
                detected_frame, crop_logo, bb, centerLogo = self.yolo_bb_obj(results, img_org, classes=self.obj_classes)
                crop_logo = np.ascontiguousarray(crop_logo)
                crop_logo = cv2.resize(crop_logo, (124, 124))
                return True, detected_frame, crop_logo, bb, centerLogo
            else:
                return False, img_org, self.waited_capture, (0, 0, 0, 0), [0, 0]
        except Exception as e:
            print(e, "-obj process")


# -----------------------------------------------------------
# 2. Detect size and color of object process
class SizeAndColorProcess:
    def __init__(self):
        # probabilistic algorithm for detect size continuous
        self.uncertain_algorithm = SizeProbabilistic()

        # load standard scaler from size training data
        self.sz_scaler = joblib.load('data/scaler_size.save')

        # load size model
        self.size_model = load_model_json.load('models/size.json', "models/size.h5")

        # size calib factors
        self.cf1, self.cf2, self.cf3 = 1, 1, 1

        # load color model
        self.color_model = load_model_json.load('models/color.json', "models/color.h5")

        # threshold for performance
        self.SIZE_MODEL_THRESHOLD = 0.5
        self.COLOR_MODEL_THRESHOLD = 0.5

    # result functions for size and color model
    @staticmethod
    def size_result(y):
        size_dict = {0: '30', 1: '31', 2: '32', 3: 'error'}
        return size_dict.get(y, None)

    @staticmethod
    def color_result(y):
        color_dict = {0: 'Red', 1: 'Yellow', 2: 'error'}
        return color_dict.get(y, None)

    # preprocessing data for color model
    @staticmethod
    def transform_color(I, c_X, c_Y):
        cen_img = I[c_Y - 40:c_Y + 40, c_X - 40:c_X + 40]
        img_hsv = cv2.cvtColor(cen_img, cv2.COLOR_BGR2HSV)
        img_rz = cv2.resize(img_hsv, (80, 80))
        img_output = img_rz / 255
        return img_output.reshape(-1, 80, 80, 3)

    # detect both size and color
    def detect_size_color(self, I, bb_x1, bb_y1, bb_x2, bb_y2):
        img_org, img_contour = preprocessing_obj(I, (bb_x1, bb_y1, bb_x2, bb_y2))
        img, center, d1, d2, d3 = ffeats.find_size_features(img_org, img_contour)

        # calib features
        d1, d2, d3 = d1 * self.cf1, d2 * self.cf2, d3 * self.cf3

        size, color = '', ''
        cX, cY = 0, 0

        if d1 == 0:
            return img, size, color, cX, cY
        try:
            # detect size
            data = [tuple([d1, d2, d3])]
            predict_size = self.size_model.predict(self.sz_scaler.transform(data), verbose=0)
            prob_predict = self.uncertain_algorithm.predict(predict_size)
            y_pred = np.argmax(prob_predict)
            size = self.size_result(y_pred)
            if prob_predict[0, y_pred] < self.SIZE_MODEL_THRESHOLD:
                size = 'error'

            # detect color
            cX = int(center[0, 0])
            cY = int(center[0, 1])
            bigger_part_obj = ffeats.find_the_bigger_part()
            center_index_x, center_index_y = 0, 0
            if bigger_part_obj:
                if bigger_part_obj == "part3":
                    if ffeats.directObject == "vertical":
                        center_index_y = 50
                        center_index_x = 0
                    elif ffeats.directObject == "horizontal":
                        center_index_y = 0
                        center_index_x = 50
                else:
                    if ffeats.directObject == "vertical":
                        center_index_y = -50
                        center_index_x = 0
                    elif ffeats.directObject == "horizontal":
                        center_index_y = 0
                        center_index_x = -50

            if ffeats.isShowColorFeats:
                cv2.rectangle(img, (cX + center_index_x - 40, cY + center_index_y - 40),
                              (cX + center_index_x + 40, cY + center_index_y + 40), (0, 255, 0), 2)

            predict_color = self.color_model.predict(self.transform_color(img, cX + center_index_x, cY + center_index_y),
                                                     verbose=0)
            y_pred = np.argmax(predict_color)
            color = self.color_result(y_pred)
            if predict_color[0, y_pred] < self.COLOR_MODEL_THRESHOLD:
                color = 'error'

        except Exception as e:
            print(e, "-unclear object")

        return img, size, color, cX, cY


# -----------------------------------------------------------
# 3. Draw the working area
def draw_working_area(img, corner_points: list):
    rect = cv2.minAreaRect(np.array(corner_points))
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    cv2.drawContours(img, [box], 0, (0, 0, 255), 2)
    return img


# -----------------------------------------------------------
# 4. Save and read the image
def save_image(path, image):
    cv2.imwrite(path, image)


def read_image(path):
    return cv2.imread(path)


# -----------------------------------------------------------
# 5. Detect logo of object process
class LogoProcess:
    def __init__(self):
        # load ink logo model
        self.logo_model = load_model_json.load('models/logo.json', "models/logo.h5")

        # Initialize HoG descriptor
        self.hog = cv2.HOGDescriptor((64, 64), (16, 16), (8, 8), (8, 8), 9)

        self.REDUNDANT_THRESHOLD_30RED = 3800
        self.REDUNDANT_THRESHOLD_30YELLOW = 6100
        self.REDUNDANT_THRESHOLD_31RED = 3800
        self.REDUNDANT_THRESHOLD_31YELLOW = 6100
        self.REDUNDANT_THRESHOLD_32RED = 4200
        self.REDUNDANT_THRESHOLD_32YELLOW = 6100

    def logo_detector(self, logo, size_obj: str, color_obj: str, vector_conveyor: list):
        """
        :param color_obj: color of detected object
        :param vector_conveyor: direction vector of conveyor
        :param size_obj: size of detected object
        :param logo: 124x124
        :return: result of logo
        """

        if size_obj == "error":
            return ""

        logo_detected = logo.copy()

        # region Check Location
        result_location = ffeats.find_logo_locate_features()
        ffeats.cor_part1 = []
        ffeats.cor_part3 = []
        if result_location == "error":
            cv2.putText(logo, "Location Error", (10, 10), cv2.FONT_HERSHEY_PLAIN, 0.9, (0, 0, 255), 1, cv2.LINE_AA)
            return "location error"

        # check length of logo location at the true part
        len_of_located_logo = math.sqrt((int(ffeats.center_obj[0, 0]) - ffeats.center_logo[0])**2 +
                                        (int(ffeats.center_obj[0, 1]) - ffeats.center_logo[1])**2)
        if size_obj == "30":
            if len_of_located_logo > 93 or len_of_located_logo < 75:
                cv2.putText(logo, "Location Error", (10, 10), cv2.FONT_HERSHEY_PLAIN, 0.9, (0, 0, 255), 1, cv2.LINE_AA)
                return "location error"
        elif size_obj == "31":
            if len_of_located_logo > 107 or len_of_located_logo < 83:
                cv2.putText(logo, "Location Error", (10, 10), cv2.FONT_HERSHEY_PLAIN, 0.9, (0, 0, 255), 1, cv2.LINE_AA)
                return "location error"
        elif size_obj == "32":
            if len_of_located_logo > 115 or len_of_located_logo < 89:
                cv2.putText(logo, "Location Error", (10, 10), cv2.FONT_HERSHEY_PLAIN, 0.9, (0, 0, 255), 1, cv2.LINE_AA)
                return "location error"
        else:
            return ""

        # endregion

        if color_obj == "error":
            return ""

        # region Check Direction
        vecto_obj = [int(ffeats.center_obj[0, 0]) - ffeats.center_logo[0],
                     int(ffeats.center_obj[0, 1]) - ffeats.center_logo[1]]
        # print(vecto_obj)
        if vecto_obj[1] < 0:  # direction of object is inverse to direction of conveyor
            vector_conveyor[0] = -vector_conveyor[0]
            vector_conveyor[1] = -vector_conveyor[1]
        rotated_angle = self.angle_between(np.array(vecto_obj), np.array(vector_conveyor))
        if rotated_angle > 135:
            rotated_angle = 180 - rotated_angle

        if vecto_obj[1] < 0:  # direction of object is inverse to direction of conveyor
            if vecto_obj[0] > 7:  # right side, rotate reverse
                rotated_angle = rotated_angle
            elif vecto_obj[0] <= -4:  # left side, rotate forward
                rotated_angle = -rotated_angle
            else:
                rotated_angle = -rotated_angle
        else:
            if vecto_obj[0] >= 0:  # right side, rotate reverse
                rotated_angle = -rotated_angle
            elif vecto_obj[0] < -6:  # left side, rotate forward
                rotated_angle = rotated_angle
            else:
                rotated_angle = -rotated_angle

        # print("angle: ", rotated_angle)

        if vecto_obj[1] > 0:
            obj_dir = 1  # forward
        else:
            obj_dir = 0  # reverse

        redundant_area, type_obj = ffeats.find_logo_direct_features(logo_detected, size_obj, color_obj, rotated_angle, obj_dir)
        # print("redundant: ", redundant_area)

        if ffeats.isShowLogoFeats:
            cv2.putText(logo, str(round(float(rotated_angle), 2)), (10, 30), cv2.FONT_HERSHEY_PLAIN,
                        0.7, (255, 0, 0), 1, cv2.LINE_AA)
            cv2.putText(logo, str(redundant_area), (10, 50), cv2.FONT_HERSHEY_PLAIN, 0.7, (255, 0, 0), 1, cv2.LINE_AA)

        if type_obj == "30_red" and redundant_area > self.REDUNDANT_THRESHOLD_30RED:
            isError = True
        elif type_obj == "30_yellow" and redundant_area > self.REDUNDANT_THRESHOLD_30YELLOW:
            isError = True
        elif type_obj == "31_red" and redundant_area > self.REDUNDANT_THRESHOLD_31RED:
            isError = True
        elif type_obj == "31_yellow" and redundant_area > self.REDUNDANT_THRESHOLD_31YELLOW:
            isError = True
        elif type_obj == "32_red" and redundant_area > self.REDUNDANT_THRESHOLD_32RED:
            isError = True
        elif type_obj == "32_yellow" and redundant_area > self.REDUNDANT_THRESHOLD_32YELLOW:
            isError = True
        else:
            isError = False
        if isError:
            cv2.putText(logo, "Direction Error", (10, 10), cv2.FONT_HERSHEY_PLAIN, 0.9, (0, 0, 255), 1, cv2.LINE_AA)
            return "direction error"

        # if int(ffeats.center_obj[0, 0]) != 0:
        #     lines = preprocessing_logo(logo_detected)
        #     vecto_logo, x1, y1, x2, y2 = ffeats.find_logo_direct_features(lines)
        #     vecto_obj = [int(ffeats.center_obj[0, 0]) - ffeats.center_logo[0],
        #                  int(ffeats.center_obj[0, 1]) - ffeats.center_logo[1]]
        #     if vecto_logo[0] != 0:
        #         angle = self.angle_between(np.array(vecto_obj), np.array(vecto_logo))
        #         if ffeats.isShowLogoFeats:
        #             cv2.line(logo, (x1, y1), (x2, y2), (255, 0, 0), 2)
        #             cv2.putText(logo, str(round(float(angle), 2)), (10, 30), cv2.FONT_HERSHEY_PLAIN, 0.7,
        #                         (255, 0, 0), 1, cv2.LINE_AA)
        #         if not (82 < angle < 96):
        #             cv2.putText(logo, "Direction Error", (10, 10), cv2.FONT_HERSHEY_PLAIN, 0.9, (0, 0, 255), 1, cv2.LINE_AA)
        #             return "direction error"

        # endregion

        # region Check Lost Ink
        logo_gray_image = cv2.cvtColor(cv2.resize(logo_detected, (64, 64)), cv2.COLOR_BGR2GRAY)
        logo_features = self.hog.compute(logo_gray_image)
        features = np.array(logo_features).reshape(-1)
        prediction = self.logo_model.predict(np.expand_dims(features, axis=0), verbose=0)
        if prediction < 0.5:
            cv2.putText(logo, "Normal", (10, 10), cv2.FONT_HERSHEY_PLAIN, 0.9, (255, 0, 0), 1, cv2.LINE_AA)
            return "not error"
        else:
            cv2.putText(logo, "Ink Error", (10, 10), cv2.FONT_HERSHEY_PLAIN, 0.9, (0, 0, 255), 1, cv2.LINE_AA)
            return "ink error"

        # endregion

    @staticmethod
    def angle_between(v1, v2):
        v1_unit = v1 / np.linalg.norm(v1)
        v2_unit = v2 / np.linalg.norm(v2)
        dot_product = np.dot(v1_unit, v2_unit)
        angle = np.arccos(dot_product)
        return np.degrees(angle)
