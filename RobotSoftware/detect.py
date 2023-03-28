"""
Provide functions, class for processing models (yolov5, size ANN, color CNN)
"""

from find_feature import find_features
from preprocessing import preprocessing_img
from size_uncertainty import SizeProbabilistic
import load_model_json
import joblib
import cv2
import numpy as np
import torch


# -----------------------------------------------------------
# 1. Detect object process
class ObjectProcess:
    def __init__(self):
        self.obj_model = torch.hub.load('ultralytics/yolov5', 'custom', path='models/obj.pt', force_reload=True)
        self.obj_model.iou = 0.5
        self.obj_model.conf = 0.83
        self.obj_classes = self.obj_model.names
        self.bb = (0, 0, 0, 0)
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
        for i in range(len(labels)):
            row = cord[i]
            x1, y1, x2, y2 = int(row[0] * x_shape), int(row[1] * y_shape), int(row[2] * x_shape), int(row[3] * y_shape)
            cls = classes[int(labels[i])]
            if cls == 'logo':
                cropped_logo = frame[y1 + 3:y2 - 3, x1 + 3:x2 - 3]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f" {round(float(row[4]), 2)}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 2)
            elif cls == 'dep':
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.putText(frame, f" {round(float(row[4]), 2)}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 2)
                bbox = (x1, y1, x2, y2)
        return frame, cropped_logo, bbox

    def obj_detector_process(self, input_queue):
        try:
            img_org = input_queue.copy()
            results = self.yolov5_detect(cv2.cvtColor(input_queue, cv2.COLOR_BGR2RGB), model=self.obj_model)

            # detect both of logo and obj: {15:obj, 16:logo}
            if 15 in results[0] and 16 in results[0]:
                detected_frame, crop_logo, bb = self.yolo_bb_obj(results, img_org, classes=self.obj_classes)
                return True, detected_frame, crop_logo, bb
            else:
                return False, img_org, self.waited_capture, self.bb
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
        size_dict = {0: '16', 1: '18', 2: '20', 3: 'error'}
        return size_dict.get(y, None)

    @staticmethod
    def color_result(y):
        color_dict = {0: 'Red', 1: 'Yellow', 2: 'error'}
        return color_dict.get(y, None)

    # preprocessing data for color model
    @staticmethod
    def transform_color(I, c_X, c_Y):
        cen_img = I[c_Y - 50:c_Y + 50, c_X - 50:c_X + 50]
        img_hsv = cv2.cvtColor(cen_img, cv2.COLOR_BGR2HSV)
        img_rz = cv2.resize(img_hsv, (80, 80))
        img_output = img_rz / 255
        return img_output.reshape(-1, 80, 80, 3)

    # detect both size and color
    def detect_size_color(self, I, bb_x1, bb_y1, bb_x2, bb_y2):
        img_org, img_contour = preprocessing_img(I, (bb_x1, bb_y1, bb_x2, bb_y2))
        img, center, d1, d2, d3 = find_features(img_org, img_contour)

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
            predict_color = self.color_model.predict(self.transform_color(img, cX, cY), verbose=0)
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
