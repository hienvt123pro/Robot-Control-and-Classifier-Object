from find_feature import find_features
from preprocessing import preprocessing_img
from uncertainty import Probabilistic
import cv2
import load_model
import imutils
import joblib
import numpy as np
import torch


class ModelsProcessing:
    def __init__(self):
        self.isObj = False

        # probabilistic algorithm
        self.prob = Probabilistic()

        # load standard scaler from size training data
        self.sc = joblib.load('data/scaler_size.save')

        # load size model
        self.size_model = load_model.load('models/size.json', "models/size.h5")

        # calib factors
        self.cf1, self.cf2, self.cf3 = 0.9807243463285489, 0.9518439731431791, 0.9622422345027216

        # load color model
        self.color_model = load_model.load('models/color.json', "models/color.h5")

        # load object detect model
        self.obj_model = torch.hub.load('ultralytics/yolov5', 'custom', path='models/obj.pt', force_reload=True)
        self.obj_model.iou = 0.5
        self.obj_model.conf = 0.8
        self.obj_classes = self.obj_model.names

        # load error logo model
        self.logo_model = torch.hub.load('ultralytics/yolov5', 'custom', path='models/logo.pt', force_reload=True)
        self.logo_model.iou = 0.5
        self.logo_model.conf = 0.25
        self.logo_classes = self.logo_model.names

        # error logo
        self.elogo = None
        self.logo = None

    @staticmethod
    def size_result(y):
        result = None
        if y == 0:
            result = 'size16'
        elif y == 1:
            result = 'size18'
        elif y == 2:
            result = 'size20'
        elif y == 3:
            result = 'error'
        return result

    @staticmethod
    def color_result(y):
        result = None
        if y == 0:
            result = 'red'
        elif y == 1:
            result = 'yellow'
        return result

    @staticmethod
    def transform_color(I, c_X, c_Y):
        cen_img = I[c_Y - 50:c_Y + 50, c_X - 50:c_X + 50]
        img_hsv = cv2.cvtColor(cen_img, cv2.COLOR_BGR2HSV)
        img_rz = imutils.resize(img_hsv, 80, 80)
        img_output = img_rz / 255
        return img_output.reshape(-1, 80, 80, 3)

    def detect_size_color(self, I):
        img_org, img_contour = preprocessing_img(I)
        img, center, d1, d2, d3 = find_features(img_org, img_contour)

        # calib features
        d1, d2, d3 = d1 * self.cf1, d2 * self.cf2, d3 * self.cf3

        if d1 == 0:
            return img
        try:
            # size
            data = np.array([(d1, d2, d3)])
            predict_size = self.size_model.predict(self.sc.transform(data), verbose=0)
            prob_predict = self.prob.predict(predict_size)
            y_pred = np.argmax(prob_predict)
            size = self.size_result(y_pred)
            if prob_predict[y_pred, 0] < 0.5:
                size = 'error'

            # color
            cX = int(center[0, 0])
            cY = int(center[0, 1])
            predict_color = self.color_model.predict(self.transform_color(img, cX, cY), verbose=0)
            y_pred = np.argmax(predict_color)
            color = self.color_result(y_pred)
            if predict_color[0, y_pred] < 0.65:
                color = 'error'

            # put label
            cv2.putText(img, size, (10, 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 100, 10), 2)
            cv2.putText(img, color, (10, 50), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 100, 10), 2)
        except Exception as e:
            print(e, 'undefined object')
            pass
        return img

    @staticmethod
    def yolo_detect(I, model):
        result = model([I], size=320)
        # cordinates is: x1, y2, x2, y2, confidence
        labels, cordinates = result.xyxyn[0][:, -1], result.xyxyn[0][:, :-1]
        return labels, cordinates

    @staticmethod
    def yolo_bb_obj(result, frame, classes):
        labels, cord = result
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        cropped_logo = None
        for i in range(len(labels)):
            row = cord[i]
            x1, y1, x2, y2 = int(row[0] * x_shape), int(row[1] * y_shape), int(row[2] * x_shape), int(row[3] * y_shape)
            text = classes[int(labels[i])]
            if text == 'logo':
                cropped_logo = frame[y1 + 3:y2 - 3, x1 + 3:x2 - 3]
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.rectangle(frame, (x1, y1 - 30), (x2, y1), (0, 255, 0), -1)
                cv2.putText(frame, text + f" {round(float(row[4]), 2)}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 2)
            elif text == 'dep':
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.rectangle(frame, (x1, y1 - 20), (x2, y1), (255, 0, 0), -1)
                cv2.putText(frame, text + f" {round(float(row[4]), 2)}", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 2)
        return frame, cropped_logo

    def detect_object(self, I):
        img = cv2.cvtColor(I, cv2.COLOR_BGR2RGB)
        results = self.yolo_detect(img, model=self.obj_model)
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if not ((15 in results[0]) and (16 in results[0])):
            self.isObj = False
            return frame, None
        frame, crop_logo = self.yolo_bb_obj(results, frame, classes=self.obj_classes)
        self.isObj = True
        return frame, crop_logo

    @staticmethod
    def yolo_bb_elogo(result, frame, classes):
        labels, cord = result
        x_shape, y_shape = frame.shape[1], frame.shape[0]
        for i in range(len(labels)):
            row = cord[i]
            x1, y1, x2, y2 = int(row[0] * x_shape), int(row[1] * y_shape), int(row[2] * x_shape), int(row[3] * y_shape)
            text = classes[int(labels[i])]
            if text == 'Error':
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 1)
                cv2.rectangle(frame, (x1, y1 - 10), (x2, y1), (0, 0, 255), -1)
        return frame

    def detect_elogo(self):
        while True:
            try:
                I = cv2.cvtColor(self.elogo, cv2.COLOR_BGR2RGB)
                I = cv2.resize(I, (150, 150))
                result_logo = self.yolo_detect(I, model=self.logo_model)
                I = cv2.cvtColor(I, cv2.COLOR_RGB2BGR)
                if 0 in result_logo[0]:
                    I = self.yolo_bb_elogo(result_logo, I, classes=self.logo_classes)
                self.logo = I
            except:
                pass


models_process = ModelsProcessing()
