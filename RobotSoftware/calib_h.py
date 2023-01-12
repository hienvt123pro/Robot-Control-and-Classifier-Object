import cv2
from find_feature import find_features
from preprocessing import preprocessing_img
import numpy as np
import load_model
import joblib
from uncertainty import Probabilistic

"""
----D1----------D2----------D3---------
s16-221.7217993	118.812854	127.9492743
s18-249.4226841	138.9003896	137.4469691
s20-276.8924798	150.5317482	159.4083070

h_org/d1_0 = h1/d1 => h1 = ? -- calib factor for d1
h_org/d2_0 = h2/d2 => h2 = ? -- calib factor for d2
h_org/d3_0 = h3/d3 => h3 = ? -- calib factor for d3
"""


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


vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)

prob = Probabilistic()

d1_0, d2_0, d3_0 = 221.7217993, 118.812854, 127.9492743
h_org = 40
h1, h2, h3 = 0, 0, 0
cf1, cf2, cf3 = 1.0024351638959412, 1.0040801732942484, 1.0589157285307962

size_model = load_model.load('models/size.json', "models/size.h5")
sc = joblib.load('data/scaler_size.save')

if __name__ == "__main__":
    while True:
        try:
            _, image = vid.read()
            img_org, img_contour = preprocessing_img(image)
            img, center, d1, d2, d3 = find_features(img_org, img_contour)

            h1 = (d1 / d1_0) * h_org
            h2 = (d2 / d2_0) * h_org
            h3 = (d3 / d3_0) * h_org

            cf1 = h_org / h1
            cf2 = h_org / h2
            cf3 = h_org / h3

            cv2.imshow('object', img)
        except:
            pass

        if cv2.waitKey(20) & 0xFF == ord('q'):
            vid.release()
            cv2.destroyAllWindows()
        elif cv2.waitKey(20) & 0xFF == ord('w'):
            print(cf1, cf2, cf3)

    # while True:
    #     try:
    #         _, image = vid.read()
    #         img_org, img_contour = preprocessing_img(image)
    #         img, center, d1, d2, d3 = find_features(img_org, img_contour)
    #
    #         d1, d2, d3 = d1 * cf1, d2 * cf2, d3 * cf3
    #         data = np.array([(d1, d2, d3)])
    #         predict_size = size_model.predict(sc.transform(data))
    #         prob_predict = prob.predict(predict_size)
    #         y_pred = np.argmax(prob_predict)
    #         size = size_result(y_pred)
    #         if prob_predict[y_pred, 0] < 0.5:
    #             size = 'error'
    #         cv2.putText(img, size, (10, 20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (255, 100, 10), 2)
    #         cv2.imshow('object', img)
    #     except:
    #         pass
    #
    #     if cv2.waitKey(20) & 0xFF == ord('q'):
    #         vid.release()
    #         cv2.destroyAllWindows()
