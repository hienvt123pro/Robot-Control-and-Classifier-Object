import numpy as np
import load_model
import joblib

"""
----d1-mean-----d2-mean-----d3-mean----
s16-221.7217993	118.812854	127.9492743
s18-249.4226841	138.9003896	137.4469691
s20-276.8924798	150.5317482	159.4083070

h_org/d1_0 = h1/d1 => h1 = ? -- calib factor for d1 => cf1 = d1_0/d1
h_org/d2_0 = h2/d2 => h2 = ? -- calib factor for d2 => cf2 = d2_0/d2
h_org/d3_0 = h3/d3 => h3 = ? -- calib factor for d3 => cf3 = d3_0/d3
"""


class SizeCalib:
    def __init__(self):
        self.d1_0, self.d2_0, self.d3_0 = 221.7217993, 118.812854, 127.9492743
        self.cf1, self.cf2, self.cf3 = 0, 0, 0
        self.size_model = load_model.load('models/size.json', "models/size.h5")
        self.sc = joblib.load('data/scaler_size.save')

    def convert(self, d1, d2, d3):
        try:
            self.cf1 = self.d1_0 / d1
            self.cf2 = self.d2_0 / d2
            self.cf3 = self.d3_0 / d3
        except:
            self.cf1 = 0
            self.cf2 = 0
            self.cf3 = 0

    def test(self, d1, d2, d3):
        try:
            if self.cf1 == 0:
                return 'error'
            data = np.array([(d1 * self.cf1, d2 * self.cf2, d3 * self.cf3)])
            predict_size = self.size_model.predict(self.sc.transform(data), verbose=0)
            y_pred = np.argmax(predict_size)
            size = self.size_result(y_pred)
            if predict_size[y_pred, 0] < 0.5:
                size = 'error'
            return size
        except:
            pass

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


size_calib = SizeCalib()
