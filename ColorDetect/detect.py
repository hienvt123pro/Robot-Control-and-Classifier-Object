import numpy as np
from keras.models import model_from_json
import cv2

# load json and create model
json_file = open('output_model/color.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
loaded_model = model_from_json(loaded_model_json)
# load weights into new model
loaded_model.load_weights("color.h5")
print("Loaded model from disk")

# scaler for the input image
test_org = cv2.imread("link to image")
test = test_org[0:100, 0:100]
test = cv2.cvtColor(test, cv2.COLOR_BGR2HSV)
test = cv2.resize(test, (80, 80))
test = test / 255
test = test.reshape(-1, 80, 80, 3)

# predict
y_pred = loaded_model.predict(test)
print(y_pred)
y_max = np.argmax(y_pred)
result = y_pred[0, y_max]
print(result)
