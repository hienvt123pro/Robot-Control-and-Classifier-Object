delete X_Data and Y_data if using new dataset to train model

## Example code to test model:
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
test_org = cv2.imread("path to image")
test = cv2.cvtColor(test_org, cv2.COLOR_BGR2HSV)
test = cv2.resize(test, (80, 80))
test = test / 255
test = test.reshape(-1, 80, 80, 3)

# predict
y_pred = loaded_model.predict(test)
print(y_pred)

y_max = np.argmax(y_pred)
result = y_pred[0, y_max]
print(result)
