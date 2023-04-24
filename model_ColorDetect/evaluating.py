from load_data_training import MasterImage
from keras.models import model_from_json
from sklearn.metrics import confusion_matrix, accuracy_score
import numpy as np

path = 'C:/Users/hieng/PycharmProjects/ColorDetect/dataset_test'
a = MasterImage(PATH=path, IMAGE_SIZE=80)
X_test, Y_test = a.load_dataset()

# ------------------------------------------
# 1. Load the trained model

path_old = "old/"
path_new = "output_model/"
# load json and create model
json_file = open(f'{path_new}/color.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
loaded_model = model_from_json(loaded_model_json)

# load weights into new model
loaded_model.load_weights(f"{path_new}/color.h5")
print("Loaded model from disk")

# ------------------------------------------
# 2. Evaluate the model

y_pred = loaded_model.predict(X_test)

for i in range(len(y_pred)):
    y_pred[i] = np.argmax(y_pred[i, :])

y_pred = np.array([(y_pred[:, 0])])

cm = confusion_matrix(Y_test, y_pred[0])
print("1. Red  2. Yellow  3. Others")
print(cm, '\n Accuracy: ', round(accuracy_score(Y_test, y_pred[0]) * 100, 2), "%")
