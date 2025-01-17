from keras.models import model_from_json
import joblib
import numpy as np
import pandas as pd
from sklearn.metrics import confusion_matrix, accuracy_score


# ------------------------------------------
# 1. Load the trained model

path_old = "old/old4/"
path_new = "output_models/"
path = path_old

# load json and create model
json_file = open(f'{path}/size.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
loaded_model = model_from_json(loaded_model_json)

# load weights into new model
loaded_model.load_weights(f"{path}/size.h5")
print("Loaded model from disk")


# ------------------------------------------
# 2. Scaler the test input

MAX_DATA_TEST_LEN = 215

sc = joblib.load(f"{path}/scaler_size.save")

data_test = pd.read_excel(r'new_datasets/test/size_data.xlsx', sheet_name='size')
X1_test = pd.DataFrame(data_test.iloc[0:MAX_DATA_TEST_LEN, :], columns=['d1'])
X2_test = pd.DataFrame(data_test.iloc[0:MAX_DATA_TEST_LEN, :], columns=['d2'])
X3_test = pd.DataFrame(data_test.iloc[0:MAX_DATA_TEST_LEN, :], columns=['d3'])
x_test = np.concatenate((X1_test, X2_test, X3_test), axis=1)
y_test = np.array(pd.DataFrame(data_test.iloc[0:MAX_DATA_TEST_LEN, :], columns=['label'])).T

for item in set(y_test[0]):
    count = list(y_test[0]).count(item)
    print(f"label {item} appears {count} times in the list label")

# ------------------------------------------
# 3. Evaluate the model

y_pred = loaded_model.predict(sc.transform(x_test))

for i in range(len(y_pred)):
    y_pred[i] = np.argmax(y_pred[i, :])

y_pred = np.array([(y_pred[:, 0])])

cm = confusion_matrix(y_test[0], y_pred[0])
print(cm, '\n Accuracy: ', round(accuracy_score(y_test[0], y_pred[0]) * 100, 2), "%")
