from keras.models import model_from_json
from sklearn.preprocessing import LabelEncoder
from sklearn.preprocessing import OneHotEncoder
import pandas as pd
import numpy as np
import joblib

MAX_DATA_LEN = 97  # count in Excel

# load standard scaler from size training data
sz_scaler = joblib.load('output_models/scaler_size.save')

data_0 = pd.read_excel(r'transfer_dataset/train/size_data.xlsx', sheet_name='size')
X1_0 = pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['d1'])
X2_0 = pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['d2'])
X3_0 = pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['d3'])
X_train = sz_scaler.transform(np.concatenate((X1_0, X2_0, X3_0), axis=1))
Y_train = np.array(pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['label']))

# one hot vector for label
label_encoder = LabelEncoder()
integer_encoded = label_encoder.fit_transform(Y_train.ravel())
integer_encoded = integer_encoded.reshape(len(integer_encoded), 1)
onehot_encoder = OneHotEncoder(sparse=False)
Y_train = onehot_encoder.fit_transform(integer_encoded)

# load json of pretrained model
json_file = open("output_models/size.json", 'r')
loaded_model_json = json_file.read()
json_file.close()
transfer_model = model_from_json(loaded_model_json)

# load weights into pretrained model
transfer_model.load_weights("output_models/size.h5")

# compiling the model
transfer_model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

# fitting the model
transfer_model.fit(X_train, Y_train, batch_size=6, epochs=10)

# evaluate the model
scores = transfer_model.evaluate(X_train, Y_train)
print("%s: %.2f%%" % (transfer_model.metrics_names[1], scores[1] * 100))

# serialize model to JSON
model_json = transfer_model.to_json()
with open("transfer_output_models/transfer_size.json", "w") as json_file:
    json_file.write(model_json)

# serialize weights to HDF5
transfer_model.save_weights("transfer_output_models/transfer_size.h5")
print("Saved model to disk")
