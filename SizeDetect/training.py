import joblib
import numpy as np
import pandas as pd
from keras.models import Sequential
from keras.layers import Dense, Dropout
from sklearn.preprocessing import StandardScaler
from sklearn.preprocessing import LabelEncoder
from sklearn.preprocessing import OneHotEncoder
from sklearn.metrics import confusion_matrix, accuracy_score
from keras.callbacks import EarlyStopping, ModelCheckpoint
from sklearn.model_selection import train_test_split


# -----------------------------------
# 1. Prepare and scaler data

MAX_DATA_LEN = 230  # count in Excel

# standard scaler input
sc = StandardScaler()
data_0 = pd.read_excel(r'new_datasets/train/size_data.xlsx', sheet_name='size')
X1_0 = pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['d1'])
X2_0 = pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['d2'])
X3_0 = pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['d3'])
X = sc.fit_transform(np.concatenate((X1_0, X2_0, X3_0), axis=1))

Y = np.array(pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['label']))

Y_test_after_training = Y.T

# one hot vector for label
label_encoder = LabelEncoder()
integer_encoded = label_encoder.fit_transform(Y.ravel())
integer_encoded = integer_encoded.reshape(len(integer_encoded), 1)
onehot_encoder = OneHotEncoder(sparse=False)
Y = onehot_encoder.fit_transform(integer_encoded)

# Split data into train and test sets
X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.2, random_state=42)

# -----------------------------------
# 2. Create structure of ANN

# create model
classifier = Sequential()
classifier.add(Dropout(0.2, input_shape=(3,)))
classifier.add(Dense(units=100, activation='relu'))
classifier.add(Dropout(0.5))
classifier.add(Dense(units=84, activation='relu'))
classifier.add(Dropout(0.5))
classifier.add(Dense(units=4, activation='softmax'))

# Compile model
classifier.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

# Define Callbacks
early_stop = EarlyStopping(monitor='val_loss', patience=20, verbose=1, mode='auto')
best_model = ModelCheckpoint(filepath='output_models/size.h5', monitor='val_loss', save_best_only=True)

# Fit the model
history = classifier.fit(X_train, Y_train, batch_size=16, epochs=500, validation_data=(X_test, Y_test),
                         callbacks=[early_stop, best_model])

# evaluate the model
scores = classifier.evaluate(X_train, Y_train)
print("%s: %.2f%%" % (classifier.metrics_names[1], scores[1] * 100))


# -----------------------------------
# 3. Save the model into JSON, h5 file

# serialize model to JSON
model_json = classifier.to_json()
with open('output_models/size.json', "w") as json_file:
    json_file.write(model_json)

# # serialize weights to HDF5
# classifier.save_weights('output_models/size.h5')
# print("Saved model to disk")


# -----------------------------------
# 4. Save the scaler standard input data, and mean size calibration parameters (size16)

data_0 = pd.read_excel(r'new_datasets/train/size_data.xlsx', sheet_name='size')

d1 = pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['d1'])
d2 = pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['d2'])
d3 = pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['d3'])
sc.fit(np.concatenate((d1, d2, d3), axis=1))
joblib.dump(sc, 'output_models/scaler_size.save')

label = np.array(pd.DataFrame(data_0.iloc[0:MAX_DATA_LEN, :], columns=['label']))
dim_label_0 = np.count_nonzero(label == 0)
d1 = np.sum(pd.DataFrame(data_0.iloc[0:dim_label_0, :], columns=['d1']))
d2 = np.sum(pd.DataFrame(data_0.iloc[0:dim_label_0, :], columns=['d2']))
d3 = np.sum(pd.DataFrame(data_0.iloc[0:dim_label_0, :], columns=['d3']))
mean_d1 = float(d1 / dim_label_0)
mean_d2 = float(d2 / dim_label_0)
mean_d3 = float(d3 / dim_label_0)
joblib.dump(np.array([mean_d1, mean_d2, mean_d3]), 'output_models/mean_16.save')

# -----------------------------------
# 5. Evaluating model on training datasets

print("\n", "Valuating model on training data:")
y_pred = classifier.predict(X)
for i in range(len(y_pred)):
    y_pred[i] = np.argmax(y_pred[i, :])

y_pred = np.array([(y_pred[:, 0])])

cm = confusion_matrix(Y_test_after_training[0], y_pred[0])
print(cm, '\n Accuracy: ', accuracy_score(Y_test_after_training[0], y_pred[0]))
