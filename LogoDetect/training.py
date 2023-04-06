import numpy as np
from keras.models import Sequential
from keras.layers import Dense, Dropout
from sklearn.metrics import confusion_matrix, accuracy_score
import cv2
import os

# -----------------------------------
# 1. Compute HoG descriptors and labels for each image pair

# Initialize HoG descriptor
hog = cv2.HOGDescriptor((64, 64), (16, 16), (8, 8), (8, 8), 9)

# Load images
normal_images = []
error_images = []

normal_path = "datasets/not_error"
error_path = "datasets/error"

for filename in os.listdir(normal_path):
    img = cv2.imread(os.path.join(normal_path, filename))
    normal_images.append(cv2.resize(img, (64, 64)))

for filename in os.listdir(error_path):
    img = cv2.imread(os.path.join(error_path, filename))
    error_images.append(cv2.resize(img, (64, 64)))

data = []
labels = []
for i in range(len(normal_images)):
    # Convert normal and error images to grayscale
    normal_gray_image = cv2.cvtColor(normal_images[i], cv2.COLOR_BGR2GRAY)
    error_gray_image = cv2.cvtColor(error_images[i], cv2.COLOR_BGR2GRAY)

    # Compute HoG descriptors for normal and error logo images
    normal_features = hog.compute(normal_gray_image)
    normal_features = np.array(normal_features).reshape(-1)

    error_features = hog.compute(error_gray_image)
    error_features = np.array(error_features).reshape(-1)

    # Append HoG descriptors and labels to data array
    data.append(normal_features)
    data.append(error_features)
    labels.append(0)
    labels.append(1)

# Convert data and labels to numpy arrays
data = np.array(data)
labels = np.array(labels)

# Shuffle data and labels
idx = np.random.permutation(len(data))
X_train, Y_train = data[idx], labels[idx]

# -----------------------------------
# 2. Create structure of ANN

# create model
classifier = Sequential()
classifier.add(Dropout(0.2, input_shape=(1764,)))
classifier.add(Dense(units=15, activation='relu'))
classifier.add(Dropout(0.5))
classifier.add(Dense(units=1, activation='sigmoid'))

# Compile model
classifier.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])

# Fit the model
classifier.fit(X_train, Y_train, batch_size=32, epochs=100)

# evaluate the model
scores = classifier.evaluate(X_train, Y_train)
print("%s: %.2f%%" % (classifier.metrics_names[1], scores[1] * 100))


# -----------------------------------
# 3. Save the model into JSON, h5 file

# serialize model to JSON
model_json = classifier.to_json()
with open('output_models/logo.json', "w") as json_file:
    json_file.write(model_json)

# serialize weights to HDF5
classifier.save_weights('output_models/logo.h5')
print("Saved model to disk")


# -----------------------------------
# 4. Evaluating model on training datasets

print("\n", "Valuating model on training data:")
y_pred = classifier.predict(X_train)
for i in range(len(y_pred)):
    if y_pred[i] < 0.5:
        y_pred[i] = 0
    else:
        y_pred[i] = 1

y_pred = np.array([(y_pred[:, 0])])

cm = confusion_matrix(Y_train, y_pred[0])
print(cm, '\n accuracy: ', accuracy_score(Y_train, y_pred[0]))
