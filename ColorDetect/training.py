from keras.models import Sequential
from keras.layers import Conv2D, Dense, Flatten, MaxPool2D
from load_data_training import MasterImage
from keras.models import model_from_json

# -----------------------------------------
# 1. Load dataset from your path, if the data haven't saved as file.pkl before, the code will create new file.pkl
path = 'C:/Users/hieng/PycharmProjects/ColorDetect/dataset'
a = MasterImage(PATH=path, IMAGE_SIZE=80)
X_train, Y_train = a.load_dataset()

# -----------------------------------------
# 2. Create structure of CNN

# # defining model
# model = Sequential()
#
# # adding convolution layer
# model.add(Conv2D(64, (2, 2), strides=3, activation='relu', input_shape=(80, 80, 3)))
#
# # adding pooling layer
# model.add(MaxPool2D(2, 2))
#
# # adding convolution layer
# model.add(Conv2D(24, (2, 2), strides=2, activation='relu'))
#
# # adding pooling layer
# model.add(MaxPool2D(2, 2))
#
# # adding fully connected layer
# model.add(Flatten())
# model.add(Dense(32, activation='relu'))
#
# # adding output layer
# model.add(Dense(3, activation='softmax'))
#
# model.summary()

# transfer model
json_file = open('old/color.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
model = model_from_json(loaded_model_json)
model.load_weights("old/color.h5")

# compiling the model
model.compile(loss='sparse_categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

# fitting the model
model.fit(X_train, Y_train, batch_size=16, epochs=25)

# evaluate the model
scores = model.evaluate(X_train, Y_train)
print("%s: %.2f%%" % (model.metrics_names[1], scores[1] * 100))


# -----------------------------------------
# 3. Save the model

# serialize model to JSON
model_json = model.to_json()
with open("output_model/color.json", "w") as json_file:
    json_file.write(model_json)

# serialize weights to HDF5
model.save_weights("output_model/color.h5")
print("Saved model to disk")
