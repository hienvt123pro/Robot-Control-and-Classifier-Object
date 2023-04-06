from keras.models import model_from_json
from load_data_training import MasterImage

path = 'C:/Users/hieng/PycharmProjects/ColorDetect/transfer_dataset'
a = MasterImage(PATH=path, IMAGE_SIZE=80)
X_train, Y_train = a.load_dataset()

# load json of pretrained model
json_file = open("output_model/color.json", 'r')
loaded_model_json = json_file.read()
json_file.close()
transfer_model = model_from_json(loaded_model_json)

# load weights into pretrained model
transfer_model.load_weights("output_model/color.h5")

# compiling the model
transfer_model.compile(loss='sparse_categorical_crossentropy', optimizer='adam', metrics=['accuracy'])

# fitting the model
transfer_model.fit(X_train, Y_train, batch_size=12, epochs=20)

# evaluate the model
scores = transfer_model.evaluate(X_train, Y_train)
print("%s: %.2f%%" % (transfer_model.metrics_names[1], scores[1] * 100))

# serialize model to JSON
model_json = transfer_model.to_json()
with open("transfer_output_model/transfer_color.json", "w") as json_file:
    json_file.write(model_json)

# serialize weights to HDF5
transfer_model.save_weights("transfer_output_model/transfer_color.h5")
print("Saved model to disk")
