"""
Load json model with weight saved in file h5 (size ANN, color CNN)
"""

from keras.models import model_from_json


# model_json:  ./model.json
# model_weights:  ./weight.h5
def load(model_json, model_weights):
    # load json and create model
    json_file = open(model_json, 'r')
    loaded_model_json = json_file.read()
    json_file.close()
    loaded_model = model_from_json(loaded_model_json)

    # load weights into new model
    loaded_model.load_weights(model_weights)
    return loaded_model
