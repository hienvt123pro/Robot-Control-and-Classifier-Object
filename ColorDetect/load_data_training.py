import cv2
import os
import pickle
import numpy as np


class MasterImage(object):
    def __init__(self, PATH='', IMAGE_SIZE=80):
        self.PATH = PATH
        self.IMAGE_SIZE = IMAGE_SIZE

        self.image_data = []
        self.x_data = []
        self.y_data = []
        self.CATEGORIES = []

        # This will get List of categories
        self.list_categories = []

    def get_categories(self):
        for paths in os.listdir(self.PATH):
            if '.DS_Store' in paths:
                pass
            else:
                self.list_categories.append(paths)
        print("Found Categories ", self.list_categories, '\n')
        return self.list_categories

    def Process_Image(self):
        try:
            """
            Return Numpy array of image
            :return: X_Data, Y_Data
            """
            self.CATEGORIES = self.get_categories()
            for categories in self.CATEGORIES:  # Iterate over categories

                train_folder_path = os.path.join(self.PATH, categories)  # Folder Path
                class_index = self.CATEGORIES.index(categories)  # this will get index for classification

                for img in os.listdir(train_folder_path):  # This will iterate in the Folder
                    new_path = os.path.join(train_folder_path, img)  # image Path
                    try:  # if any image is corrupted
                        image_data_temp = cv2.imread(new_path)  # Read Image as numbers
                        image_data_temp = cv2.cvtColor(image_data_temp, cv2.COLOR_BGR2HSV)
                        image_temp_resize = cv2.resize(image_data_temp, (self.IMAGE_SIZE, self.IMAGE_SIZE))
                        self.image_data.append([image_temp_resize, class_index])
                    except:
                        pass

            data = np.asanyarray(self.image_data)

            # Iterate over the Data
            for x in data:
                self.x_data.append(x[0])  # Get the X_Data
                self.y_data.append(x[1])  # get the label

            X_data = np.asarray(self.x_data) / 255.0  # Normalize Data
            Y_data = np.asarray(self.y_data)

            # reshape x_Data

            X_data = X_data.reshape(-1, self.IMAGE_SIZE, self.IMAGE_SIZE, 3)

            return X_data, Y_data
        except:
            print("Failed to run Function Process Image ")

    def pickle_image(self):
        """
        :return: None Creates a Pickle Object of DataSet
        """
        # Call the Function and Get the Data
        X_data, Y_data = self.Process_Image()

        # Write the Entire Data into a Pickle File
        pickle_out = open('X_Data', 'wb')
        pickle.dump(X_data, pickle_out)
        pickle_out.close()

        # Write the Y Label Data
        pickle_out = open('Y_Data', 'wb')
        pickle.dump(Y_data, pickle_out)
        pickle_out.close()

        print("Pickled Image Successfully ")
        return X_data, Y_data

    def load_dataset(self):
        try:
            # Read the Data from Pickle Object
            X_Temp = open('X_Data', 'rb')
            X_data = pickle.load(X_Temp)

            Y_Temp = open('Y_Data', 'rb')
            Y_data = pickle.load(Y_Temp)

            print('Reading Dataset from PIckle Object')

            return X_data, Y_data

        except:
            print('Could not Found Pickle File ')
            print('Loading File and Dataset  ..........')

            X_data, Y_data = self.pickle_image()
            return X_data, Y_data
