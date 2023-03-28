import math
import numpy as np
import load_model_json
import joblib
import cv2
import os
from os import path
import glob


class SizeCalib:
    """
    Size Model Calibration
    Examples:
        ----d1-mean-----d2-mean-----d3-mean----
        s16-221.7217993	118.812854	127.9492743
        s18-249.4226841	138.9003896	137.4469691
        s20-276.8924798	150.5317482	159.4083070

        h_org/d1_0 = h1/d1 => h1 = ? -- calib factor for d1 => cf1 = d1_0/d1
        h_org/d2_0 = h2/d2 => h2 = ? -- calib factor for d2 => cf2 = d2_0/d2
        h_org/d3_0 = h3/d3 => h3 = ? -- calib factor for d3 => cf3 = d3_0/d3

    with: - d1_0, d2_0, d3_0: mean data of d1 pretrained feature of 1 class
          - d1, d2, d3: data which is found after processing new preprocessing

    """

    def __init__(self):
        self.calib_16m = joblib.load("data/mean_16.save")

        # size 16 mean data feature, from 'data/mean16.save'
        self.d1_0, self.d2_0, self.d3_0 = self.calib_16m[0], self.calib_16m[1], self.calib_16m[2]

        self.cf1, self.cf2, self.cf3 = 0, 0, 0
        self.size_model = load_model_json.load('models/size.json', "models/size.h5")
        self.sc = joblib.load('data/scaler_size.save')

    def convert(self, d1: float, d2: float, d3: float):
        try:
            self.cf1 = self.d1_0 / d1
            self.cf2 = self.d2_0 / d2
            self.cf3 = self.d3_0 / d3
        except:
            self.cf1 = 0
            self.cf2 = 0
            self.cf3 = 0

    def test(self, d1: float, d2: float, d3: float):
        try:
            if self.cf1 == 0:
                return 'error'
            data = np.array([(d1 * self.cf1, d2 * self.cf2, d3 * self.cf3)])
            predict_size = self.size_model.predict(self.sc.transform(data), verbose=0)
            y_pred = np.argmax(predict_size)
            size = self.size_result(y_pred)
            if predict_size[0, y_pred] < 0.5:
                size = 'error'
            return size
        except:
            pass

    @staticmethod
    def size_result(y):
        size_dict = {0: '16', 1: '18', 2: '20', 3: 'error'}
        return size_dict.get(y, None)


size_calib = SizeCalib()


class CamCalib:
    """
    Camera Calibration
    Purpose: convert coordinates in 2D image space to 3D real space.
    Use checkerboard to easily apply opencv library to find points in 3D space to pixels in 2D space:
    - Measure points in real space manually.
    - Find pixels in 2D space from opencv support library.
    - Use cv2.calibrateCamera() to find:
        => K_mtx: intrinsic matrix camera matrix, is a 3x3 matrix representing the relationship between coordinates in
    real space and coordinates in image space. This matrix includes parameters such as focal length, focal distance, and
    center position of the image in image space.
        => dist: vector distortion coefficients, is a vector of length 5 that contains coefficients to represent distortions
    in the camera image, including curve deviation, angular deviation, ...
        => rvecs: list of rotation vectors, each vector is a rotation to transform real space into image space. This vector
    is represented by the Roll-Pitch-Yaw parameters.
        => tvecs: a list of translation vectors, each vector is a translation to transform real space into image space.

    These values can then be used to convert coordinates between real and image space, as well as to correct for errors
    in the image caused by the camera.

    The relationship between coordinates in real space and coordinates in image space formula:
                             un-normalize 2D point = K * [R|t] * 3D point
                                with: + K: (3x3).
                                      + [R|t] or T: (3x4) for cal un-normalize 2D point
                                      + 3D point: (4x1) [xw,yw,zw,1].
                                      + un-normalize 2D point: (3x1) [X,Y,Z]

                            3D point = inv(P) * un-normalize 2D point
                                with: + P = (K * [R|t]).append(K * [R|t], [0,0,0,1], axis=0)
                                      + un-normalize 2D point: (4,1) [X,Y,Z,1]
                                      + 3D point: (4,1) [xw,yw,zw,1]

        - [R|t] * 3D point: 3D point on camera coordinates, convert world coors to camera coors.
        - inv(K) * un-normalize 2D point: 3D point on camera coordinates, convert un-normalize image coors to camera coors.
        - un-normalize 2D point = (X, Y, Z) with Z is the deep => normalize 2D point = (X/Z, Y/Z, 1) is pixel coordinates

    """

    def __init__(self):
        # checkerboard dimension
        self.CHESS_BOARD_DIM = (7, 5)

        # count number of saved images
        self.counter_shot_img = 1

        # optimal coefficient for Harris or Shi-Tomasi corner detector algorithm (loop: 30, precision: 0.01)
        self.CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

        # saved data directory
        self.img_dir_path = "data/checker_board_images"
        self.calib_dir_path = "data/cam_calib_data"

        # load known 3D points
        self.known_obj_3D = joblib.load("data/cam_calib_data/3D_points.save")

        # frame size
        self.FRAME_SIZE = (640, 480)

        # data calibration
        self.K_mtx, self.dist, self.rvecs, self.tvecs = np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3)), np.zeros((3, 3))
        self.Z = 0

    def check_dir(self):
        dirs = [self.img_dir_path, self.calib_dir_path]
        checks = []

        for d in dirs:
            if not os.path.isdir(d):
                os.makedirs(d)
                checks.append(f"folder '{d}' created")
            else:
                checks.append(f"'{d}'")

        return tuple(checks)

    def check_file_in_images_folder(self):
        if path.exists(f"{self.img_dir_path}/img_1.png"):
            files = os.listdir(self.img_dir_path)
            return True, len(files)
        else:
            return False, 0

    def delete_files_images_folder(self):
        for filename in os.listdir(self.img_dir_path):
            file_path = os.path.join(self.img_dir_path, filename)
            try:
                if os.path.isfile(file_path):
                    os.remove(file_path)
            except Exception as e:
                print(f"Cannot delete {file_path} by {e}")

    def detect_checker_board(self, img):
        copyImage = img.copy()
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray_img, self.CHESS_BOARD_DIM)
        if ret:
            corner1 = cv2.cornerSubPix(gray_img, corners, (3, 3), (-1, -1), self.CRITERIA)
            img = cv2.drawChessboardCorners(img, self.CHESS_BOARD_DIM, corner1, ret)
        cv2.putText(img, f"save image {self.counter_shot_img}", (30, 40), cv2.FONT_HERSHEY_PLAIN, 1.4,
                    (0, 255, 0), 2, cv2.LINE_AA)
        return ret, img, copyImage

    def save_images(self, isBoardDetected: bool, copyFrame):
        if isBoardDetected:
            cv2.imwrite(f"{self.img_dir_path}/img_{self.counter_shot_img}.png", copyFrame)
            self.counter_shot_img += 1
            return f"Total saved images: {self.counter_shot_img - 1}"
        else:
            return "Error detect Checkerboard"

    def calibrate_camera(self):
        obj_point_3D = []
        img_point_2D = []

        # load the 3D real space points into 'obj_point_3D' and the corresponding 2D space pixels into 'img_point_2D'
        files = os.listdir(self.img_dir_path)
        for f in files:
            imgPath = os.path.join(self.img_dir_path, f)
            image = cv2.imread(imgPath)
            grayScale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(image, self.CHESS_BOARD_DIM, None)
            if ret:
                obj_point_3D.append(self.known_obj_3D)
                corner1 = cv2.cornerSubPix(grayScale, corners, (3, 3), (-1, -1), self.CRITERIA)
                img_point_2D.append(corner1)

        ret, self.K_mtx, self.dist, self.rvecs, self.tvecs = cv2.calibrateCamera(obj_point_3D, img_point_2D,
                                                                                 self.FRAME_SIZE, None, None)

        if ret:
            return 'Successful camera calibration'
        else:
            return 'Unsuccessful camera calibration'

    @staticmethod
    def convert3Dto2D_findDeptZ(K_mtx, rvecs, tvecs, point_3D: list):
        """
        :param K_mtx: intrinsic matrix of camera from previous calibration
        :param rvecs: rotation vectors from previous calibration
        :param tvecs: translation vectors from previous calibration
        :param point_3D: 1 point of known 3D points
        :return: normalize 2D point converting, relative dept Z
        """

        # random relative homogeneous coordinate transformation matrix
        rmats, _ = cv2.Rodrigues(rvecs[0])
        R_t = np.hstack((rmats, tvecs[0]))

        # world coordinates
        world_point = np.array(np.hstack((point_3D, 1)), dtype=np.float32).reshape(4, 1)

        # find un-normalize 2D point
        image_point = np.dot(K_mtx, np.dot(R_t, world_point))

        # random relative dept Z
        Z = image_point[2].copy()

        # find normalize 2D pixel coordinates
        image_point /= image_point[2]

        return image_point[:2], Z

    @staticmethod
    def convert2Dto3D(K_mtx, rvecs, tvecs, norm_point_2D: list, Z_dept):
        """
        :param K_mtx: intrinsic matrix of camera from previous calibration
        :param rvecs: rotation vectors from previous calibration
        :param tvecs: translation vectors from previous calibration
        :param norm_point_2D: 1 un-normalize point 2D, format: [x_pixel,y_pixel,1]
        :param Z_dept: Z deep coefficient
        :return: x_world, y_world, z_world
        """
        # random relative homogeneous coordinate transformation matrix
        rmats, _ = cv2.Rodrigues(rvecs[0])
        R_t = np.hstack((rmats, tvecs[0]))

        # find P matrix
        P_mats = K_mtx.dot(R_t)
        P_mats = np.append(P_mats, [np.array([0, 0, 0, 1], dtype=np.float32)], axis=0)

        # create un-normalize 2D point
        un_norm_point_2D = np.array(norm_point_2D) * Z_dept
        un_norm_point_2D = un_norm_point_2D.flatten().tolist()

        image_point = np.array(np.hstack((un_norm_point_2D, 1)), dtype=np.float32).reshape(4, 1)

        # convert pixel coors -> camera coors -> world coors
        world_point = np.dot(np.linalg.inv(P_mats), image_point)

        return tuple(world_point[:3, 0])

    def save_model(self, name: str):
        try:
            _, zDept = self.convert3Dto2D_findDeptZ(self.K_mtx, self.rvecs, self.tvecs, self.known_obj_3D[0, :])
            np.savez(f"{self.calib_dir_path}/{name}", K_matrix=self.K_mtx, distCoef=self.dist, rVectors=self.rvecs,
                     tVectors=self.tvecs, Z_dept=zDept)
            return f"Save model '{name}.npz' successful", True
        except:
            return "Error occurs when saving model", False

    def load_model(self, name: str):
        try:
            data = np.load(f"{self.calib_dir_path}/{name}")
            self.K_mtx = data['K_matrix']
            self.dist = data['distCoef']
            self.rvecs = data['rVectors']
            self.tvecs = data['tVectors']
            self.Z = data['Z_dept']
            return f"Load calibration model '{name}' successful"
        except:
            return "Calibration model is not exists"

    def findImage2DCoors(self, real_coors: list):
        """
        :param real_coors: [xw,yw,0]
        :return: image coordinates [x,y]
        """
        try:
            pix_coor, _ = self.convert3Dto2D_findDeptZ(self.K_mtx, self.rvecs, self.tvecs, real_coors)
            ceil_arr = np.ceil(pix_coor)
            return tuple(map(int, ceil_arr.flatten()))
        except Exception as e:
            print(e)
            return 0, 0

    def findPerspective3DCoors(self, pixel_coors: list):
        """
        :param pixel_coors: [x,y,1]
        :return: world coordinates [xw,yw,zw]
        """
        try:
            x_real, y_real, z_real = self.convert2Dto3D(self.K_mtx, self.rvecs, self.tvecs, pixel_coors, self.Z)
            return tuple(round(coor, 2) for coor in (x_real, y_real, 3))
        except Exception as e:
            print(e)
            return 0, 0, 0

    def get_undistorted_images(self, image):
        try:
            return cv2.undistort(image, self.K_mtx, self.dist)
        except:
            return joblib.load("data/logo_cap_wait.save")

    def list_model_names(self):
        npz_files = glob.glob(f"{self.calib_dir_path}/*.npz")
        for i, f in enumerate(npz_files):
            npz_files[i] = os.path.basename(f)
        return npz_files

    def test_model_use_checkerboard(self, img_undist, num_points):
        """
        :param img_undist: undistorted image
        :return: checkerboard image with calib point
        """
        img_org = img_undist.copy()

        # Find the chessboard corners in the undistorted image
        ret, corners = cv2.findChessboardCorners(img_undist, self.CHESS_BOARD_DIM, None)

        # Calculate the object points and image points for the detected corners
        if ret:
            img_points = cv2.cornerSubPix(cv2.cvtColor(img_undist, cv2.COLOR_BGR2GRAY), corners, (3, 3),
                                          (-1, -1), self.CRITERIA)
            for p in img_points[::math.ceil(35 / num_points)]:
                point_3D = self.findPerspective3DCoors(list(np.hstack((p[-1], 1))))
                if point_3D == (0, 0, 0):
                    return img_org
                point_2D = tuple(p[-1])
                point_2D = tuple(int(round(x, 0)) for x in point_2D)
                cv2.circle(img_undist, point_2D, radius=2, color=(0, 0, 255), thickness=2)
                cv2.putText(img_undist, f"{point_3D[:2]}", point_2D, cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1,
                            cv2.LINE_AA)
            return img_undist
        return img_org


camera_calib = CamCalib()
