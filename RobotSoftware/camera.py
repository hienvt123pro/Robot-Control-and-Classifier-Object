import cv2


class MyCamera:
    """
    Provide functions for connect, disconnect, read frame of webcam or camera
    """
    def __init__(self):
        self.vid = None

    def __del__(self):
        try:
            if self.vid.isOpened():
                self.vid.release()
        except:
            pass

    def connect_cam(self):
        self.vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        return self.vid.isOpened()

    def disconnect_cam(self):
        if self.vid.isOpened():
            self.vid.release()
        return not self.vid.isOpened()

    def get_frames(self):
        if self.vid.isOpened():
            _, image = self.vid.read()
            return image


mycam = MyCamera()
