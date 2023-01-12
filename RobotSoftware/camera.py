import cv2


class MyCamera:
    def __init__(self):
        self.vid = None

    def __del__(self):
        try:
            if self.vid.isOpened():
                self.vid.release()
                cv2.destroyAllWindows()
        except:
            pass

    def connect_cam(self):
        self.vid = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        if self.vid.isOpened():
            return 1
        else:
            return 0

    def disconnect_cam(self):
        if self.vid.isOpened():
            self.vid.release()
            cv2.destroyAllWindows()
        if not self.vid.isOpened():
            return 1
        else:
            return 0

    def get_frames(self):
        if self.vid.isOpened():
            _, image = self.vid.read()
            return image


mycam = MyCamera()
