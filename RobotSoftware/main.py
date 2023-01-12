from PySide2.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide2.QtGui import QIcon, QImage, QPixmap
from PySide2.QtCore import QSize
from GUI import gui
from communicate import serialCom, receive_thread
from camera import mycam
from detect import models_process
from preprocessing import preprocessing_img
from find_feature import find_features
import math
import sys
import threading
import cv2


class MainWindow:
    def __init__(self):
        self.Control_UI = QMainWindow()
        self.uic = gui.Ui_MainWindow()
        self.uic.setupUi(self.Control_UI)
        self.msg = QMessageBox()
        self.buffer = "0,0,0,0,0"
        self.isPortCnt = False
        self.isOpenCam = False
        self.isCalibMode = False

        # robot params
        self.d1 = 12.9
        self.a2 = 14
        self.a3 = 14
        self.a4 = 5.5 - 0.5
        self.a5 = 2.2 + 0.8
        self.Px = 0
        self.Py = 0
        self.Pz = 0
        self.Roll = 0
        self.Pitch = 0
        self.Yaw = 0
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0

        # setup properties
        self.uic.btn_move.setDisabled(True)
        self.uic.btn_x_dec.setDisabled(True)
        self.uic.btn_y_dec.setDisabled(True)
        self.uic.btn_z_dec.setDisabled(True)
        self.uic.btn_x_inc.setDisabled(True)
        self.uic.btn_y_inc.setDisabled(True)
        self.uic.btn_z_inc.setDisabled(True)
        self.uic.btn_j1_dec.setDisabled(True)
        self.uic.btn_j2_dec.setDisabled(True)
        self.uic.btn_j3_dec.setDisabled(True)
        self.uic.btn_j1_inc.setDisabled(True)
        self.uic.btn_j2_inc.setDisabled(True)
        self.uic.btn_j3_inc.setDisabled(True)
        self.uic.btn_calib_size.setDisabled(True)
        self.uic.btn_test_size.setDisabled(True)

        # event combo box
        self.uic.cb_com.activated.connect(self.list_ports())
        self.uic.cb_baudrate.activated.connect(self.list_baudrate())

        # event push button
        self.uic.btn_cnt.clicked.connect(self.connection)
        self.uic.btn_calibJ1.clicked.connect(self.calib_J1)
        self.uic.btn_calibJ2.clicked.connect(self.calib_J2)
        self.uic.btn_calibJ3.clicked.connect(self.calib_J3)
        self.uic.btn_Home.clicked.connect(self.home)
        self.uic.btn_calibHome.clicked.connect(self.calib_home)
        self.uic.btn_j1_dec.clicked.connect(self.j1_decrease)
        self.uic.btn_j1_inc.clicked.connect(self.j1_increase)
        self.uic.btn_j2_dec.clicked.connect(self.j2_decrease)
        self.uic.btn_j2_inc.clicked.connect(self.j2_increase)
        self.uic.btn_j3_dec.clicked.connect(self.j3_decrease)
        self.uic.btn_j3_inc.clicked.connect(self.j3_increase)
        self.uic.btn_x_dec.clicked.connect(self.x_decrease)
        self.uic.btn_x_inc.clicked.connect(self.x_increase)
        self.uic.btn_y_dec.clicked.connect(self.y_decrease)
        self.uic.btn_y_inc.clicked.connect(self.y_increase)
        self.uic.btn_z_dec.clicked.connect(self.z_decrease)
        self.uic.btn_z_inc.clicked.connect(self.z_increase)
        self.uic.btn_move.clicked.connect(self.move)
        self.uic.btn_apply.clicked.connect(self.apply_conf)
        self.uic.btn_start.clicked.connect(self.start)
        self.uic.btn_calib_mode.clicked.connect(self.calib_model_mode)

        # event slide
        self.uic.slide_speed.valueChanged.connect(self.set_speed)
        self.uic.slide_acc.valueChanged.connect(self.set_accelerate)
        self.uic.slide_deg.valueChanged.connect(self.set_degree)
        self.uic.slide_cm.valueChanged.connect(self.set_centimeters)

    # region port connection
    def list_ports(self):
        self.uic.cb_com.clear()
        ports = serialCom.list_ports_name()
        if not ports:
            self.uic.cb_com.addItem("not found")
        for port in ports:
            a = str(port)
            self.uic.cb_com.addItem(a[:5])

    def list_baudrate(self):
        self.uic.cb_baudrate.addItem("115200")

    def connection(self):
        self.list_ports()
        if self.uic.btn_cnt.text() == "Connect":
            port_name = self.uic.cb_com.currentText()
            baudrate = self.uic.cb_baudrate.currentText()
            self.isPortCnt = serialCom.connect(port_name=port_name, baud=baudrate)
            if self.isPortCnt:
                receive_thread.start()
                self.uic.light_stt.setStyleSheet("background: rgb(51,255,51); border-radius:20px")
                self.uic.btn_cnt.setText("Disconnect")
                self.uic.text_stt.setText("Status: On")
            else:
                self.uic.text_stt.setText("Status: Off")
                self.uic.light_stt.setStyleSheet("background: rgb(255,0,0); border-radius:20px")
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText("Check your COM Port")
                self.msg.setWindowTitle("Error")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.exec()
        else:
            serialCom.disconnect()
            self.isPortCnt = False
            self.uic.btn_cnt.setText("Connect")
            self.uic.text_stt.setText("Status: Off")
            self.uic.light_stt.setStyleSheet("background: rgb(255,0,0); border-radius:20px")

    # endregion

    # region calibrate robot
    def check_sending(self, state):
        if not state:
            self.msg.setIcon(QMessageBox.Warning)
            self.msg.setText("Check your COM Port")
            self.msg.setWindowTitle("Error")
            self.msg.setStandardButtons(QMessageBox.Ok)
            self.msg.exec()

    def calib_J1(self):
        self.buffer = "2,1,0,0,0,"
        isSend = serialCom.send_data(self.buffer)
        self.check_sending(isSend)

    def calib_J2(self):
        self.buffer = "2,2,0,0,0,"
        isSend = serialCom.send_data(self.buffer)
        self.check_sending(isSend)

    def calib_J3(self):
        self.buffer = "2,3,0,0,0,"
        isSend = serialCom.send_data(self.buffer)
        self.check_sending(isSend)

    def home(self):
        if self.isPortCnt:
            self.forwardKinematics(theta1=0, theta2=100, theta3=-130)
            self.uic.txt_j1.setText("0")
            self.uic.txt_j2.setText("100")
            self.uic.txt_j3.setText("-130")
            self.theta1 = 0
            self.theta2 = 100
            self.theta3 = -130
            self.uic.btn_move.setDisabled(False)
            self.uic.btn_x_dec.setDisabled(False)
            self.uic.btn_y_dec.setDisabled(False)
            self.uic.btn_z_dec.setDisabled(False)
            self.uic.btn_x_inc.setDisabled(False)
            self.uic.btn_y_inc.setDisabled(False)
            self.uic.btn_z_inc.setDisabled(False)
            self.uic.btn_j1_dec.setDisabled(False)
            self.uic.btn_j2_dec.setDisabled(False)
            self.uic.btn_j3_dec.setDisabled(False)
            self.uic.btn_j1_inc.setDisabled(False)
            self.uic.btn_j2_inc.setDisabled(False)
            self.uic.btn_j3_inc.setDisabled(False)
            self.uic.txt_mX.setText('0')
            self.uic.txt_mY.setText('0')
            self.uic.txt_mZ.setText('0')
        else:
            self.msg.setIcon(QMessageBox.Warning)
            self.msg.setText("Check your COM Port")
            self.msg.setWindowTitle("Error")
            self.msg.setStandardButtons(QMessageBox.Ok)
            self.msg.exec()

    def calib_home(self):
        self.buffer = "2,4,0,0,0,"
        isSend = serialCom.send_data(self.buffer)
        self.check_sending(isSend)
        if self.isPortCnt:
            self.home()

    # endregion

    # region joint mode
    def set_degree(self):
        deg = self.uic.slide_deg.value()
        self.uic.txt_deg.setText(str(deg))

    def j1_decrease(self):
        step = int(self.uic.slide_deg.value())
        self.theta1 = float(self.uic.txt_j1.toPlainText()) - step
        self.uic.txt_j1.setText(str(self.theta1))
        self.forwardKinematics(theta1=self.theta1, theta2=self.theta2, theta3=self.theta3)

    def j1_increase(self):
        step = int(self.uic.slide_deg.value())
        self.theta1 = float(self.uic.txt_j1.toPlainText()) + step
        self.uic.txt_j1.setText(str(self.theta1))
        self.forwardKinematics(theta1=self.theta1, theta2=self.theta2, theta3=self.theta3)

    def j2_decrease(self):
        step = int(self.uic.slide_deg.value())
        self.theta2 = float(self.uic.txt_j2.toPlainText()) - step
        self.uic.txt_j2.setText(str(self.theta2))
        self.forwardKinematics(theta1=self.theta1, theta2=self.theta2, theta3=self.theta3)

    def j2_increase(self):
        step = int(self.uic.slide_deg.value())
        self.theta2 = float(self.uic.txt_j2.toPlainText()) + step
        self.uic.txt_j2.setText(str(self.theta2))
        self.forwardKinematics(theta1=self.theta1, theta2=self.theta2, theta3=self.theta3)

    def j3_decrease(self):
        step = int(self.uic.slide_deg.value())
        self.theta3 = float(self.uic.txt_j3.toPlainText()) - step
        self.uic.txt_j3.setText(str(self.theta3))
        self.forwardKinematics(theta1=self.theta1, theta2=self.theta2, theta3=self.theta3)

    def j3_increase(self):
        step = int(self.uic.slide_deg.value())
        self.theta3 = float(self.uic.txt_j3.toPlainText()) + step
        self.uic.txt_j3.setText(str(self.theta3))
        self.forwardKinematics(theta1=self.theta1, theta2=self.theta2, theta3=self.theta3)

    # endregion

    # region world mode
    def set_centimeters(self):
        centi = self.uic.slide_cm.value()
        self.uic.txt_cm.setText(str(centi))

    def x_decrease(self):
        step = int(self.uic.slide_cm.value())
        self.Px = float(self.uic.txt_x.toPlainText()) - step
        self.uic.txt_x.setText(str(round(self.Px, 2)))
        self.inverseKinematics(px=self.Px, py=self.Py, pz=self.Pz)

    def x_increase(self):
        step = int(self.uic.slide_cm.value())
        self.Px = float(self.uic.txt_x.toPlainText()) + step
        self.uic.txt_x.setText(str(round(self.Px, 2)))
        self.inverseKinematics(px=self.Px, py=self.Py, pz=self.Pz)

    def y_decrease(self):
        step = int(self.uic.slide_cm.value())
        self.Py = float(self.uic.txt_y.toPlainText()) - step
        self.uic.txt_y.setText(str(round(self.Py, 2)))
        self.inverseKinematics(px=self.Px, py=self.Py, pz=self.Pz)

    def y_increase(self):
        step = int(self.uic.slide_cm.value())
        self.Py = float(self.uic.txt_y.toPlainText()) + step
        self.uic.txt_y.setText(str(round(self.Py, 2)))
        self.inverseKinematics(px=self.Px, py=self.Py, pz=self.Pz)

    def z_decrease(self):
        step = int(self.uic.slide_cm.value())
        self.Pz = float(self.uic.txt_z.toPlainText()) - step
        self.uic.txt_z.setText(str(round(self.Pz, 2)))
        self.inverseKinematics(px=self.Px, py=self.Py, pz=self.Pz)

    def z_increase(self):
        step = int(self.uic.slide_cm.value())
        self.Pz = float(self.uic.txt_z.toPlainText()) + step
        self.uic.txt_z.setText(str(round(self.Pz, 2)))
        self.inverseKinematics(px=self.Px, py=self.Py, pz=self.Pz)

    # endregion

    # region moving configuration
    def set_speed(self):
        sp = self.uic.slide_speed.value()
        self.uic.txt_speed.setText(str(sp))

    def set_accelerate(self):
        acc = self.uic.slide_acc.value()
        self.uic.txt_acc.setText(str(acc))

    def apply_conf(self):
        vel = self.uic.slide_speed.value()
        acc = self.uic.slide_acc.value()
        self.buffer = "0" + "," + str(vel) + "," + str(acc) + "," + "0" + ',' + '0' + ','
        isSend = serialCom.send_data(self.buffer)
        self.check_sending(isSend)

    # endregion

    # region moving robot
    def move(self):
        self.Px = float(self.uic.txt_mX.toPlainText())
        self.Py = float(self.uic.txt_mY.toPlainText())
        self.Pz = float(self.uic.txt_mZ.toPlainText())
        self.uic.txt_x.setText(str(self.Px))
        self.uic.txt_y.setText(str(self.Py))
        self.uic.txt_z.setText(str(self.Pz))
        self.inverseKinematics(px=self.Px, py=self.Py, pz=self.Pz)

    # endregion

    # region robot 3 dof kinematics
    def forwardKinematics(self, theta1, theta2, theta3):
        self.buffer = "3" + "," + str(theta1) + "," + str(theta2) + "," + str(theta3) + ',' + '0' + ','
        isSend = serialCom.send_data(self.buffer)
        self.check_sending(isSend)
        if isSend:
            theta1 = round(float(theta1) * math.pi / 180, 2)
            theta2 = round(float(theta2) * math.pi / 180, 2)
            theta3 = round(float(theta3) * math.pi / 180, 2)
            self.Px = round(math.cos(theta1) * (self.a4 + self.a3 * math.cos(theta2 + theta3) + self.a2
                                                * math.cos(theta2)), 2)
            self.Py = round(math.sin(theta1) * (self.a4 + self.a3 * math.cos(theta2 + theta3) + self.a2
                                                * math.cos(theta2)), 2)
            self.Pz = round(self.d1 - self.a5 + self.a3 * math.sin(theta2 + theta3) + self.a2 * math.sin(theta2), 2)
            self.Pitch = 90
            self.Roll = 0
            self.Yaw = round(90 - float(theta1), 2)
            self.uic.txt_cX.setText(str(self.Px))
            self.uic.txt_cY.setText(str(self.Py))
            self.uic.txt_cZ.setText(str(self.Pz))
            self.uic.txt_cRoll.setText(str(self.Roll))
            self.uic.txt_cPitch.setText(str(self.Pitch))
            self.uic.txt_cYaw.setText(str(self.Yaw))
            self.uic.txt_x.setText(str(self.Px))
            self.uic.txt_y.setText(str(self.Py))
            self.uic.txt_z.setText(str(self.Pz))

    def inverseKinematics(self, px, py, pz):
        self.theta1 = math.atan2(py, px)
        pwx = px - self.a4 * math.cos(self.theta1)
        pwy = py - self.a4 * math.sin(self.theta1)
        pwz = pz + self.a5
        r = math.sqrt(pwx ** 2 + pwy ** 2 + (pwz - self.d1) ** 2)
        self.theta3 = -math.acos((r ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3))
        self.theta2 = math.asin((pwz - self.d1) / r) + math.atan(self.a3 * math.sin(abs(self.theta3))
                                                                 / (self.a2 + self.a3 * math.cos(abs(self.theta3))))
        self.theta1 = round(self.theta1 * 180 / math.pi, 2)
        self.theta2 = round(self.theta2 * 180 / math.pi, 2)
        self.theta3 = round(self.theta3 * 180 / math.pi, 2)
        self.Pitch = 90
        self.Roll = 0
        self.Yaw = round(90 - self.theta1, 2)
        self.buffer = "3" + "," + str(self.theta1) + "," + str(self.theta2) + "," + str(self.theta3) + ',' + '0' + ','
        isSend = serialCom.send_data(self.buffer)
        self.check_sending(isSend)
        if isSend:
            self.uic.txt_j1.setText(str(self.theta1))
            self.uic.txt_j2.setText(str(self.theta2))
            self.uic.txt_j3.setText(str(self.theta3))
            self.uic.txt_cX.setText(str(round(px, 2)))
            self.uic.txt_cY.setText(str(round(py, 2)))
            self.uic.txt_cZ.setText(str(round(pz, 2)))
            self.uic.txt_cRoll.setText(str(self.Roll))
            self.uic.txt_cPitch.setText(str(self.Pitch))
            self.uic.txt_cYaw.setText(str(self.Yaw))

    # endregion

    # region on/off camera
    def start(self):
        icon1, icon2 = QIcon(), QIcon()
        icon1.addFile(u"turn_on.PNG", QSize(), QIcon.Normal, QIcon.Off)
        icon2.addFile(u"turn_off.PNG", QSize(), QIcon.Normal, QIcon.Off)
        if self.uic.btn_start.text() == " Start Cam":
            if mycam.connect_cam():
                self.uic.btn_start.setIcon(icon2)
                self.uic.btn_start.setIconSize(QSize(40, 40))
                self.uic.btn_start.setText(" Stop Cam")
                self.isOpenCam = True
        else:
            if mycam.disconnect_cam():
                self.uic.btn_start.setIcon(icon1)
                self.uic.btn_start.setIconSize(QSize(40, 40))
                self.uic.btn_start.setText(" Start Cam")
                self.isOpenCam = False

        def display():
            try:
                while self.isOpenCam:
                    if self.isCalibMode:
                        self.uic.obj_view.clear()
                        self.uic.size_view.clear()
                        self.uic.logo_view.clear()
                        break
                    image = mycam.get_frames()
                    obj, logo = models_process.detect_object(image)
                    if models_process.isObj:
                        size = models_process.detect_size_color(image)
                        size = cv2.resize(size, (500, 350), interpolation=cv2.INTER_LINEAR)
                        size = QImage(size, size.shape[1], size.shape[0], QImage.Format_BGR888)
                        self.uic.size_view.setPixmap(QPixmap.fromImage(size))
                        models_process.elogo = logo
                    else:
                        models_process.prob.reset_bel()
                    obj = cv2.resize(obj, (500, 350), interpolation=cv2.INTER_LINEAR)
                    obj = QImage(obj, obj.shape[1], obj.shape[0], QImage.Format_BGR888)
                    self.uic.obj_view.setPixmap(QPixmap.fromImage(obj))
                    elogo = models_process.logo
                    if elogo:
                        elogo = cv2.resize(elogo, (300, 280), interpolation=cv2.INTER_LINEAR)
                        elogo = QImage(elogo, elogo.shape[1], elogo.shape[0], QImage.Format_BGR888)
                        self.uic.logo_view.setPixmap(QPixmap.fromImage(elogo))

                if not self.isOpenCam:
                    self.uic.obj_view.clear()
                    self.uic.size_view.clear()
                    self.uic.logo_view.clear()
            except Exception as e:
                print(e)
                pass

        display_thread = threading.Thread(target=display)
        display_thread.start()
        logo_detect_thread = threading.Thread(target=models_process.detect_elogo)
        logo_detect_thread.start()

    def calib_model_mode(self):
        if self.uic.btn_calib_mode.text() == "On Mode":
            self.uic.btn_calib_size.setDisabled(False)
            self.uic.btn_test_size.setDisabled(False)
            self.uic.btn_calib_mode.setText("Off Mode")
            self.isCalibMode = True
        else:
            self.uic.btn_calib_size.setDisabled(True)
            self.uic.btn_test_size.setDisabled(True)
            self.uic.btn_calib_mode.setText("On Mode")
            self.isCalibMode = False

        def display():
            while self.isCalibMode:
                image = mycam.get_frames()
                img_org, img_contour = preprocessing_img(image)
                img, _, d1, d2, d3 = find_features(img_org, img_contour)

        display_thread = threading.Thread(target=display)
        display_thread.start()

    # endregion


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.Control_UI.show()
    sys.exit(app.exec_())
