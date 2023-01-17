from PySide2.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide2.QtGui import QIcon, QImage, QPixmap
from PySide2.QtCore import QSize, Qt
from GUI import gui
from communicate import serialCom, receive_thread
from camera import mycam
from detect import models_process
from preprocessing import preprocessing_img
from find_feature import find_features
from database import my_database
from calib_h import size_calib
import math
import sys
import threading


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
        self.isCalib = False
        self.isTestCalib = False
        self.detect_result = ['not detect', 'not detect', 'not detect']
        self.center_result = [0, 0]
        self.pick_place = None
        self.drop_place = None
        self.type_dict = {}
        self.coordinate_dict = {}
        self.isAutoMode = False
        self.isRobotAvailable = False

        # region robot params
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
        # endregion

        # region setup gui object properties
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
        self.uic.btn_savepoint.setDisabled(True)
        self.uic.btn_stop.setDisabled(True)
        self.uic.btn_reset.setDisabled(True)

        # endregion

        # region events
        # event combo box
        self.uic.cb_com.activated.connect(self.list_ports())
        self.uic.cb_baudrate.activated.connect(self.list_baudrate())
        self.uic.cb_point.activated.connect(self.list_points())
        self.uic.cb_point.currentTextChanged.connect(self.changed_point)
        self.uic.cb_sizetype.activated.connect(self.list_sizetype())
        self.uic.cb_colortype.activated.connect(self.list_colortype())

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
        self.uic.btn_start.clicked.connect(self.start_cam)
        self.uic.btn_calib_mode.clicked.connect(self.calib_model_mode)
        self.uic.btn_calib_size.clicked.connect(self.is_calib)
        self.uic.btn_test_size.clicked.connect(self.is_test_calib)
        self.uic.rbutton_new.clicked.connect(self.teaching_mode)
        self.uic.btn_savepoint.clicked.connect(self.save_point)
        self.uic.btn_reset_point.clicked.connect(self.reset_point)
        self.uic.btn_run.clicked.connect(self.run)
        self.uic.btn_stop.clicked.connect(self.stop)
        self.uic.btn_reset.clicked.connect(self.reset)

        # event slide
        self.uic.slide_speed.valueChanged.connect(self.set_speed)
        self.uic.slide_acc.valueChanged.connect(self.set_accelerate)
        self.uic.slide_deg.valueChanged.connect(self.set_degree)
        self.uic.slide_cm.valueChanged.connect(self.set_centimeters)
        # endregion

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
                self.uic.light_stt.setStyleSheet(
                    "background: rgb(51,255,51); border-radius:20px; border-color: rgb(0, 0, 0); "
                    "border-width : 0.5px; border-style:inset;")
                self.uic.btn_cnt.setText("Disconnect")
                self.uic.text_stt.setText("Status: On")
            else:
                self.uic.text_stt.setText("Status: Off")
                self.uic.light_stt.setStyleSheet(
                    "background: rgb(255,0,0); border-radius:20px; border-color: rgb(0, 0, 0); "
                    "border-width : 0.5px; border-style:inset;")
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

    # region teaching pendant
    def list_points(self):
        p = ['P00', 'P01', 'P02', 'P03', 'P04']
        self.uic.cb_point.addItems(p)
        self.uic.cb_sizetype.setDisabled(True)
        self.uic.cb_colortype.setDisabled(True)

    def changed_point(self):
        if self.uic.cb_point.currentText() == "P00":
            self.uic.cb_sizetype.setDisabled(True)
            self.uic.cb_colortype.setDisabled(True)
        else:
            self.uic.cb_sizetype.setDisabled(False)
            self.uic.cb_colortype.setDisabled(False)

    def list_sizetype(self):
        t = ['1', '2']
        self.uic.cb_sizetype.addItems(t)

    def list_colortype(self):
        t = ['1', '2']
        self.uic.cb_colortype.addItems(t)

    def teaching_mode(self):
        if self.uic.rbutton_new.isChecked():
            self.uic.btn_savepoint.setDisabled(False)
            self.uic.txt_info_teaching.setText(my_database.get_table_comment())
        else:
            self.uic.btn_savepoint.setDisabled(True)

    @staticmethod
    def re_text(dic):
        text = "[/point/, /size/, /color/, /x/, /y/, /z/]"
        for unit in dic:
            text = text + '\n' + str(unit)
        return text

    @staticmethod
    def reset_point():
        my_database.delete_on_database()

    def save_point(self):
        self.uic.btn_savepoint.setDisabled(True)
        self.uic.rbutton_new.setChecked(False)
        _p = self.uic.cb_point.currentText()
        _size = self.uic.cb_sizetype.currentText()
        _color = self.uic.cb_colortype.currentText()
        if _p == "P00":
            _size = '0'
            _color = '0'
        _x = self.uic.txt_cX.toPlainText()
        _y = self.uic.txt_cY.toPlainText()
        _z = self.uic.txt_cZ.toPlainText()
        my_database.save_into_database(_p, _size, _color, _x, _y, _z)
        if not _x and not _y and not _z:
            try:
                my_database.save_into_database(_p, _size, _color, _x, _y, _z)
            except:
                my_database.update_into_database(_p, _size, _color, _x, _y, _z)
            self.uic.txt_info_teaching.setText(self.re_text(my_database.read_from_database()))
        else:
            self.msg.setIcon(QMessageBox.Warning)
            self.msg.setText("Check your teaching point")
            self.msg.setWindowTitle("Error")
            self.msg.setStandardButtons(QMessageBox.Ok)
            self.msg.exec()

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

    # region camera detection and robotic control
    def start_cam(self):
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
            if self.isCalibMode:
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText("Calib mode is online")
                self.msg.setWindowTitle("Error")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.exec()
                return
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

                    # -> detect frames
                    image = mycam.get_frames()
                    obj, logo = models_process.detect_object(image)
                    if models_process.isObj:
                        size, result1, result2, center_x, center_y = models_process.detect_size_color(image)
                        size = self.convert_cv_qt(size, 550, 375)
                        self.uic.size_view.setPixmap(size)
                        elogo, result3 = models_process.detect_elogo(logo)
                        elogo = self.convert_cv_qt(elogo, 360, 300)
                        self.uic.logo_view.setPixmap(elogo)
                        self.detect_result = [result1, result2, result3]
                        self.center_result = [center_x, center_y]
                    else:
                        models_process.prob.reset_bel()
                    obj = self.convert_cv_qt(obj, 550, 375)
                    self.uic.obj_view.setPixmap(obj)

                    # -> control mode on
                    if self.isAutoMode:
                        """ 
                        when robot is available, then send center of robot coordinate:
                        - step 1: if isObj is true, check robot is available ? by read the buffer.
                        - step 2: if yes, check detect result (size, color, logo) ?
                        - step 3: if one of detect result is error, back step 1, else next step.
                        - step 4: convert center x, center y to robot coordinate (pick place).
                        - step 5: validate drop place from dict (type, coordinate).
                        - step 6: inverse kinematic pick place and drop place.
                        - step 7: send that points to mcu, set robot unavailable.
                        
                        when robot is unavailable, and there is a object which is seen, then adjust the velocity of
                        conveyor motor.
                        - case 1: check if isObj is False, set default velocity.
                        - case 2: check if isObj is true, and robot is unavailable, adjust the velocity.
                        - case 3: check if isObj is true, and robot is available, set default velocity.
                        """
                        if models_process.isObj:
                            # read buffer -> validate robot is available?
                            pass
                        else:
                            # set default velocity
                            pass

                if not self.isOpenCam:
                    self.uic.obj_view.clear()
                    self.uic.size_view.clear()
                    self.uic.logo_view.clear()
            except Exception as e:
                print(e)
                pass

        display_thread = threading.Thread(target=display)
        display_thread.start()

    def calib_model_mode(self):
        if self.uic.btn_calib_mode.text() == "On Mode":
            if not self.isOpenCam:
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText("Check your Camera")
                self.msg.setWindowTitle("Error")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.exec()
                return
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
                if self.isOpenCam:
                    image = mycam.get_frames()
                    img_org, img_contour = preprocessing_img(image)
                    img, _, d1, d2, d3 = find_features(img_org, img_contour)
                    size = self.convert_cv_qt(img, 550, 375)
                    self.uic.size_view.setPixmap(size)
                    if self.isCalib:
                        self.isCalib = False
                        size_calib.convert(d1, d2, d3)
                        models_process.cf1 = size_calib.cf1
                        models_process.cf2 = size_calib.cf2
                        models_process.cf3 = size_calib.cf3
                        print(models_process.cf1, models_process.cf2, models_process.cf3)

                    if self.isTestCalib:
                        self.isTestCalib = False
                        result = size_calib.test(d1, d2, d3)
                        print(result)

            if not self.isCalibMode:
                self.uic.size_view.clear()

        display_thread = threading.Thread(target=display)
        display_thread.start()

    def is_calib(self):
        self.isCalib = True

    def is_test_calib(self):
        self.isTestCalib = True

    @staticmethod
    def convert_cv_qt(cv_img, dis_width, dis_height):
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        qt_format = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888)
        p = qt_format.scaled(dis_width, dis_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    @staticmethod
    def calib_camera():
        pass

    @staticmethod
    def convert_robot_coordinate():
        pass

    @staticmethod
    def validate_robot_available():
        pass

    # endregion

    # region run auto mode
    def convert_teaching_data(self):
        table = my_database.read_from_database()
        """ example: 
            {(1, 1): "P01"} --- type_dict
            {"P01": [5, 10, 10]} --- coordinate_dict
        """
        if table:
            for row in table:
                self.type_dict.update({(row[1], row[2]): row[0]})
                self.coordinate_dict.update({row[0]: [row[3], row[4], row[5]]})
            return 1
        return 0

    def run(self):
        # read teaching point from database, if not find data, return false mode
        if not self.convert_teaching_data():
            self.msg.setIcon(QMessageBox.Warning)
            self.msg.setText("Check your teaching point")
            self.msg.setWindowTitle("Error")
            self.msg.setStandardButtons(QMessageBox.Ok)
            self.msg.exec()
            self.isAutoMode = False
            return

        # setup properties
        self.uic.btn_run.setDisabled(True)
        self.uic.btn_stop.setDisabled(False)
        self.uic.light_stt_2.setStyleSheet(
            "background: rgb(51,255,51); border-radius:35px; border-color: rgb(0, 0, 0); "
            "border-width : 1px; border-style:inset;")
        self.uic.text_stt_2.setText('Status: On')

        # validate robot is available ? by asking mcu
        # .....send buffer

        # run conveyor motor
        # .....send buffer

        # on mode
        self.isAutoMode = True

    def stop(self):
        self.uic.btn_run.setDisabled(False)
        self.uic.btn_stop.setDisabled(True)
        self.uic.btn_reset.setDisabled(False)
        self.uic.light_stt_2.setStyleSheet(
            "background: rgb(255,0,0); border-radius:35px; border-color: rgb(0, 0, 0); "
            "border-width : 1px; border-style:inset;")
        self.uic.text_stt_2.setText('Status: Off')

        # stop conveyor motor
        # .....send buffer

        # off mode
        self.isAutoMode = False

    def reset(self):
        self.uic.btn_run.setDisabled(False)
        self.uic.btn_stop.setDisabled(True)
        self.uic.btn_reset.setDisabled(True)
        self.uic.light_stt_2.setStyleSheet(
            "background: rgb(255,255,0); border-radius:35px; border-color: rgb(0, 0, 0); "
            "border-width : 1px; border-style:inset;")
        self.uic.text_stt_2.setText('Status: Reset')

        # robot go home
        self.home()

    # endregion


if __name__ == '__main__':
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.Control_UI.show()
    sys.exit(app.exec_())
