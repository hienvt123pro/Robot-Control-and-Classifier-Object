from PySide2.QtWidgets import QApplication, QMainWindow, QMessageBox, QGraphicsPixmapItem, QGraphicsScene
from PySide2.QtGui import QIcon, QImage, QPixmap
from PySide2.QtCore import QSize, Qt
from PySide2 import QtCore
from GUI import gui
from communicate import serialCom1, receive_thread_1, serialCom2, receive_thread_2
from camera import mycam
from detect import SizeAndColorProcess, ObjectProcess
from find_feature import find_features
from preprocessing import preprocessing_img
from database import my_database
from calibration import size_calib, camera_calib
from robot_code import robocod
from predict_point import rfPoint
import math
import sys
import threading
import matplotlib.pyplot as plt

# import time


class MainWindow:
    def __init__(self):
        self.Control_UI = QMainWindow()
        self.uic = gui.Ui_MainWindow()
        self.uic.setupUi(self.Control_UI)
        self.msg = QMessageBox()
        self.buffer = "0,0,0,0,0,0,0,0,0,0,0"
        self.isPortsCnt = False
        self.isOpenCam = False
        self.isCalibSizeModelMode = False
        self.isSizeCalibModel = False
        self.isTestCalibSizeModel = False
        self.detect_result = ['', '']
        self.product_result = ''
        self.center_result = [0, 0, 1]
        self.pick_place = tuple([0, 0, 0])
        self.temp_pick_place = tuple([0, 0, 0])
        self.drop_place = tuple([0, 0, 0])
        self.type_dict = {}
        self.coordinate_dict = {}
        self.index_type = 0
        self.isAutoMode = False
        self.isRobotAvailable = False
        self.isCalibCamMode = False
        self.isTakeShot = False
        self.isCalibCam = False
        self.isTestCamCalibModel = False
        self.isRunConveyor = False
        self.isReadSpeedMode = False
        self.isApplyRF = False

        # timer interrupt
        self.timer = QtCore.QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.update_results)

        # PID graph setting
        self.scene = QGraphicsScene()
        self.uic.graphicsView.setScene(self.scene)
        self.x, self.t = [], 0
        self.y_pv, self.y_sp = [], []
        self.fig = plt.figure()
        self.fig.set_size_inches(550/100, 360/90)
        plt.xlabel('time (s)')
        plt.ylabel('speed conveyor (cm/s)')
        plt.title("PID Control")
        plt.ylim([0, 10])

        # region robot params
        self.d1 = 12.9
        self.a2 = 14
        self.a3 = 14
        self.a4 = 5
        self.a5 = 3
        self.Px = 0
        self.Py = 0
        self.Pz = 0
        self.Roll = 0
        self.Pitch = 0
        self.Yaw = 0
        self.theta1 = 0
        self.theta2 = 0
        self.theta3 = 0
        self.robotHome = [0, 100, -130]
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
        self.uic.btn_calib_mode_cam.setDisabled(True)
        self.uic.btn_scrshot.setDisabled(True)
        self.uic.btn_calib_cam.setDisabled(True)
        self.uic.btn_save_model.setDisabled(True)
        self.uic.txt_convey_sp_rf.setText("3")

        # endregion

        # region events
        # even text edit
        self.uic.txt_setpoint_sp.textChanged.connect(self.sp_convey_changed)

        # event combo box
        self.uic.cb_com.activated.connect(self.list_ports())
        self.uic.cb_com_2.activated.connect(self.list_ports())
        self.uic.cb_baudrate.activated.connect(self.list_baudrate())
        self.uic.cb_point.activated.connect(self.list_points())
        self.uic.cb_point.currentTextChanged.connect(self.changed_point)
        self.uic.cb_sizetype.activated.connect(self.list_sizetype())
        self.uic.cb_opt_calib_cam.activated.connect(self.list_opt_calib())
        self.uic.cb_opt_calib_cam.currentTextChanged.connect(self.changed_opt)
        self.uic.cb_calib_model.activated.connect(self.list_calibrated_model())
        self.uic.cb_low_area.activated.connect(self.list_low_area())
        self.uic.cb_high_area.activated.connect(self.list_high_area())

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
        self.uic.btn_calib_mode_cam.clicked.connect(self.calib_cam_model)
        self.uic.btn_scrshot.clicked.connect(self.take_screen_shot)
        self.uic.btn_calib_cam.clicked.connect(self.is_calib_cam)
        self.uic.btn_save_model.clicked.connect(self.is_save_cam_calib_model)
        self.uic.btn_apply_calib_model.clicked.connect(self.apply_calib_cam_model)
        self.uic.btn_on_cam_test.clicked.connect(self.test_calib_cam_model)
        self.uic.btn_end_effector.clicked.connect(self.end_effector)
        self.uic.btn_apply_rf.clicked.connect(self.apply_rf_model)
        self.uic.btn_run_convey.clicked.connect(self.run_conveyor)

        # event slide
        self.uic.slide_speed.valueChanged.connect(self.set_speed)
        self.uic.slide_acc.valueChanged.connect(self.set_accelerate)
        self.uic.slide_deg.valueChanged.connect(self.set_degree)
        self.uic.slide_cm.valueChanged.connect(self.set_centimeters)
        self.uic.slide_points_test.valueChanged.connect(self.set_num_points)
        # endregion

    # region port connection
    def list_ports(self):
        self.uic.cb_com.clear()
        self.uic.cb_com_2.clear()
        ports = serialCom1.list_ports_name()
        if not ports:
            self.uic.cb_com.addItem("not found")
            self.uic.cb_com_2.addItem("not found")
            return
        name_ports = []
        for port in ports:
            p = str(port)
            name_ports.append(p[:5])
        self.uic.cb_com.addItems(list(set(name_ports)))
        self.uic.cb_com_2.addItems(list(set(name_ports)))

    def list_baudrate(self):
        self.uic.cb_baudrate.addItem("115200")

    def connection(self):
        self.list_ports()
        if self.uic.btn_cnt.text() == "Connect":
            port_name_1 = self.uic.cb_com.currentText()
            port_name_2 = self.uic.cb_com_2.currentText()
            if port_name_1 == port_name_2:
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText("Duplicate COM Ports at 2 device\nor ports are not available")
                self.msg.setWindowTitle("Error")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.exec()
                return

            baudrate = self.uic.cb_baudrate.currentText()
            self.isPortsCnt = serialCom1.connect(port_name=port_name_1, baud=baudrate) and \
                              serialCom2.connect(port_name=port_name_2, baud=baudrate)
            if self.isPortsCnt:
                try:
                    # start receive data from mcu thread
                    receive_thread_1.start()
                    receive_thread_2.start()
                except RuntimeError:
                    pass
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
            serialCom1.disconnect()
            serialCom2.disconnect()
            self.isPortsCnt = False
            self.uic.btn_cnt.setText("Connect")
            self.uic.text_stt.setText("Status: Off")
            self.uic.light_stt.setStyleSheet("background: rgb(255,0,0); border-radius:20px")

    # endregion

    # region teaching pendant
    def list_points(self):
        p = ['P00', 'P01', 'P02', 'P03']
        self.uic.cb_point.addItems(p)
        self.uic.cb_sizetype.setDisabled(True)

    def changed_point(self):
        if self.uic.cb_point.currentText() == "P00":
            self.uic.cb_sizetype.setDisabled(True)
        else:
            self.uic.cb_sizetype.setDisabled(False)

    def list_sizetype(self):
        t = ['1', '2', '3']
        self.uic.cb_sizetype.addItems(t)

    def teaching_mode(self):
        if self.uic.rbutton_new.isChecked():
            self.uic.btn_savepoint.setDisabled(False)
            self.uic.txt_info_teaching.setText(my_database.get_table_comment())
        else:
            self.uic.btn_savepoint.setDisabled(True)

    @staticmethod
    def re_text(dic):
        text = "|  point  |  size  |  x  |  y  |  z  |"
        for unit in dic:
            text = text + '\n' + str(unit)
        return text

    def reset_point(self):
        my_database.delete_on_database()
        self.uic.txt_info_teaching.setText("\n Clear all data!")

    def save_point(self):
        self.uic.btn_savepoint.setDisabled(True)
        self.uic.rbutton_new.setChecked(False)
        _p = self.uic.cb_point.currentText()
        _size = self.uic.cb_sizetype.currentText()
        if _p == "P00":
            _size = '0'
        _x = self.uic.txt_cX.toPlainText()
        _y = self.uic.txt_cY.toPlainText()
        _z = self.uic.txt_cZ.toPlainText()
        if float(_x) != 0 and float(_y) != 0 and float(_z) != 0:
            try:
                my_database.save_into_database(_p, _size, _x, _y, _z)
            except Exception as e:
                print(e, "-save point")
                my_database.update_into_database(_p, _size, _x, _y, _z)
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
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.CALIBJ1, data=None)
        isSend = serialCom1.send_data(self.buffer)
        self.check_sending(isSend)

    def calib_J2(self):
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.CALIBJ2, data=None)
        isSend = serialCom1.send_data(self.buffer)
        self.check_sending(isSend)

    def calib_J3(self):
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.CALIBJ3, data=None)
        isSend = serialCom1.send_data(self.buffer)
        self.check_sending(isSend)

    def home(self):
        if self.isPortsCnt:
            self.forwardKinematics(theta1=self.robotHome[0], theta2=self.robotHome[1], theta3=self.robotHome[2])
            self.uic.txt_j1.setText(str(self.robotHome[0]))
            self.uic.txt_j2.setText(str(self.robotHome[1]))
            self.uic.txt_j3.setText(str(self.robotHome[2]))
            self.theta1 = self.robotHome[0]
            self.theta2 = self.robotHome[1]
            self.theta3 = self.robotHome[2]
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
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.CALIBHOME, data=None)
        isSend = serialCom1.send_data(self.buffer)
        self.check_sending(isSend)
        if self.isPortsCnt:
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

    # region end-effector
    def end_effector(self):
        if self.uic.btn_end_effector.text() == "On":
            self.buffer = robocod.concatenate(dev="1", cmd=robocod.EFFECTOR, data=[1, 0, 0, 0, 0, 0])
            isSend = serialCom1.send_data(self.buffer)
            self.check_sending(isSend)
            if not isSend:
                return
            self.uic.btn_end_effector.setText("Off")
        else:
            self.buffer = robocod.concatenate(dev="1", cmd=robocod.EFFECTOR, data=None)
            isSend = serialCom1.send_data(self.buffer)
            self.check_sending(isSend)
            if not isSend:
                return
            self.uic.btn_end_effector.setText("On")

    # endregion

    # region speed configuration
    def set_speed(self):
        sp = self.uic.slide_speed.value()
        if sp >= 35000:
            color = tuple([255, 0, 0])
        elif sp >= 25000:
            color = tuple([255, 167, 15])
        elif sp >= 15000:
            color = tuple([0, 255, 0])
        elif sp >= 5000:
            color = tuple([21, 165, 255])
        else:
            color = tuple([166, 254, 255])
        self.uic.light_stt_sp.setStyleSheet(
            f"background: rgb{color}; border-radius:10px; border-color: rgb(0, 0, 0); "
            "border-width : 0.5px; border-style:inset;")

    def set_accelerate(self):
        acc = self.uic.slide_acc.value()
        if acc >= 35000:
            color = tuple([255, 0, 0])
        elif acc >= 25000:
            color = tuple([255, 167, 15])
        elif acc >= 15000:
            color = tuple([0, 255, 0])
        elif acc >= 5000:
            color = tuple([21, 165, 255])
        else:
            color = tuple([166, 254, 255])
        self.uic.light_stt_acc.setStyleSheet(
            f"background: rgb{color}; border-radius:10px; border-color: rgb(0, 0, 0); "
            "border-width : 0.5px; border-style:inset;")

    def apply_conf(self):
        vel = self.uic.slide_speed.value()
        acc = self.uic.slide_acc.value()
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.SCON, data=[vel, acc, 0, 0, 0, 0])
        isSend = serialCom1.send_data(self.buffer)
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
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.MOVE, data=[theta1, theta2, theta3, 0, 0, 0])
        isSend = serialCom1.send_data(self.buffer)
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
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.MOVE,
                                          data=[self.theta1, self.theta2, self.theta3, 0, 0, 0])
        isSend = serialCom1.send_data(self.buffer)
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

    def inverseForAutoMode(self, auto_point: list):
        """
        :param auto_point: pick-place, drop-place in world coordinate
        :return: rotated angles for robot
        """
        output_point = []
        for p in auto_point:
            theta1 = math.atan2(p[1], p[0])
            pwx = p[0] - self.a4 * math.cos(theta1)
            pwy = p[1] - self.a4 * math.sin(theta1)
            pwz = p[2] + self.a5
            r = math.sqrt(pwx ** 2 + pwy ** 2 + (pwz - self.d1) ** 2)
            theta3 = -math.acos((r ** 2 - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3))
            theta2 = math.asin((pwz - self.d1) / r) + math.atan(self.a3 * math.sin(abs(theta3))
                                                                / (self.a2 + self.a3 * math.cos(abs(theta3))))
            theta1 = round(theta1 * 180 / math.pi, 2)
            theta2 = round(theta2 * 180 / math.pi, 2)
            theta3 = round(theta3 * 180 / math.pi, 2)
            output_point.append((theta1, theta2, theta3))
        return output_point

    def sendProcessMove(self, pick: tuple, drop: tuple):
        """
        :param pick: rotated angle of pick-place
        :param drop: rotated angle of drop-place
        :return: transfer data to mcu
        """
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.PROCESS_MOVE,
                                          data=[pick[0], pick[1], pick[2], drop[0], drop[1], drop[2]])
        isSend = serialCom1.send_data(self.buffer)
        self.check_sending(isSend)

    # endregion

    # region camera detection and robotic control
    def start_cam(self):
        icon1, icon2 = QIcon(), QIcon()
        icon1.addFile(u"GUI/icon_turn_on.PNG", QSize(), QIcon.Normal, QIcon.Off)
        icon2.addFile(u"GUI/icon_turn_off.PNG", QSize(), QIcon.Normal, QIcon.Off)
        if self.uic.btn_start.text() == " Start Cam":
            if self.isCalibSizeModelMode or self.isTestCamCalibModel:
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText("Camera calibration mode is online")
                self.msg.setWindowTitle("Error")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.exec()
                return
            if camera_calib.Z == 0:
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText("Calibration model is not found")
                self.msg.setWindowTitle("Error")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.exec()
                return
            if mycam.connect_cam():
                self.uic.btn_start.setIcon(icon2)
                self.uic.btn_start.setIconSize(QSize(40, 40))
                self.uic.btn_start.setText(" Stop Cam")
                self.isOpenCam = True
                self.uic.lb_info.setText("Start detection models")
                self.uic.res_size.setStyleSheet("background: rgb(255,255,255);")
                if not self.timer.isActive():
                    self.timer.start()
        else:
            self.isOpenCam = False
            if mycam.disconnect_cam():
                self.uic.btn_start.setIcon(icon1)
                self.uic.btn_start.setIconSize(QSize(40, 40))
                self.uic.btn_start.setText(" Start Cam")
                self.uic.obj_view.clear()
                self.uic.size_view.clear()
                self.uic.logo_view.clear()
                self.uic.lb_info.setText("Terminate detection models")
                self.timer.stop()

        def display():
            try:
                while self.isOpenCam:
                    if self.isCalibSizeModelMode:
                        self.uic.obj_view.clear()
                        self.uic.size_view.clear()
                        break

                    # Detect frames
                    # start_time = time.time()
                    image = mycam.get_frames()
                    isObject, obj, logo, bbox = object_processing.obj_detector_process(image)
                    self.uic.obj_view.setPixmap(self.convert_cv_qt(obj, 600, 450))
                    if isObject:
                        size, size_result, color_result, center_x, center_y = sz_color_processing.detect_size_color(
                            image, bbox[0], bbox[1], bbox[2], bbox[3])
                        self.uic.size_view.setPixmap(self.convert_cv_qt(size, 600, 450))

                        # 2D center result of object
                        self.center_result[0] = center_x
                        self.center_result[1] = center_y

                        # size, color detection results
                        self.detect_result = [size_result, color_result]

                        # convert 2D center to 3D center
                        self.product_result, self.index_type = self.result_of_product(size_result, color_result)
                        if self.product_result == 'Error':
                            self.pick_place = (0, 0, 0)
                            self.drop_place = (0, 0, 0)
                        else:
                            if not self.isAutoMode:
                                self.pick_place = camera_calib.findPerspective3DCoors(self.center_result)
                            else:
                                self.temp_pick_place = camera_calib.findPerspective3DCoors(self.center_result)

                        # Free up memory of images and pixmap
                        del size
                        del obj
                        del logo
                        del image

                    else:
                        sz_color_processing.uncertain_algorithm.reset_bel()
                        self.uic.logo_view.setPixmap(self.convert_cv_qt(object_processing.waited_capture, 200, 200))
                        self.uic.size_view.setPixmap(self.convert_cv_qt(object_processing.waited_capture, 400, 400))

                        # Free up memory of images and pixmap
                        del logo
                        del image

                    # end_time = time.time()
                    # print("Time of models is:", end_time - start_time, "sec")

                    # Auto mode on
                    if self.isAutoMode:
                        """
                        When robot is available, then send center of robot coordinate:
                        - step 1: if isObj is true, check robot is available ? by read the buffer.
                        - step 2: if yes, check detect result (size, color, logo) ?
                        - step 3: if one of detect result is error, back step 1, else next step.
                        - step 4: convert center x, center y to robot coordinate (pick place).
                        - step 5: validate drop place from dict (type, coordinate), if not find, raise error.
                        - step 6: inverse kinematic pick place and drop place.
                        - step 7: send that points to mcu, set robot unavailable.
                        """

                        if isObject:
                            # read buffer -> validate robot is available?
                            if self.isRobotAvailable:
                                # checking center point (pick place) is in range of conveyor work-place
                                # 18 < 'x_coor' < 24 (centi) and 'y_coor' in working area of model RandomForest
                                if 18 < self.temp_pick_place[0] < 24 and \
                                        rfPoint.LOW_WORKING_Y_AREA < self.temp_pick_place[1] < rfPoint.HIGH_WORKING_Y_AREA:
                                    # check the object is not error
                                    if self.product_result == "Not error":
                                        # predicting center point (pick place) depend on machine learning algorithm
                                        if self.isRunConveyor and self.isApplyRF:
                                            y_new = rfPoint.predict_new_point(self.temp_pick_place[1],
                                                                              rfPoint.AVERAGE_SYS_DELAY_TIME)
                                            self.pick_place = (self.pick_place[0], y_new, self.pick_place[2])
                                        else:
                                            self.pick_place = self.temp_pick_place

                                        # validate the drop place
                                        # index = {'size16': 1, 'size18': 2, 'size20': 3, 'error': 4}
                                        teaching_point = self.type_dict.get(self.index_type)
                                        self.drop_place = tuple(self.coordinate_dict.get(teaching_point))

                                        # inverse kinematic pick place and drop place
                                        inversePoints = self.inverseForAutoMode([self.pick_place, self.drop_place])

                                        # send points to mcu
                                        self.sendProcessMove(inversePoints[0], inversePoints[1])

                                        # set robot unavailable
                                        self.isRobotAvailable = False

            except Exception as e:
                print(e, "-start cam")

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
            self.isCalibSizeModelMode = True
            self.uic.btn_calib_size.setDisabled(False)
            self.uic.btn_test_size.setDisabled(False)
            self.uic.btn_calib_mode.setText("Off Mode")
            self.uic.btn_calib_mode.setStyleSheet("background: rgb(0,255,0);")
            self.uic.btn_start.setDisabled(True)
            self.uic.lb_info.setText("Start calibrating size model\nInsert standard size 16 object")
        else:
            self.isCalibSizeModelMode = False
            self.uic.btn_calib_size.setDisabled(True)
            self.uic.btn_test_size.setDisabled(True)
            self.uic.btn_calib_mode.setText("On Mode")
            self.uic.btn_calib_mode.setStyleSheet("background: rgb(235,235,235);")
            self.uic.btn_start.setDisabled(False)
            self.uic.lb_info.setText("Stop calibrating size model\nRestart camera to detect")

        def display():
            try:
                while self.isCalibSizeModelMode:
                    if self.isOpenCam:
                        image = mycam.get_frames()
                        isObject, _, _, bbox = object_processing.obj_detector_process(image)
                        if isObject:
                            x1, y1, x2, y2 = bbox[0], bbox[1], bbox[2], bbox[3]
                            img_org, img_contour = preprocessing_img(image, (x1, y1, x2, y2))
                            img, _, d1, d2, d3 = find_features(img_org, img_contour)
                            self.uic.size_view.setPixmap(self.convert_cv_qt(img, 600, 450))

                            if self.isSizeCalibModel:
                                self.isSizeCalibModel = False
                                size_calib.convert(d1, d2, d3)
                                sz_color_processing.cf1 = size_calib.cf1
                                sz_color_processing.cf2 = size_calib.cf2
                                sz_color_processing.cf3 = size_calib.cf3
                                self.detect_result[0] = ''

                            if self.isTestCalibSizeModel:
                                self.isTestCalibSizeModel = False
                                result = size_calib.test(d1, d2, d3)
                                self.detect_result[0] = result
                        else:
                            self.uic.size_view.setPixmap(self.convert_cv_qt(image, 600, 450))
                            self.isTestCalibSizeModel = False
                            self.isSizeCalibModel = False

                if not self.isCalibSizeModelMode:
                    self.uic.size_view.clear()

            except Exception as e:
                print(e)

        display_thread = threading.Thread(target=display)
        display_thread.start()

    def is_calib(self):
        self.isSizeCalibModel = True
        self.uic.lb_info.setText("Calibrated")

    def is_test_calib(self):
        self.isTestCalibSizeModel = True
        self.uic.lb_info.setText("Test new calibrated model")

    @staticmethod
    def convert_cv_qt(cv_img, dis_width, dis_height):
        h, w, ch = cv_img.shape
        bytes_per_line = ch * w
        qt_format = QImage(cv_img.data, w, h, bytes_per_line, QImage.Format_BGR888)
        p = qt_format.scaled(dis_width, dis_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def update_results(self):
        """
        Timer interrupt 1s
        """
        # show size result
        if self.detect_result[0] == 'error':
            self.uic.res_size.setStyleSheet("background: rgb(255,0,0);")
        else:
            self.uic.res_size.setStyleSheet("background: rgb(255,255,255);")
        self.uic.res_size.setText(self.detect_result[0])

        # show color result
        if self.detect_result[1] == 'red':
            self.uic.res_color.setStyleSheet("background: rgb(255,255,255);color: rgb(255,0,0);")
        elif self.detect_result[1] == 'yellow':
            self.uic.res_color.setStyleSheet("background: rgb(255,255,255);color: rgb(255,255,0);")
        elif self.detect_result[1] == 'error':
            self.uic.res_color.setStyleSheet("background: rgb(255,0,0);color: rgb(0,0,0);")
        else:
            self.uic.res_color.setStyleSheet("background: rgb(255,255,255);")
        self.uic.res_color.setText(self.detect_result[1])

        # show product result
        if self.product_result == "Error":
            self.uic.result.setStyleSheet("background: rgb(255,0,0);")
        else:
            self.uic.result.setStyleSheet("background: rgb(255,255,255);")
        self.uic.result.setText(self.product_result)

        # show pick-place and drop-place
        self.uic.pick_pl.setText(str(self.pick_place))
        self.uic.drop_pl.setText(str(self.drop_place))

        # device 1: arduino/robot
        if serialCom1.receive_buff[1] == 1:
            if serialCom1.receive_buff[3] == 1:
                self.isRobotAvailable = True
                serialCom1.receive_buff = b'00000'
            elif serialCom1.receive_buff[3] == 0:
                self.isRobotAvailable = False
                serialCom1.receive_buff = b'00000'

        # device 2: stm32/conveyor
        if self.isReadSpeedMode:
            self.buffer = robocod.concatenate(dev="2", cmd=robocod.READSPEED, data=None)
            _ = serialCom2.send_data(self.buffer)
            if serialCom2.receive_buff[1] == 0x32:
                integer = int(serialCom2.receive_buff[2])
                dec = int(serialCom2.receive_buff[3])
                cSpeed = float(integer + dec / 10)
                self.uic.txt_pv_sp.setText(str(cSpeed))
                serialCom2.receive_buff = b'00000'

                self.draw_graph(self.t, float(self.uic.txt_setpoint_sp.toPlainText()), cSpeed)
                self.t += 1

    @staticmethod
    def result_of_product(size, color):
        """
        :param size: size of product
        :param color: color of product
        :return: Result of Product, Index of size type
        """
        size_dict = {'size16': 1, 'size18': 2, 'size20': 3, 'error': 4}
        color_dict = {'red': 0, 'yellow': 1, 'error': 2}
        if size_dict.get(size, None) == 4 or color_dict.get(color, None) == 2:
            return 'Error', 4
        return 'Not error', size_dict.get(size, None)

    # endregion

    # region run auto mode
    def convert_teaching_data(self):
        """
            example:
                {1: "P01"} --- type_dict
                {"P01": [5, 10, 10]} --- coordinate_dict
            :return: True or False if there have teaching data in database
        """
        table = my_database.read_from_database()
        if table:
            for row in table:
                self.type_dict.update({row[1]: row[0]})
                self.coordinate_dict.update({row[0]: [row[2], row[3], row[4]]})
            return True
        return False

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

        # get intermediate point (P00)
        point00 = self.type_dict.get(0)
        coor00 = list(self.coordinate_dict.get(point00))
        coor00 = self.inverseForAutoMode([tuple(coor00)])
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.SET_INTER_POINT, data=[coor00[0][0], coor00[0][1],
                                                                                      coor00[0][2], 0, 0, 0])
        isSend = serialCom1.send_data(self.buffer)
        self.check_sending(isSend)
        if not isSend:
            return

        if not self.isOpenCam:
            self.msg.setIcon(QMessageBox.Warning)
            self.msg.setText("Check your Camera")
            self.msg.setWindowTitle("Error")
            self.msg.setStandardButtons(QMessageBox.Ok)
            self.msg.exec()
            return

        # validate robot is available ? by asking mcu
        # .....send buffer and validate
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.CHECK_STATE, data=None)
        isSend = serialCom1.send_data(self.buffer)
        self.check_sending(isSend)
        if not isSend:
            return

        self.isReadSpeedMode = False

        if isSend:
            if serialCom1.receive_buff[3] == 1:
                self.isRobotAvailable = True
                serialCom1.receive_buff = b'00000'
            elif serialCom1.receive_buff[3] == 0:
                self.isRobotAvailable = False
                serialCom1.receive_buff = b'00000'

        if not self.isRobotAvailable:
            self.msg.setIcon(QMessageBox.Warning)
            self.msg.setText("Move robot to home position")
            self.msg.setWindowTitle("Error")
            self.msg.setStandardButtons(QMessageBox.Ok)
            self.msg.exec()
            return

        # on auto mode
        self.isAutoMode = True

        # setup properties
        self.uic.btn_run.setDisabled(True)
        self.uic.btn_stop.setDisabled(False)
        self.uic.light_stt_2.setStyleSheet("background: rgb(51,255,51); border-radius:35px; "
                                           "border-color: rgb(0, 0, 0); border-width : 1px; border-style:inset;")
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

    def stop(self):
        self.uic.btn_run.setDisabled(False)
        self.uic.btn_stop.setDisabled(True)
        self.uic.btn_reset.setDisabled(False)
        self.uic.light_stt_2.setStyleSheet(
            "background: rgb(255,0,0); border-radius:35px; border-color: rgb(0, 0, 0); "
            "border-width : 1px; border-style:inset;")
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

        # off mode
        self.isAutoMode = False

    def reset(self):
        self.uic.btn_run.setDisabled(False)
        self.uic.btn_stop.setDisabled(True)
        self.uic.btn_reset.setDisabled(True)
        self.uic.light_stt_2.setStyleSheet(
            "background: rgb(255,255,0); border-radius:35px; border-color: rgb(0, 0, 0); "
            "border-width : 1px; border-style:inset;")

        # robot go home
        self.home()

    # endregion

    # region camera calibration
    def list_opt_calib(self):
        it = ["Default Calibration", "Custom Calibration"]
        self.uic.cb_opt_calib_cam.addItems(it)

    def changed_opt(self):
        if self.uic.cb_opt_calib_cam.currentText() == "Default Calibration":
            self.uic.lb_opt_calib_cam.setText("Use pretrained model")
            self.uic.lb_opt_calib_cam.setStyleSheet("color: rgb(0,0,255);")
            self.uic.btn_calib_mode_cam.setDisabled(True)
            self.uic.btn_scrshot.setDisabled(True)
            self.uic.btn_calib_cam.setDisabled(True)
            self.uic.btn_save_model.setDisabled(True)
            self.uic.btn_apply_calib_model.setDisabled(False)
            self.uic.btn_on_cam_test.setDisabled(False)
            self.uic.cb_calib_model.clear()
            self.list_calibrated_model()
        else:
            self.uic.lb_opt_calib_cam.setText("Create new calib model")
            self.uic.lb_opt_calib_cam.setStyleSheet("color: rgb(255,0,0);")
            self.uic.btn_calib_mode_cam.setDisabled(False)
            self.uic.btn_apply_calib_model.setDisabled(True)
            self.uic.btn_on_cam_test.setDisabled(True)

    def calib_cam_model(self):
        if self.uic.btn_calib_mode_cam.text() == "On Mode":
            if self.isOpenCam or self.isTestCamCalibModel:
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText("Another mode is online")
                self.msg.setWindowTitle("Error")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.exec()
                return

            # check images file in 'checkerboard' folder
            ret, num_files = camera_calib.check_file_in_images_folder()
            if ret:
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText(f"Images folder has existed {num_files} files, delete all or cancel?")
                self.msg.setWindowTitle("Warning")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.addButton(QMessageBox.Cancel)
                reply = self.msg.exec()
                if reply == QMessageBox.Ok:
                    camera_calib.delete_files_images_folder()

            if mycam.connect_cam():
                self.uic.btn_calib_mode_cam.setText("Off Mode")
                self.uic.btn_scrshot.setDisabled(False)
                self.uic.btn_calib_cam.setDisabled(False)
                self.uic.cb_opt_calib_cam.setDisabled(True)
                self.uic.btn_calib_mode_cam.setStyleSheet("background: rgb(0,255,0);")
                _, _ = camera_calib.check_dir()
                self.uic.txt_info_calib_cam.setText("Start the calibration process")
                self.isCalibCamMode = True
        else:
            self.isCalibCamMode = False
            if mycam.disconnect_cam():
                self.uic.btn_calib_mode_cam.setText("On Mode")
                self.uic.btn_scrshot.setDisabled(True)
                self.uic.btn_calib_cam.setDisabled(True)
                self.uic.btn_save_model.setDisabled(True)
                self.uic.cb_opt_calib_cam.setDisabled(False)
                self.uic.btn_calib_mode_cam.setStyleSheet("background: rgb(235,235,235);")
                self.uic.calib_cam_view.clear()
                self.uic.txt_info_calib_cam.setText("Stop the calibration process")

        def display():
            try:
                while self.isCalibCamMode:
                    if self.isCalibCam:
                        self.uic.calib_cam_view.clear()
                        self.isCalibCam = False
                        break

                    image = mycam.get_frames()
                    isCheckerBoard, show_img, save_img = camera_calib.detect_checker_board(image)
                    self.uic.calib_cam_view.setPixmap(self.convert_cv_qt(show_img, 720, 530))

                    if self.isTakeShot:
                        self.isTakeShot = False
                        saved_info = camera_calib.save_images(isCheckerBoard, save_img)
                        self.uic.txt_info_calib_cam.setText(saved_info)

                    if not self.isCalibCamMode:
                        self.uic.calib_cam_view.clear()
                        break

            except Exception as e:
                print(e, "-calib cam mode")
                pass

        display_thread = threading.Thread(target=display)
        display_thread.start()

    def take_screen_shot(self):
        self.isTakeShot = True

    def is_calib_cam(self):
        camera_calib.counter_shot_img = 1
        self.isCalibCam = True
        self.uic.btn_save_model.setDisabled(False)
        self.uic.btn_scrshot.setDisabled(True)
        self.uic.txt_info_calib_cam.setText(f"Use data at: '../{camera_calib.img_dir_path}' to calibrate")
        info_calib = camera_calib.calibrate_camera()
        self.uic.txt_info_calib_cam.setText(info_calib)

    def is_save_cam_calib_model(self):
        if not self.uic.txt_name_calib_model.toPlainText():
            self.uic.txt_info_calib_cam.setStyleSheet("background: rgb(255,0,0);")
            self.uic.txt_info_calib_cam.setText("Insert the calibrate model name to continue")
        else:
            self.uic.txt_info_calib_cam.setStyleSheet("background: transparent;")
            model_name = self.uic.txt_name_calib_model.toPlainText()
            self.uic.txt_info_calib_cam.setText(f"Save model into: '../{camera_calib.calib_dir_path}/{model_name}'")
            self.uic.btn_save_model.setDisabled(True)
            info_save, _ = camera_calib.save_model(model_name)
            self.uic.txt_info_calib_cam.setText(info_save)

    def list_calibrated_model(self):
        files_name = camera_calib.list_model_names()
        self.uic.cb_calib_model.addItems(files_name)

    def apply_calib_cam_model(self):
        if not self.isCalibSizeModelMode:
            info_apply = camera_calib.load_model(self.uic.cb_calib_model.currentText())
            self.uic.txt_info_calib_cam.setText(info_apply)
            self.uic.txt_info_calib_cam.setStyleSheet("background: transparent;")
            self.uic.txt_calib_model.setText(f"Change to '{self.uic.cb_calib_model.currentText()}' model")
            self.uic.lb_info.setText(f"Change to '{self.uic.cb_calib_model.currentText()}' model")
        else:
            self.uic.txt_info_calib_cam.setStyleSheet("background: rgb(255,0,0);")
            self.uic.txt_info_calib_cam.setText("Camera calibration mode is online")

    def test_calib_cam_model(self):
        if self.uic.btn_on_cam_test.text() == "ON":
            if self.isCalibCamMode or self.isOpenCam:
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText("Another mode is online")
                self.msg.setWindowTitle("Error")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.exec()
                return
            if camera_calib.Z == 0:
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText("Calibration model is not found")
                self.msg.setWindowTitle("Error")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.exec()
                return
            if mycam.connect_cam():
                self.uic.btn_on_cam_test.setText("OFF")
                self.uic.btn_on_cam_test.setStyleSheet("background: rgb(0,255,0);")
                self.uic.txt_info_calib_cam.setText("Insert the checkerboard to test model")
                self.isTestCamCalibModel = True
                self.uic.btn_apply_calib_model.setDisabled(True)
        else:
            self.isTestCamCalibModel = False
            if mycam.disconnect_cam():
                self.uic.btn_on_cam_test.setText("ON")
                self.uic.btn_on_cam_test.setStyleSheet("background: rgb(235,235,235);")
                self.uic.calib_cam_view.clear()
                self.uic.btn_apply_calib_model.setDisabled(False)

        def display():
            while self.isTestCamCalibModel:
                try:
                    while self.isTestCamCalibModel:
                        image = mycam.get_frames()
                        show_img = camera_calib.get_undistorted_images(image)
                        show_img = camera_calib.test_model_use_checkerboard(show_img, int(self.set_num_points()))
                        self.uic.calib_cam_view.setPixmap(self.convert_cv_qt(show_img, 720, 540))

                        if not self.isTestCamCalibModel:
                            self.uic.calib_cam_view.clear()
                            break

                except Exception as e:
                    print(e, "-test calib cam mode")

        display_thread = threading.Thread(target=display)
        display_thread.start()

    def set_num_points(self):
        return self.uic.slide_points_test.value()

    # endregion

    # region conveyor speed
    def draw_graph(self, x, y_sp, y_pv):
        self.uic.graphicsView.scene().clear()
        self.x.append(x)
        self.y_pv.append(y_pv)
        self.y_sp.append(y_sp)
        plt.plot(self.x, self.y_pv, 'r')
        plt.plot(self.x, self.y_sp, 'b')
        self.fig.savefig('data/graph.png')
        pixmap = QPixmap('data/graph.png')
        item = QGraphicsPixmapItem(pixmap)
        self.uic.graphicsView.scene().addItem(item)
        self.uic.graphicsView.show()

    def list_low_area(self):
        it = ["-10", "-8", "-6", "-4", "-2", "0"]
        self.uic.cb_low_area.addItems(it)
        self.uic.cb_low_area.setCurrentIndex(3)

    def list_high_area(self):
        it = ["0", "2", "4", "6", "8", "10"]
        self.uic.cb_high_area.addItems(it)
        self.uic.cb_high_area.setCurrentIndex(2)

    def apply_rf_model(self):
        if self.uic.btn_apply_rf.text() == "Apply":
            rfPoint.CONVEYOR_VELOCITY = int(self.uic.txt_convey_sp_rf.toPlainText())
            rfPoint.LOW_WORKING_Y_AREA = int(self.uic.cb_low_area.currentText())
            rfPoint.HIGH_WORKING_Y_AREA = int(self.uic.cb_high_area.currentText())
            rfPoint.create_new_model()
            self.uic.txt_info_convey_rf.setText("Applied!")
            self.uic.txt_info_convey_rf.setStyleSheet("background: rgb(0,255,0);")
            self.uic.btn_apply_rf.setText("Cancel")
            self.isReadSpeedMode = False
            self.x.clear()
            self.y_pv.clear()
            self.y_sp.clear()
            self.isApplyRF = True
        else:
            self.uic.txt_info_convey_rf.clear()
            self.uic.txt_info_convey_rf.setStyleSheet("background: transparent")
            self.uic.btn_apply_rf.setText("Apply")
            self.isApplyRF = False

    def sp_convey_changed(self):
        self.uic.txt_convey_sp_rf.setText(self.uic.txt_setpoint_sp.toPlainText())

    def run_conveyor(self):
        if self.uic.btn_run_convey.text() == "Start Conveyor":
            if not 1 <= float(self.uic.txt_setpoint_sp.toPlainText()) <= 5:
                self.msg.setIcon(QMessageBox.Warning)
                self.msg.setText("Set-point is out of range [1,5] cm")
                self.msg.setWindowTitle("Error")
                self.msg.setStandardButtons(QMessageBox.Ok)
                self.msg.exec()
                return

            self.buffer = robocod.concatenate(dev="2", cmd=robocod.RUNCONVEY,
                                              data=[self.uic.txt_setpoint_sp.toPlainText(),
                                                    0, 0, 0, 0, 0])
            isSend = serialCom2.send_data(self.buffer)
            self.check_sending(isSend)
            if not isSend:
                return

            self.uic.btn_run_convey.setText("Stop Conveyor")
            self.isRunConveyor = True
            self.isReadSpeedMode = True
            if not self.timer.isActive():
                self.timer.start()
        else:
            self.buffer = robocod.concatenate(dev="2", cmd=robocod.STOPCONVEY, data=None)
            isSend = serialCom2.send_data(self.buffer)
            self.check_sending(isSend)
            if not isSend:
                return

            self.uic.btn_run_convey.setText("Start Conveyor")
            self.isRunConveyor = False
            self.isReadSpeedMode = False

    # endregion


if __name__ == '__main__':
    # load the size, color model
    sz_color_processing = SizeAndColorProcess()

    # load yolov5 model
    object_processing = ObjectProcess()

    app = QApplication(sys.argv)
    self = MainWindow()
    self.Control_UI.show()
    sys.exit(app.exec_())
