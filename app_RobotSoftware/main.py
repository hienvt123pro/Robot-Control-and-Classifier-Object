from PySide2.QtWidgets import QApplication, QMainWindow, QMessageBox, QGraphicsPixmapItem, QGraphicsScene, \
    QTableWidgetItem, QDialog
from PySide2.QtGui import QIcon, QImage, QPixmap, QKeySequence
from PySide2.QtCore import QSize, Qt
from PySide2 import QtCore
from GUI import gui, gui_speed, gui_size, gui_camera
from communicate import serialCom1, receive_thread_1, serialCom2, receive_thread_2
from camera import mycam
from detect import SizeAndColorProcess, ObjectProcess, draw_working_area, save_image, read_image, LogoProcess
from find_feature import ffeats
from preprocessing import preprocessing_obj
from database import robot_database, product_database
from calibration import size_calib, camera_calib
from robot_code import robocod
from predict_point import rfPoint
import math
import sys
import threading
import matplotlib.pyplot as plt
import datetime
import os
import time


class Lock:
    def __init__(self):
        self.isOpenCam = False
        self.isCalibSizeModelMode = False
        self.isCalibCamMode = False
        self.isTestCamCalibModel = False
        self.isRunConveyor = False
        self.isReadSpeedMode = False
        self.isApplyRF = False


class MainWindow:
    def __init__(self):
        self.Control_UI = QMainWindow()
        self.uic = gui.Ui_MainWindow()
        self.uic.setupUi(self.Control_UI)
        self.msg = QMessageBox()
        self.buffer = "0,0,0,0,0,0,0,0,0,0,0"
        self.isPortsCnt = False
        self.detect_result = ['', '', '']
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
        self.isPreviousRobotAvailable = False
        self.isPreviewMode = True
        self.detect_time = 0

        # Timer Interrupt
        self.timer = QtCore.QTimer()
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.update_results)

        # Working Area
        self.isShowArea = False
        self.rectangleArea = []

        # Product Data
        self.current_page = 1
        self.row_position = 0
        self.product_name = "SP"
        self.product_index = 0
        self.note = ""
        self.refresh_sheet()
        self.image_error_product = object_processing.waited_capture
        product_database.check_dir()
        product_database.delete_img_if_empty_database()

        # Vector
        self.vectorConveyor = [0, 0]

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
        self.uic.btn_savepoint.setDisabled(True)
        self.uic.btn_stop.setDisabled(True)
        self.uic.btn_reset.setDisabled(True)
        self.uic.cb_size.setDisabled(True)
        self.uic.cb_color.setDisabled(True)
        self.uic.cb_com.setToolTip("Choose COM to Robot")
        self.uic.cb_com.setToolTipDuration(10000)
        self.uic.cb_com_2.setToolTip("Choose COM to Conveyor")
        self.uic.cb_com_2.setToolTipDuration(10000)
        self.uic.btn_Home.setToolTip("Robot go to home position (H)")
        self.uic.btn_Home.setToolTipDuration(10000)
        self.uic.btn_calibHome.setToolTip("Calibrate and move robot to home (Ctrl+H)")
        self.uic.btn_Home.setToolTipDuration(10000)
        self.uic.rbutton_new.setToolTip("Create new teaching point")
        self.uic.rbutton_new.setToolTipDuration(10000)
        self.uic.btn_reset_point.setToolTip("Clear all teaching point")
        self.uic.btn_reset_point.setToolTipDuration(10000)
        self.uic.btn_savepoint.setToolTip("Save the point to database")
        self.uic.btn_savepoint.setToolTipDuration(10000)
        self.uic.btn_renew_ports.setToolTip("Refresh COM ports")
        self.uic.btn_renew_ports.setToolTipDuration(10000)
        self.uic.btn_refresh_table.setToolTip("Refresh product data")
        self.uic.btn_refresh_table.setToolTipDuration(10000)
        self.uic.btn_export_table.setToolTip("Export data to Excel")
        self.uic.btn_export_table.setToolTipDuration(10000)
        self.uic.btn_run.setToolTip("Start robot and stop preview mode")
        self.uic.btn_run.setToolTipDuration(10000)
        self.uic.btn_stop.setToolTip("Stop robot and restart preview mode")
        self.uic.btn_stop.setToolTipDuration(10000)
        self.uic.btn_start.setToolTip("Start the preview screen")
        self.uic.btn_start.setToolTipDuration(10000)
        self.uic.btn_move.setToolTip("M")
        self.uic.btn_move.setToolTipDuration(10000)
        self.uic.btn_end_effector.setToolTip("F")
        self.uic.btn_end_effector.setToolTipDuration(10000)

        # endregion

        # region events
        # event open dialog
        self.uic.actionSize_model.triggered.connect(self.show_size_dialog)
        self.uic.actionCamera.triggered.connect(self.show_camera_dialog)
        self.uic.actionSpeed_control.triggered.connect(self.show_speed_dialog)
        self.uic.actionExit.triggered.connect(self.exit)

        # event menubar
        self.uic.actionWorking_Area.triggered.connect(self.show_working_area)
        self.uic.actionSizeFeatures.triggered.connect(self.show_size_feat)
        self.uic.actionLogoFeatures.triggered.connect(self.show_logo_feat)
        self.uic.actionColorFeatures.triggered.connect(self.show_color_feat)

        # event tableWidget
        self.uic.tableWidget.cellClicked.connect(self.handle_cell_clicked)

        # event text edit
        self.uic.txt_mX.returnPressed.connect(self.handle_enter_pressed)
        self.uic.txt_mY.returnPressed.connect(self.handle_enter_pressed)
        self.uic.txt_mZ.returnPressed.connect(self.handle_enter_pressed)

        # event check edit
        self.uic.cbox_error.clicked.connect(self.filter_error)
        self.uic.cbox_not_error.clicked.connect(self.filter_not_error)

        # event combo box
        self.uic.cb_com.activated.connect(self.list_ports1())
        self.uic.cb_com_2.activated.connect(self.list_ports2())
        self.uic.cb_baudrate.activated.connect(self.list_baudrate())
        self.uic.cb_point.activated.connect(self.list_points())
        self.uic.cb_point.currentTextChanged.connect(self.changed_point)
        self.uic.cb_sizetype.activated.connect(self.list_sizetype())

        # event push button
        self.uic.btn_cnt.clicked.connect(self.connection)
        self.uic.btn_renew_ports.clicked.connect(self.renew_ports)
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
        self.uic.rbutton_new.clicked.connect(self.teaching_mode)
        self.uic.btn_savepoint.clicked.connect(self.save_point)
        self.uic.btn_reset_point.clicked.connect(self.reset_point)
        self.uic.btn_run.clicked.connect(self.run)
        self.uic.btn_stop.clicked.connect(self.stop)
        self.uic.btn_reset.clicked.connect(self.reset)
        self.uic.btn_end_effector.clicked.connect(self.end_effector)
        self.uic.btn_next_page.clicked.connect(self.change_page)
        self.uic.btn_refresh_table.clicked.connect(self.refresh_sheet)
        self.uic.btn_export_table.clicked.connect(self.export_sheet_to_excel)
        self.uic.btn_filter.clicked.connect(self.detail_filter)

        # event slide
        self.uic.slide_speed.valueChanged.connect(self.set_speed)
        self.uic.slide_acc.valueChanged.connect(self.set_accelerate)
        self.uic.slide_deg.valueChanged.connect(self.set_degree)
        self.uic.slide_cm.valueChanged.connect(self.set_centimeters)

        # endregion

    def showMessageBox(self, type_box: QMessageBox, text: str, title: str, standard_button: QMessageBox):
        self.msg.setIcon(type_box)
        self.msg.setText(text)
        self.msg.setWindowTitle(title)
        self.msg.setStandardButtons(standard_button)
        self.msg.exec()

    # region port connection
    def list_ports1(self):
        self.uic.cb_com.clear()
        ports1 = serialCom1.list_ports_name()
        if not ports1:
            self.uic.cb_com.addItem("not found")
            return
        for port in ports1:
            p = str(port)
            self.uic.cb_com.addItem(p[:5])

    def list_ports2(self):
        self.uic.cb_com_2.clear()
        ports2 = serialCom2.list_ports_name()
        if not ports2:
            self.uic.cb_com_2.addItem("not found")
            return
        for port in ports2:
            p = str(port)
            self.uic.cb_com_2.addItem(p[:5])

    def renew_ports(self):
        self.list_ports1()
        self.list_ports2()

    def list_baudrate(self):
        self.uic.cb_baudrate.addItem("115200")

    def connection(self):
        if self.uic.btn_cnt.text() == "Connect":
            port_name_1 = self.uic.cb_com.currentText()
            port_name_2 = self.uic.cb_com_2.currentText()
            if port_name_1 == port_name_2:
                self.showMessageBox(QMessageBox.Warning, "Duplicate COM Ports at 2 device\nor ports are not available",
                                    "Error", QMessageBox.Ok)
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
                self.showMessageBox(QMessageBox.Warning, "Check your COM Port", "Error", QMessageBox.Ok)
        else:
            serialCom1.disconnect()
            serialCom2.disconnect()
            serialCom1.isKillThread = True
            serialCom2.isKillThread = True
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
            self.uic.txt_info_teaching.setText(robot_database.get_table_comment() + "\n"
                                               + self.re_text(robot_database.read_from_database()))
        else:
            self.uic.btn_savepoint.setDisabled(True)

    @staticmethod
    def re_text(dic):
        text = "|  point  |  size  |  x  |  y  |  z  |"
        for unit in dic:
            text = text + '\n' + str(unit)
        return text

    def reset_point(self):
        self.msg.setIcon(QMessageBox.Warning)
        self.msg.setText(f"Do you want to reset all teaching points?")
        self.msg.setWindowTitle("Warning")
        self.msg.setStandardButtons(QMessageBox.Ok)
        self.msg.addButton(QMessageBox.Cancel)
        reply = self.msg.exec()
        if reply == QMessageBox.Ok:
            robot_database.delete_on_database()
            self.uic.txt_info_teaching.setText("\n Clear all data!")
        else:
            self.uic.txt_info_teaching.setText("\n Abort resetting!")

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
                robot_database.save_into_database(_p, _size, _x, _y, _z)
            except Exception as e:
                print(e, "-save point")
                robot_database.update_into_database(_p, _size, _x, _y, _z)
            self.uic.txt_info_teaching.setText(self.re_text(robot_database.read_from_database()))
        else:
            self.showMessageBox(QMessageBox.Warning, "Check your teaching point", "Error", QMessageBox.Ok)

    # endregion

    # region calibrate robot
    def check_sending(self, state):
        if not state:
            self.showMessageBox(QMessageBox.Warning, "Check your COM Port", "Error", QMessageBox.Ok)

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
        else:
            self.showMessageBox(QMessageBox.Warning, "Check your COM Port", "Error", QMessageBox.Ok)

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
            self.uic.btn_end_effector.setShortcut(QKeySequence("F"))
        else:
            self.buffer = robocod.concatenate(dev="1", cmd=robocod.EFFECTOR, data=None)
            isSend = serialCom1.send_data(self.buffer)
            self.check_sending(isSend)
            if not isSend:
                return
            self.uic.btn_end_effector.setText("On")
            self.uic.btn_end_effector.setShortcut(QKeySequence("F"))

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
    def handle_enter_pressed(self):
        self.uic.txt_mX.clearFocus()
        self.uic.txt_mY.clearFocus()
        self.uic.txt_mZ.clearFocus()

    def move(self):
        try:
            if not self.uic.txt_mX.text() or not self.uic.txt_mY.text() or not self.uic.txt_mZ.text():
                return
            self.Px = float(self.uic.txt_mX.text())
            self.Py = float(self.uic.txt_mY.text())
            self.Pz = float(self.uic.txt_mZ.text())
            if self.Px < 5 and self.Py < 5:
                return
            if self.Px > 28 or self.Py > 28:
                return
            if self.Pz < -5 or self.Pz > 22:
                return
            self.uic.txt_x.setText(str(self.Px))
            self.uic.txt_y.setText(str(self.Py))
            self.uic.txt_z.setText(str(self.Pz))
            self.inverseKinematics(px=self.Px, py=self.Py, pz=self.Pz)
        except ValueError:
            self.showMessageBox(QMessageBox.Warning, "Value Error", "Error", QMessageBox.Ok)

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
            if lock.isCalibSizeModelMode:
                self.showMessageBox(QMessageBox.Warning, "Size calibration mode is online", "Error", QMessageBox.Ok)
                return
            if lock.isCalibCamMode or lock.isTestCamCalibModel:
                self.showMessageBox(QMessageBox.Warning, "Camera calibration mode is online", "Error", QMessageBox.Ok)
                return
            if camera_calib.Z == 0:
                self.showMessageBox(QMessageBox.Warning, "Calibration model is not found\nGo to 'Tools/Calibration/Camera'",
                                    "Error", QMessageBox.Ok)
                return
            if mycam.connect_cam():
                self.uic.btn_start.setIcon(icon2)
                self.uic.btn_start.setIconSize(QSize(40, 40))
                self.uic.btn_start.setText(" Stop Cam")
                lock.isOpenCam = True
                self.uic.lb_info.setText("Start detection models")
                self.uic.res_size.setStyleSheet("background: rgb(255,255,255);")
                self.timer.start()
        else:
            lock.isOpenCam = False
            if mycam.disconnect_cam():
                self.uic.btn_start.setIcon(icon1)
                self.uic.btn_start.setIconSize(QSize(40, 40))
                self.uic.btn_start.setText(" Start Cam")
                self.uic.obj_view.clear()
                self.uic.size_view.clear()
                self.uic.logo_view.clear()
                self.uic.lb_info.setText("Terminate detection models")
                self.timer.stop()

        self.validate_direct_conveyor()

        def display():
            try:
                while lock.isOpenCam:
                    # Detect frames
                    start_time = time.time()
                    image = mycam.get_frames()

                    # YOLOv5 detector
                    isObject, obj_view, logo_view, bbox, cenLogo = object_processing.obj_detector_process(image)

                    # save the image if it's an error product
                    self.image_error_product = image[bbox[1]:bbox[3], bbox[0]:bbox[2]]

                    # show the working area of robot and decision area of models
                    if self.isShowArea:
                        obj_view = draw_working_area(obj_view, self.rectangleArea)

                    self.uic.obj_view.setPixmap(self.convert_cv_qt(obj_view, 600, 450))
                    if isObject:
                        ffeats.center_logo = cenLogo

                        # size, color detector
                        size_view, size_result, color_result, center_x, center_y = sz_color_processing.detect_size_color(
                            image, bbox[0], bbox[1], bbox[2], bbox[3])
                        self.uic.size_view.setPixmap(self.convert_cv_qt(size_view, 600, 450))
                        if size_result == "":
                            continue

                        # logo detector
                        logo_result = logo_processing.logo_detector(logo=logo_view, size_obj=size_result,
                                                                    color_obj=color_result, vector_conveyor=self.vectorConveyor)
                        self.uic.logo_view.setPixmap(self.convert_cv_qt(logo_view, 200, 200))

                        if self.isPreviewMode or self.isRobotAvailable:
                            # 2D center result of object
                            self.center_result[0] = center_x
                            self.center_result[1] = center_y

                            # size, color, logo detection results
                            self.detect_result = [size_result, color_result, logo_result]
                            self.product_result, self.index_type = self.result_of_product(size_result,
                                                                                          color_result, logo_result)

                            # convert 2D center to 3D center
                            if not self.isAutoMode:
                                self.pick_place = camera_calib.findPerspective3DCoors(self.center_result)
                            else:
                                self.temp_pick_place = camera_calib.findPerspective3DCoors(self.center_result)

                        # Free up memory of images and pixmap
                        del size_view
                        del obj_view
                        del logo_view
                        del image
                    else:
                        sz_color_processing.uncertain_algorithm.reset_bel()
                        self.uic.logo_view.setPixmap(self.convert_cv_qt(object_processing.waited_capture, 200, 200))
                        self.uic.size_view.setPixmap(self.convert_cv_qt(object_processing.waited_capture, 400, 400))

                        # Free up memory of images and pixmap
                        del logo_view
                        del image

                    end_time = time.time()
                    self.detect_time = end_time - start_time

                    # Auto mode on
                    if self.isAutoMode:
                        """
                        When robot is available, then send center of robot coordinate:
                        - step 1: if isObj is true, check robot is available ? by read the buffer.
                        - step 2: if yes, check detect result (size, color, logo) ?
                        - step 3: if one of detect result is error, send reject process command to mcu, set robot unavailable
                        and back to step 1, else next step.
                        - step 4: convert center x, center y to robot coordinate (pick place).
                        - step 5: validate drop place from dict (type, coordinate), if not find, raise error.
                        - step 6: inverse kinematic pick place and drop place.
                        - step 7: send that points to mcu, set robot unavailable.
                        """
                        if isObject:
                            # read buffer -> validate robot is available? then check product is not error or error
                            if self.isRobotAvailable:
                                # check center point (pick place) is in range of conveyor working-place
                                # 16 < 'x_coor' < 24 (cen) and 'y_coor' in working area of model RandomForest
                                if 16 < self.temp_pick_place[0] < 24 and \
                                        rfPoint.LOW_WORKING_Y_AREA < self.temp_pick_place[1] < rfPoint.HIGH_WORKING_Y_AREA:
                                    # check the product is not error
                                    if self.product_result == "Not Error":
                                        # predicting center point (pick place) depend on machine learning algorithm
                                        if lock.isRunConveyor and lock.isApplyRF:
                                            rfPoint.SYS_DELAY_TIME = rfPoint.T_ROBOT + round(self.detect_time, 4) \
                                                                     + rfPoint.T_RF
                                            y_new = rfPoint.predict_new_point(self.temp_pick_place[1],
                                                                              rfPoint.SYS_DELAY_TIME)
                                            # y_new = round(self.temp_pick_place[1] + (1.8 + round(self.detect_time, 4))
                                            #               * rfPoint.CONVEYOR_VELOCITY, 2)
                                            self.pick_place = (self.temp_pick_place[0], y_new, self.temp_pick_place[2])
                                        else:
                                            self.pick_place = self.temp_pick_place

                                        # validate the drop place
                                        # index is {'size30': 1, 'size31': 2, 'size32': 3, 'error': 4}
                                        teaching_point = self.type_dict.get(self.index_type)
                                        self.drop_place = tuple(self.coordinate_dict.get(teaching_point))

                                        # inverse kinematic pick place and drop place
                                        inversePoints = self.inverseForAutoMode([self.pick_place, self.drop_place])

                                        # send points to mcu
                                        self.sendProcessMove(inversePoints[0], inversePoints[1])

                                        # set robot unavailable
                                        self.isRobotAvailable = False

                                        # edit data
                                        self.product_index += 1
                                        self.note = "Standards"

                                    # check product is error, but robot does not pick this error product
                                    if self.product_result == "Error":
                                        # conduct the act of rejecting to pick up the object (delay robot in time of
                                        # working area / conveyor_velocity)
                                        self.pick_place = self.temp_pick_place
                                        self.drop_place = (0, 0, 0)
                                        self.sendProcessMove((0, 0, 0),
                                                             (rfPoint.HIGH_WORKING_Y_AREA - rfPoint.LOW_WORKING_Y_AREA,
                                                              rfPoint.CONVEYOR_VELOCITY, 0))
                                        self.isRobotAvailable = False

                                        # edit data
                                        self.product_index += 1
                                        sz, color, logo = '', '', ''
                                        if self.detect_result[0] == "error":
                                            self.buffer = robocod.concatenate(dev="2", cmd=robocod.ERROR_SIZE, data=None)
                                            isSend = serialCom2.send_data(self.buffer)
                                            self.check_sending(isSend)
                                            sz = "Size Error"
                                        if self.detect_result[1] == "error":
                                            color = "Color Error"
                                        if self.detect_result[2] == "location error":
                                            logo = "Logo Location Error"
                                        elif self.detect_result[2] == "direction error":
                                            logo = "Logo Direction Error"
                                        elif self.detect_result[2] == "ink error":
                                            logo = "Logo Ink Error"
                                        errors = [s for s in [sz, color, logo] if s != '']
                                        self.note = '/'.join(errors) if len(errors) > 0 else ''

                                        # save error product image
                                        save_image(f"error_product/SP{self.product_index:04d}_"
                                                   f"{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.png",
                                                   self.image_error_product)

            except Exception as e:
                print(e, "-start cam")

        display_thread = threading.Thread(target=display)
        display_thread.start()

    def validate_direct_conveyor(self):
        p3D_0 = [24, 0, 0]
        p3D_1 = [24, 4, 0]
        p2D_0 = camera_calib.findImage2DCoors(p3D_0)
        p2D_1 = camera_calib.findImage2DCoors(p3D_1)
        self.vectorConveyor = [p2D_1[0] - p2D_0[0], p2D_1[1] - p2D_0[1]]

    def show_working_area(self):
        if not self.isShowArea:
            if camera_calib.Z == 0:
                self.showMessageBox(QMessageBox.Warning,
                                    "Calibration model is not found\nGo to 'Tools/Calibration/Camera'",
                                    "Error", QMessageBox.Ok)
                self.uic.actionWorking_Area.setChecked(False)
                return
            world_area = [(24, rfPoint.HIGH_WORKING_Y_AREA, 0), (24, rfPoint.LOW_WORKING_Y_AREA, 0),
                          (16, rfPoint.LOW_WORKING_Y_AREA, 0), (16, rfPoint.HIGH_WORKING_Y_AREA, 0)]
            self.rectangleArea.clear()
            for coor in world_area:
                self.rectangleArea.append(camera_calib.findImage2DCoors(list(coor)))
            self.isShowArea = True
            self.uic.actionWorking_Area.setIconVisibleInMenu(True)
        else:
            self.isShowArea = False
            self.uic.actionWorking_Area.setIconVisibleInMenu(False)

    def show_size_feat(self):
        if ffeats.isShowSizeFeats:
            ffeats.isShowSizeFeats = False
            self.uic.actionSizeFeatures.setIconVisibleInMenu(False)
        else:
            ffeats.isShowSizeFeats = True
            self.uic.actionSizeFeatures.setIconVisibleInMenu(True)

    def show_logo_feat(self):
        if ffeats.isShowLogoFeats:
            ffeats.isShowLogoFeats = False
            self.uic.actionLogoFeatures.setIconVisibleInMenu(False)
        else:
            ffeats.isShowLogoFeats = True
            self.uic.actionLogoFeatures.setIconVisibleInMenu(True)

    def show_color_feat(self):
        if ffeats.isShowColorFeats:
            ffeats.isShowColorFeats = False
            self.uic.actionColorFeatures.setIconVisibleInMenu(False)
        else:
            ffeats.isShowColorFeats = True
            self.uic.actionColorFeatures.setIconVisibleInMenu(True)

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
        # show detected time
        self.uic.lb_detect_time.setText(f"Time: {round(self.detect_time, 5)}s")

        # show size result
        if self.detect_result[0] == 'error':
            self.uic.res_size.setStyleSheet("background: rgb(255,0,0);")
        else:
            self.uic.res_size.setStyleSheet("background: rgb(255,255,255);")
        self.uic.res_size.setText(self.detect_result[0])

        # show color result
        if self.detect_result[1] == 'Red':
            self.uic.res_color.setStyleSheet("background: rgb(255,255,255);color: rgb(255,0,0);")
        elif self.detect_result[1] == 'Yellow':
            self.uic.res_color.setStyleSheet("background: rgb(255,255,0);color: rgb(0,0,0);")
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

        # capture edge down when pick object, the purpose is saving info of product to database
        if self.isPreviousRobotAvailable and not self.isRobotAvailable:
            if lock.isRunConveyor:
                # show data on sheet
                self.add_row_data([datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S"),
                                   f"{self.product_name}{self.product_index:04d}",
                                   self.product_result, self.detect_result[0],
                                   self.detect_result[1], self.note])

                # save data into database
                product_database.save_into_database(datetime.datetime.now().strftime("%d/%m/%Y %H:%M:%S"),
                                                    f"{self.product_name}{self.product_index:04d}",
                                                    self.product_result, self.detect_result[0],
                                                    self.detect_result[1], self.note)

        # device 1: arduino/robot
        if serialCom1.receive_buff[1] == 1:
            if serialCom1.receive_buff[3] == 1:
                self.isRobotAvailable = True
                serialCom1.receive_buff = b'00000'
            elif serialCom1.receive_buff[3] == 0:
                self.isRobotAvailable = False
                serialCom1.receive_buff = b'00000'

        self.isPreviousRobotAvailable = self.isRobotAvailable

    @staticmethod
    def result_of_product(size, color, logo):
        """
        :param size: size of product
        :param color: color of product
        :param logo: logo of product
        :return: Result of Product, Index of size type
        """
        size_dict = {'30': 1, '31': 2, '32': 3, 'error': 4}
        color_dict = {'Red': 0, 'Yellow': 1, 'error': 2}
        logo_dict = {'not error': 0, 'location error': 1, 'direction error': 2, 'ink error': 3}
        if size_dict.get(size, None) == 4 or color_dict.get(color, None) == 2 or logo_dict.get(logo, None) == 1\
                or logo_dict.get(logo, None) == 2 or logo_dict.get(logo, None) == 3:
            return 'Error', 4
        return 'Not Error', size_dict.get(size, None)

    # endregion

    # region run auto mode
    def convert_teaching_data(self):
        """
            example:
                {1: "P01"} --- type_dict
                {"P01": [5, 10, 10]} --- coordinate_dict
            :return: True or False if there have teaching data in database
        """
        table = robot_database.read_from_database()
        if table:
            for row in table:
                self.type_dict.update({row[1]: row[0]})
                self.coordinate_dict.update({row[0]: [row[2], row[3], row[4]]})
            return True
        return False

    def run(self):
        # read teaching point from database, if not find data, return false mode
        if not self.convert_teaching_data():
            self.showMessageBox(QMessageBox.Warning, "Check your teaching point", "Error", QMessageBox.Ok)
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

        if not lock.isOpenCam:
            self.showMessageBox(QMessageBox.Warning, "Check your Main Camera", "Error", QMessageBox.Ok)
            return

        # validate robot is available ? by asking mcu
        # .....send buffer and validate
        self.buffer = robocod.concatenate(dev="1", cmd=robocod.CHECK_STATE, data=None)
        isSend = serialCom1.send_data(self.buffer)
        self.check_sending(isSend)
        if not isSend:
            return

        if isSend:
            if serialCom1.receive_buff[3] == 1:
                self.isRobotAvailable = True
                serialCom1.receive_buff = b'00000'
            elif serialCom1.receive_buff[3] == 0:
                self.isRobotAvailable = False
                serialCom1.receive_buff = b'00000'

        if not self.isRobotAvailable:
            self.showMessageBox(QMessageBox.Warning, "Move robot to home position", "Error", QMessageBox.Ok)
            return

        # on auto mode
        self.isAutoMode = True
        self.isPreviewMode = False
        lock.isReadSpeedMode = False

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
        self.isPreviewMode = True

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

    # region product data
    def change_page(self):
        self.current_page = 1 - self.current_page
        self.uic.stackedWidget.setCurrentIndex(self.current_page)
        if self.current_page == 0:
            self.uic.lb_info_page.setText("-- Page 2 of 2")
        else:
            self.uic.lb_info_page.setText("-- Page 1 of 2")

    def add_row_data(self, data: list):
        """
        :param data: No./DateTime/Product/Size/Color/Note
        :return:
        """
        self.row_position = self.uic.tableWidget.rowCount()
        self.uic.tableWidget.insertRow(self.row_position)
        for i in range(6):
            self.uic.tableWidget.setItem(self.row_position, i, QTableWidgetItem(str(data[i])))

    def refresh_sheet(self):
        self.uic.cbox_error.setChecked(False)
        self.uic.cbox_not_error.setChecked(False)
        self.uic.tableWidget.setRowCount(0)
        sheet = product_database.read_from_database()
        for row in sheet:
            self.add_row_data(row)
        self.product_index = len(sheet)

    def export_sheet_to_excel(self):
        if lock.isOpenCam or lock.isCalibCamMode or lock.isTestCamCalibModel or lock.isCalibSizeModelMode:
            self.showMessageBox(QMessageBox.Warning, "Cannot export data when the other mode is online", "Error",
                                QMessageBox.Ok)
            return

        if product_database.export_excel():
            self.showMessageBox(QMessageBox.Information, "Excel export successful", "Info", QMessageBox.Ok)
        else:
            self.showMessageBox(QMessageBox.Information, "Excel export unsuccessful", "Info", QMessageBox.Ok)

    def showAllData(self):
        for i in range(self.uic.tableWidget.rowCount()):
            self.uic.tableWidget.showRow(i)

    def customFilter(self, col_index: int, col_value: str):
        count = 0
        for i in range(self.uic.tableWidget.rowCount()):
            item = self.uic.tableWidget.item(i, col_index)
            if item.text() != col_value:
                self.uic.tableWidget.hideRow(i)
            else:
                count += 1
        self.uic.lb_quantity.setText(f"Quantity: {count}")

    def filter_error(self):
        self.uic.cb_size.setDisabled(True)
        self.uic.cb_color.setDisabled(True)
        self.uic.btn_filter.setDisabled(True)
        if self.uic.cbox_error.isChecked():
            if self.uic.cbox_not_error.isChecked():
                self.uic.cbox_not_error.setChecked(False)
                self.showAllData()
            # filter column 2, value "Error"
            self.customFilter(2, "Error")
        else:
            self.showAllData()
            self.uic.lb_quantity.setText("Quantity: ")

    def filter_not_error(self):
        if self.uic.cbox_not_error.isChecked():
            self.uic.cb_size.setDisabled(False)
            self.uic.cb_color.setDisabled(False)
            self.uic.btn_filter.setDisabled(False)

            if self.uic.cbox_error.isChecked():
                self.uic.cbox_error.setChecked(False)
                self.showAllData()
            # filter column 2, value "Not Error"
            self.customFilter(2, "Not Error")
        else:
            self.uic.cb_size.setDisabled(True)
            self.uic.cb_color.setDisabled(True)
            self.uic.btn_filter.setDisabled(True)
            self.showAllData()
            self.uic.lb_quantity.setText("Quantity: ")

    def detail_filter(self):
        self.showAllData()

        if self.uic.cb_size.currentText() == "All" and self.uic.cb_color.currentText() == "All":
            self.customFilter(2, "Not Error")
            return

        if self.uic.cb_size.currentText() == "All":
            self.customFilter(2, "Not Error")
            self.customFilter(4, self.uic.cb_color.currentText())
            return

        if self.uic.cb_color.currentText() == "All":
            self.customFilter(2, "Not Error")
            self.customFilter(3, self.uic.cb_size.currentText())
            return

        # size filter, color filter != "All"
        self.customFilter(2, "Not Error")
        count = 0
        for i in range(self.uic.tableWidget.rowCount()):
            item_size = self.uic.tableWidget.item(i, 3)
            item_color = self.uic.tableWidget.item(i, 4)
            if item_size.text() != self.uic.cb_size.currentText() or item_color.text() != self.uic.cb_color.currentText():
                self.uic.tableWidget.hideRow(i)
            else:
                count += 1
        self.uic.lb_quantity.setText(f"Quantity: {count}")

    def handle_cell_clicked(self, row, col):
        if col == 1:
            item = self.uic.tableWidget.item(row, col)
            search_name = item.text()[:6] + '_'
            folder = 'error_product'
            for file_name in os.listdir(folder):
                if search_name in file_name:
                    path = os.path.join(folder, file_name)
                    image = read_image(path)
                    self.uic.error_product_view.setPixmap(self.convert_cv_qt(image, 480, 670))
                    del image
                    return

    # endregion

    # region dialog menubar
    @staticmethod
    def show_size_dialog():
        sizeDialog = SizeDialog()
        sizeDialog.Size_UI.show()
        sizeDialog.Size_UI.exec_()

    @staticmethod
    def show_camera_dialog():
        camDialog = CameraDialog()
        camDialog.Cam_UI.show()
        camDialog.Cam_UI.exec_()

    @staticmethod
    def show_speed_dialog():
        speedDialog = SpeedDialog()
        speedDialog.Speed_UI.show()
        speedDialog.Speed_UI.exec_()

    @staticmethod
    def exit():
        sys.exit(app.exec_())

    # endregion


class SpeedDialog(MainWindow):
    def __init__(self):
        super().__init__()
        self.Speed_UI = QDialog()
        self.uic_speed = gui_speed.Ui_Dialog()
        self.uic_speed.setupUi(self.Speed_UI)

        # Timer Interrupt
        self.timer_speed = QtCore.QTimer()
        self.timer_speed.setInterval(1000)
        self.timer_speed.timeout.connect(self.update)

        # PID graph setting
        self.scene = QGraphicsScene()
        self.uic_speed.graphicsView.setScene(self.scene)
        self.x, self.t = [], 0
        self.y_pv, self.y_sp = [], []
        self.fig = plt.figure()
        self.fig.set_size_inches(550 / 100, 360 / 90)
        plt.xlabel('time (s)')
        plt.ylabel('speed conveyor (cm/s)')
        plt.title("PID Control")
        plt.ylim([0, 10])

        # event text edit
        self.uic_speed.txt_setpoint_sp.textChanged.connect(self.sp_convey_changed)

        # event combo box
        self.uic_speed.cb_low_area.activated.connect(self.list_low_area())
        self.uic_speed.cb_high_area.activated.connect(self.list_high_area())

        # event push button
        self.uic_speed.btn_apply_rf.clicked.connect(self.apply_rf_model)
        self.uic_speed.btn_run_convey.clicked.connect(self.run_conveyor)
        self.uic_speed.btn_clear_graph.clicked.connect(self.clear_graph)

        self.uic_speed.btn_apply_rf.setToolTip("Apply the 'pick-place' model and stop PID graph")
        self.uic_speed.btn_apply_rf.setToolTipDuration(10000)

    def update(self):
        # device 2: stm32/conveyor
        try:
            if lock.isReadSpeedMode:
                self.buffer = robocod.concatenate(dev="2", cmd=robocod.READSPEED, data=None)
                _ = serialCom2.send_data(self.buffer)
                if serialCom2.receive_buff[1] == 0x32:
                    integer = int(serialCom2.receive_buff[2])
                    dec = int(serialCom2.receive_buff[3])
                    cSpeed = float(integer + dec / 10)
                    self.uic_speed.txt_pv_sp.setText(str(cSpeed))
                    serialCom2.receive_buff = b'00000'

                    self.draw_graph(self.t, float(self.uic_speed.txt_setpoint_sp.toPlainText()), cSpeed)
                    self.t += 1
        except Exception as e:
            print(e, "-speed")

    # region conveyor speed
    def draw_graph(self, x, y_sp, y_pv):
        self.uic_speed.graphicsView.scene().clear()
        self.x.append(x)
        self.y_pv.append(y_pv)
        self.y_sp.append(y_sp)
        plt.plot(self.x, self.y_pv, 'r')
        plt.plot(self.x, self.y_sp, 'b')
        self.fig.savefig('data/graph.png')
        pixmap = QPixmap('data/graph.png')
        item = QGraphicsPixmapItem(pixmap)
        self.uic_speed.graphicsView.scene().addItem(item)
        self.uic_speed.graphicsView.show()

    def list_low_area(self):
        it = ["-10", "-9", "-8", "-7", "-6", "-5", "-4", "-3", "-2", "-1", "0"]
        self.uic_speed.cb_low_area.addItems(it)
        self.uic_speed.cb_low_area.setCurrentIndex(10)

    def list_high_area(self):
        it = ["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "10"]
        self.uic_speed.cb_high_area.addItems(it)
        self.uic_speed.cb_high_area.setCurrentIndex(3)

    def apply_rf_model(self):
        if self.uic_speed.btn_apply_rf.text() == "Apply":
            rfPoint.CONVEYOR_VELOCITY = int(self.uic_speed.txt_convey_sp_rf.toPlainText())
            rfPoint.LOW_WORKING_Y_AREA = int(self.uic_speed.cb_low_area.currentText())
            rfPoint.HIGH_WORKING_Y_AREA = int(self.uic_speed.cb_high_area.currentText())
            rfPoint.create_new_model()
            self.uic_speed.txt_info_convey_rf.setText("Applied!")
            self.uic_speed.txt_info_convey_rf.setStyleSheet("background: rgb(0,255,0);border:none;")
            self.uic_speed.btn_apply_rf.setText("Cancel")
            lock.isApplyRF = True
            lock.isReadSpeedMode = False
            self.x.clear()
            self.y_pv.clear()
            self.y_sp.clear()
            self.timer_speed.stop()
        else:
            self.uic_speed.txt_info_convey_rf.clear()
            self.uic_speed.txt_info_convey_rf.setStyleSheet("background: transparent;border:none;")
            self.uic_speed.btn_apply_rf.setText("Apply")
            lock.isApplyRF = False

    def sp_convey_changed(self):
        self.uic_speed.txt_convey_sp_rf.setText(self.uic_speed.txt_setpoint_sp.toPlainText())

    def run_conveyor(self):
        if self.uic_speed.btn_run_convey.text() == "Start Conveyor":
            if not 1 <= float(self.uic_speed.txt_setpoint_sp.toPlainText()) <= 6:
                self.showMessageBox(QMessageBox.Warning, "Set-point is out of range [1,6] cm", "Error", QMessageBox.Ok)
                return

            self.buffer = robocod.concatenate(dev="2", cmd=robocod.RUNCONVEY,
                                              data=[self.uic_speed.txt_setpoint_sp.toPlainText(),
                                                    0, 0, 0, 0, 0])
            isSend = serialCom2.send_data(self.buffer)
            self.check_sending(isSend)
            if not isSend:
                return

            self.uic_speed.btn_run_convey.setText("Stop Conveyor")
            self.uic_speed.light_stt_convey_sp.setStyleSheet(
                "background: rgb(51,255,51); border-radius:35px; border-color: rgb(0, 0, 0); "
                "border-width : 1px; border-style:inset;")
            lock.isRunConveyor = True
            lock.isReadSpeedMode = True
            self.uic_speed.txt_setpoint_sp.setDisabled(True)
            self.timer_speed.start()
        else:
            self.buffer = robocod.concatenate(dev="2", cmd=robocod.STOPCONVEY, data=None)
            isSend = serialCom2.send_data(self.buffer)
            self.check_sending(isSend)
            if not isSend:
                return

            self.uic_speed.btn_run_convey.setText("Start Conveyor")
            self.uic_speed.light_stt_convey_sp.setStyleSheet(
                "background: rgb(255,0,0); border-radius:35px; border-color: rgb(0, 0, 0); "
                "border-width : 1px; border-style:inset;")
            lock.isRunConveyor = False
            lock.isReadSpeedMode = False
            self.uic_speed.txt_setpoint_sp.setDisabled(False)
            self.timer_speed.stop()

    def clear_graph(self):
        self.x.clear()
        self.y_pv.clear()
        self.y_sp.clear()
        self.t = 0
        self.fig.clf()
        self.uic_speed.graphicsView.scene().clear()
        plt.xlabel('time (s)')
        plt.ylabel('speed conveyor (cm/s)')
        plt.title("PID Control")
        plt.ylim([0, 10])

    # endregion


class CameraDialog(MainWindow):
    def __init__(self):
        super().__init__()
        self.Cam_UI = QDialog()
        self.uic_cam = gui_camera.Ui_CameraDialog()
        self.uic_cam.setupUi(self.Cam_UI)

        self.isTakeShot = False
        self.isCalibCam = False

        self.uic_cam.btn_calib_mode_cam.setDisabled(True)
        self.uic_cam.btn_scrshot.setDisabled(True)
        self.uic_cam.btn_calib_cam.setDisabled(True)
        self.uic_cam.btn_save_model.setDisabled(True)

        self.uic_cam.cb_opt_calib_cam_2.activated.connect(self.list_opt_calib())
        self.uic_cam.cb_opt_calib_cam_2.currentTextChanged.connect(self.changed_opt)
        self.uic_cam.cb_calib_model.activated.connect(self.list_calibrated_model())

        self.uic_cam.btn_calib_mode_cam.clicked.connect(self.calib_cam_model)
        self.uic_cam.btn_scrshot.clicked.connect(self.take_screen_shot)
        self.uic_cam.btn_calib_cam.clicked.connect(self.is_calib_cam)
        self.uic_cam.btn_save_model.clicked.connect(self.is_save_cam_calib_model)
        self.uic_cam.btn_apply_calib_model.clicked.connect(self.apply_calib_cam_model)
        self.uic_cam.btn_on_cam_test.clicked.connect(self.test_calib_cam_model)

        self.uic_cam.slide_points_test.valueChanged.connect(self.set_num_points)

    # region camera calibration
    def list_opt_calib(self):
        it = ["Default Calibration", "Custom Calibration"]
        self.uic_cam.cb_opt_calib_cam_2.addItems(it)

    def list_calibrated_model(self):
        ex_files_name, in_files_name = camera_calib.list_model_names()
        self.uic_cam.cb_calib_model.addItems(ex_files_name)
        self.uic_cam.cb_calib_model_in.addItems(in_files_name)

    def changed_opt(self):
        if self.uic_cam.cb_opt_calib_cam_2.currentText() == "Default Calibration":
            self.uic_cam.lb_opt_calib_cam_2.setText("Use pretrained model")
            self.uic_cam.lb_opt_calib_cam_2.setStyleSheet("color: rgb(0,0,255);")
            self.uic_cam.btn_calib_mode_cam.setDisabled(True)
            self.uic_cam.btn_scrshot.setDisabled(True)
            self.uic_cam.btn_calib_cam.setDisabled(True)
            self.uic_cam.btn_save_model.setDisabled(True)
            self.uic_cam.btn_apply_calib_model.setDisabled(False)
            self.uic_cam.btn_on_cam_test.setDisabled(False)
            self.uic_cam.cb_calib_model.clear()
            self.uic_cam.cb_calib_model_in.clear()
            self.list_calibrated_model()
        else:
            self.uic_cam.lb_opt_calib_cam_2.setText("Create new calib model")
            self.uic_cam.lb_opt_calib_cam_2.setStyleSheet("color: rgb(255,0,0);")
            self.uic_cam.btn_calib_mode_cam.setDisabled(False)
            self.uic_cam.btn_apply_calib_model.setDisabled(True)
            self.uic_cam.btn_on_cam_test.setDisabled(True)

    def calib_cam_model(self):
        if self.uic_cam.btn_calib_mode_cam.text() == "On Mode":
            if lock.isOpenCam:
                self.showMessageBox(QMessageBox.Warning, "Main Camera is online", "Error", QMessageBox.Ok)
                return
            if lock.isCalibSizeModelMode:
                self.showMessageBox(QMessageBox.Warning, "Size calibration mode is online", "Error", QMessageBox.Ok)
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
                    camera_calib.counter_shot_img = 1
                else:
                    camera_calib.counter_shot_img = num_files + 1

            if mycam.connect_cam():
                lock.isCalibCamMode = True
                self.uic_cam.btn_calib_mode_cam.setText("Off Mode")
                self.uic_cam.btn_scrshot.setDisabled(False)
                self.uic_cam.btn_calib_cam.setDisabled(False)
                self.uic_cam.cb_opt_calib_cam_2.setDisabled(True)
                self.uic_cam.btn_calib_mode_cam.setStyleSheet("background: rgb(0,255,0);color: rgb(0,0,0);")
                camera_calib.check_dir()
                self.uic_cam.txt_info_calib_cam.setText("Start the calibration process")
        else:
            lock.isCalibCamMode = False
            if mycam.disconnect_cam():
                self.uic_cam.btn_calib_mode_cam.setText("On Mode")
                self.uic_cam.btn_scrshot.setDisabled(True)
                self.uic_cam.btn_calib_cam.setDisabled(True)
                self.uic_cam.btn_save_model.setDisabled(True)
                self.uic_cam.cb_opt_calib_cam_2.setDisabled(False)
                self.uic_cam.btn_calib_mode_cam.setStyleSheet("background: rgb(235,235,235);color: rgb(0,0,0);")
                self.uic_cam.calib_cam_view.clear()
                self.uic_cam.txt_info_calib_cam.setText("Stop the calibration process")

        def display():
            try:
                while lock.isCalibCamMode:
                    if self.isCalibCam:
                        self.uic_cam.calib_cam_view.clear()
                        self.isCalibCam = False
                        break

                    image = mycam.get_frames()
                    isCheckerBoard, show_img, save_img = camera_calib.detect_checker_board(image)
                    self.uic_cam.calib_cam_view.setPixmap(self.convert_cv_qt(show_img, 720, 530))

                    del show_img

                    if self.isTakeShot:
                        _ = camera_calib.save_images(isCheckerBoard, save_img)
                        self.isTakeShot = False
                        del save_img

                    if not lock.isCalibCamMode:
                        self.uic_cam.calib_cam_view.clear()
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
        self.uic_cam.btn_save_model.setDisabled(False)
        self.uic_cam.btn_scrshot.setDisabled(True)
        self.uic_cam.txt_info_calib_cam.setText(f"Use data at: '../{camera_calib.img_dir_path}' to calibrate")
        info_calib = camera_calib.calibrate_camera(mode=self.uic_cam.cb_intrinsic_extrinsic.currentText())
        self.uic_cam.txt_info_calib_cam.setText(info_calib)

    def is_save_cam_calib_model(self):
        if not self.uic_cam.txt_name_calib_model.toPlainText():
            self.uic_cam.txt_info_calib_cam.setStyleSheet("background: rgb(255,0,0);")
            self.uic_cam.txt_info_calib_cam.setText("Insert the calibrate model name to continue")
        else:
            self.uic_cam.txt_info_calib_cam.setStyleSheet("background: transparent;")
            model_name = self.uic_cam.txt_name_calib_model.toPlainText()
            if self.uic_cam.cb_intrinsic_extrinsic.currentText() == "Extrinsic Params":
                self.uic_cam.txt_info_calib_cam.setText(f"Save model into: '../{camera_calib.extrinsic_path}/{model_name}'")
            else:
                self.uic_cam.txt_info_calib_cam.setText(f"Save model into: '../{camera_calib.intrinsic_path}/{model_name}'")
            self.uic_cam.btn_save_model.setDisabled(True)
            info_save, _ = camera_calib.save_model(model_name, self.uic_cam.cb_intrinsic_extrinsic.currentText())
            self.uic_cam.txt_info_calib_cam.setText(info_save)

    def apply_calib_cam_model(self):
        info_apply = camera_calib.load_model(self.uic_cam.cb_calib_model.currentText(),
                                             self.uic_cam.cb_calib_model_in.currentText())
        self.uic_cam.txt_info_calib_cam.setText(info_apply)

    def test_calib_cam_model(self):
        if self.uic_cam.btn_on_cam_test.text() == "ON":
            if lock.isOpenCam:
                self.showMessageBox(QMessageBox.Warning, "Main Camera is online", "Error", QMessageBox.Ok)
                return
            if lock.isCalibSizeModelMode:
                self.showMessageBox(QMessageBox.Warning, "Size calibration mode is online", "Error", QMessageBox.Ok)
                return
            if camera_calib.Z == 0:
                self.showMessageBox(QMessageBox.Warning, "Calibration model is not found", "Error", QMessageBox.Ok)
                return
            if mycam.connect_cam():
                self.uic_cam.btn_on_cam_test.setText("OFF")
                self.uic_cam.btn_on_cam_test.setStyleSheet("background: rgb(0,255,0);")
                self.uic_cam.txt_info_calib_cam.setText("Insert the checkerboard to test model")
                lock.isTestCamCalibModel = True
                self.uic_cam.btn_apply_calib_model.setDisabled(True)
        else:
            lock.isTestCamCalibModel = False
            if mycam.disconnect_cam():
                self.uic_cam.btn_on_cam_test.setText("ON")
                self.uic_cam.btn_on_cam_test.setStyleSheet("background: rgb(235,235,235);")
                self.uic_cam.calib_cam_view.clear()
                self.uic_cam.btn_apply_calib_model.setDisabled(False)

        def display():
            while lock.isTestCamCalibModel:
                try:
                    while lock.isTestCamCalibModel:
                        image = mycam.get_frames()
                        show_img = camera_calib.get_undistorted_images(image)
                        show_img = camera_calib.test_model_use_checkerboard(show_img, int(self.set_num_points()))
                        self.uic_cam.calib_cam_view.setPixmap(self.convert_cv_qt(show_img, 720, 540))

                        if not lock.isTestCamCalibModel:
                            self.uic_cam.calib_cam_view.clear()
                            break

                except Exception as e:
                    print(e, "-test calib cam mode")

        display_thread = threading.Thread(target=display)
        display_thread.start()

    def set_num_points(self):
        return self.uic_cam.slide_points_test.value()

    # endregion


class SizeDialog(MainWindow):
    def __init__(self):
        super().__init__()
        self.Size_UI = QDialog()
        self.uic_size = gui_size.Ui_Dialog()
        self.uic_size.setupUi(self.Size_UI)
        self.result = ''
        self.isSizeCalibModel = False
        self.isTestCalibSizeModel = False

        # Timer Interrupt
        self.timer_size = QtCore.QTimer()
        self.timer_size.setInterval(500)
        self.timer_size.timeout.connect(self.update)

        self.uic_size.btn_calib_size.setDisabled(True)
        self.uic_size.btn_test_size.setDisabled(True)

        self.uic_size.btn_calib_mode.clicked.connect(self.calib_model_mode)
        self.uic_size.btn_calib_size.clicked.connect(self.is_calib)
        self.uic_size.btn_test_size.clicked.connect(self.is_test_calib)

    def update(self):
        self.uic_size.res_size.setText(self.result)

    def calib_model_mode(self):
        if self.uic_size.btn_calib_mode.text() == "On Mode":
            if lock.isOpenCam:
                self.showMessageBox(QMessageBox.Warning, "Main camera is online", "Error", QMessageBox.Ok)
                return
            if lock.isCalibCamMode or lock.isTestCamCalibModel:
                self.showMessageBox(QMessageBox.Warning, "Camera calibration mode is online", "Error", QMessageBox.Ok)
                return
            if mycam.connect_cam():
                lock.isCalibSizeModelMode = True
                self.uic_size.btn_calib_size.setDisabled(False)
                self.uic_size.btn_test_size.setDisabled(False)
                self.uic_size.btn_calib_mode.setText("Off Mode")
                self.uic_size.btn_calib_mode.setStyleSheet("background: rgb(0,255,0);")
                self.uic_size.lb_info.setText("Start calibrating size model\nInsert standard size 30 object")
                self.timer_size.start()
                ffeats.isShowLogoFeats = False
        else:
            lock.isCalibSizeModelMode = False
            if mycam.disconnect_cam():
                self.uic_size.btn_calib_size.setDisabled(True)
                self.uic_size.btn_test_size.setDisabled(True)
                self.uic_size.btn_calib_mode.setText("On Mode")
                self.uic_size.btn_calib_mode.setStyleSheet("background: rgb(235,235,235);")
                self.uic_size.lb_info.setText("Stop calibrating size model")
                self.timer_size.stop()

        def display():
            try:
                while lock.isCalibSizeModelMode:
                    image = mycam.get_frames()
                    isObject, _, _, bbox, _ = object_processing.obj_detector_process(image)
                    if isObject:
                        x1, y1, x2, y2 = bbox[0], bbox[1], bbox[2], bbox[3]
                        img_org, img_contour = preprocessing_obj(image, (x1, y1, x2, y2))
                        img, _, d1, d2, d3 = ffeats.find_size_features(img_org, img_contour)
                        self.uic_size.calib_size_view.setPixmap(self.convert_cv_qt(img, 600, 450))

                        if self.isSizeCalibModel:
                            self.isSizeCalibModel = False
                            size_calib.convert(d1, d2, d3)
                            sz_color_processing.cf1 = size_calib.cf1
                            sz_color_processing.cf2 = size_calib.cf2
                            sz_color_processing.cf3 = size_calib.cf3
                            self.result = ''

                        if self.isTestCalibSizeModel:
                            self.isTestCalibSizeModel = False
                            self.result = size_calib.test(d1, d2, d3)
                    else:
                        self.uic_size.calib_size_view.setPixmap(self.convert_cv_qt(image, 600, 450))
                        self.isTestCalibSizeModel = False
                        self.isSizeCalibModel = False

                if not lock.isCalibSizeModelMode:
                    self.uic_size.calib_size_view.clear()

            except Exception as e:
                print(e)

        display_thread = threading.Thread(target=display)
        display_thread.start()

    def is_calib(self):
        self.isSizeCalibModel = True
        self.uic_size.lb_info.setText("Calibrated")

    def is_test_calib(self):
        self.isTestCalibSizeModel = True
        self.uic_size.lb_info.setText("Test new calibrated model")


if __name__ == '__main__':
    # load the size, color model
    sz_color_processing = SizeAndColorProcess()

    # load yolov5 model
    object_processing = ObjectProcess()

    # load logo svm model
    logo_processing = LogoProcess()

    # load lock variables
    lock = Lock()

    app = QApplication(sys.argv)
    main = MainWindow()
    main.Control_UI.show()
    sys.exit(app.exec_())
