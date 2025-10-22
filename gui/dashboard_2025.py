# pyqt example for teaching GUI development for FRC dashboard
# make sure to pip install pyqt6 pyqt6-tools
import math
# print(f'Loading Modules ...', flush=True)'
import os
os.environ["OPENCV_LOG_LEVEL"] = "DEBUG"  # Options: INFO, WARNING, ERROR, DEBUG

import cv2
print("[DEBUG] OpenCV version:", cv2.__version__)
cv2.setNumThreads(4)  # Disable multithreading
cv2.ocl.setUseOpenCL(True)

import time
from datetime import datetime
from pathlib import Path
import urllib.request
import cv2
import numpy as np
import re

from PyQt6 import QtCore, QtGui, QtWidgets, uic
from PyQt6.QtCore import Qt, QTimer, QEvent, QThread, QObject, pyqtSignal
#from PyQt6.QtWidgets import  QApplication, QTreeWidget, QTreeWidgetItem
from PyQt6.QtWidgets import QGraphicsOpacityEffect

import qlabel2
from warning_label import WarningLabel

from ntcore import NetworkTableType, NetworkTableInstance
import wpimath.geometry as geo

#print(f'Initializing GUI ...', flush=True)

# Worker class for the video thread
class CameraWorker(QObject):
    finished = pyqtSignal()
    progress = pyqtSignal(int)

    def __init__(self, qtgui):
        super().__init__()
        self.qtgui = qtgui
        self.frames = 0
        self.previous_read_time = 0
        self.max_fps = 20
        self.error_count = 0
        self.cv_backend = cv2.CAP_FFMPEG  # cv2.CAP_FFMPEG or cv2.CAP_DSHOW

    def stop(self):
        self.running = False

    def run(self):
        """Blocking task that may take a long time to run"""
        self.running = True
        old_url = ''  # need to see if we switch cameras
        try:
            while self.running:
                if self.qtgui.qradiobutton_autoswitch.isChecked():  # auto determine which camera to show
                    # pass
                    # shooter_on = self.qtgui.widget_dict['qlabel_shooter_indicator']['entry'].getBoolean(False)
                    #elevator_low = self.qtgui.widget_dict['qlcd_elevator_height']['entry'].getDouble(100) < 100
                    url = self.qtgui.camera_dict['LogitechReef']['URL'] if True else self.qtgui.camera_dict['ArducamHigh']['URL']
                else:
                    url = self.qtgui.camera_dict[self.qtgui.qcombobox_cameras.currentText()]['URL']  # figure out which url we want

                # stream = urllib.request.urlopen('http://10.24.29.12:1187/stream.mjpg')
                # get and display image
                if url != old_url:  # try to only do this if we switch streams - seems to cause a lot of errors / increases bandwidth
                    cap = cv2.VideoCapture(url, self.cv_backend)
                    if not cap.isOpened():
                        print("Cannot open stream")
                        return
                old_url = url

                now = time.time()
                if now - self.previous_read_time > 1.0 / self.max_fps:
                    try:
                        ret, frame = cap.read()
                        if not ret:
                            print("[ERROR] OpenCV stopped receiving frames. Retrying...")
                            time.sleep(0.5)  # Wait before retrying
                            cap.release()
                            cap = cv2.VideoCapture(url, self.cv_backend)  # Reinitialize
                            continue

                        self.frames += 1
                        pixmap = self.qtgui.convert_cv_qt(frame, self.qtgui.qlabel_camera_view)
                        self.qtgui.qlabel_camera_view.setPixmap(pixmap)
                        # self.qtgui.qlabel_camera_view: QtWidgets.QLabel
                        # self.qtgui.qlabel_camera_view.repaint()  # do not repaint in the thread.  the main loop takes care of that.
                        # self.progress.emit(1)
                        self.previous_read_time = now
                        if self.frames % 1000 == 0:
                            print(f'{datetime.today().strftime("%H%M%S")}: frames received: {self.frames}')
                    except Exception as e:
                        self.error_count += 1
                        self.previous_read_time = now
                        if self.error_count % 100 == 0:
                            print(f'{datetime.today().strftime("%H%M%S")} error count:{self.error_count}: cv read error: {e}')
                        # should I stop the thread here? or just let the user fix it by restarting manually
                else:
                    # what should this else be?  a grab to flush the camera?  it seems to bulke a buffer of frames if we don't flush
                    cap.grab()

        except Exception as e:
            print(f'{datetime.today().strftime("%H%M%S")} Camera worker crashed: {e}')
        self.finished.emit()

class Ui(QtWidgets.QMainWindow):
    # set the root dir for the project, knowing we're one deep
    root_dir = Path('.').absolute()  # set this to be in the root, not a child, so no .parent. May need to change this.
    png_dir = root_dir / 'png'
    save_dir = root_dir / 'save'

    # -------------------  INIT  --------------------------
    def __init__(self):
        super(Ui, self).__init__()
        # trick to inherit all the UI elements from the design file  - DO NOT CODE THE LAYOUT!
        uic.loadUi('layout_2025.ui', self)  # if this isn't in the directory, you got no program

        # set up network tables - TODO need to really see which of these is necessary
        self.ntinst = NetworkTableInstance.getDefault()
        self.servers = ["10.24.29.2", "127.0.0.1"] #  "roboRIO-2429-FRC.local"]  # need to add the USB one here
        # self.ntinst.startClient3(identity=f'PyQt Dashboard {datetime.today().strftime("%H%M%S")}')
        self.ntinst.startClient4(identity=f'PyQt Dashboard {datetime.today().strftime("%H%M%S")}')
        self.server_index = 0  # manually do a round-robin later
        # self.ntinst.setServer("127.0.0.1",0)
        #self.ntinst.setServer(servers=self.servers)  # does not seem to work in round-robin in 2023 code
        self.ntinst.setServerTeam(2429)
        self.connected = self.ntinst.isConnected()

        self.sorted_tree = None  # keep a global list of all the nt addresses
        self.autonomous_list = []  # set up an autonomous list

        self.refresh_time = 50  # milliseconds before refreshing  - i've run it at 16 and it is fine when no video
        self.previous_frames = 0
        self.widget_dict = {}
        self.command_dict = {}
        self.camera_enabled = False
        self.worker = None
        self.thread = None
        # 20251021 moved all stream variables into the camera dict
        self.camera_dict = {
            'GeniusLow': {'URL': 'http://10.24.29.12:1186/stream.mjpg', 'IS_ALIVE': False, 'TIMESTAMP_ENTRY': self.ntinst.getEntry('/Cameras/GeniusLow/_timestamp'),
                          'CONNECTIONS': 0, 'CONNECTIONS_ENTRY': self.ntinst.getEntry('/Cameras/GeniusLow/_connections'), 'NICKNAME': 'GENIUS LO', 'INDICATOR': self.qlabel_genius_low_indicator},
            'ArducamBack': {'URL': 'http://10.24.29.12:1187/stream.mjpg', 'IS_ALIVE': False, 'TIMESTAMP_ENTRY': self.ntinst.getEntry('/Cameras/ArducamBack/_timestamp'),
                            'CONNECTIONS': 0, 'CONNECTIONS_ENTRY': self.ntinst.getEntry('/Cameras/ArducamBack/_connections'), 'NICKNAME': 'ARDU BACK', 'INDICATOR': self.qlabel_arducam_back_indicator},
            'LogitechReef': {'URL': 'http://10.24.29.13:1186/stream.mjpg', 'IS_ALIVE': False, 'TIMESTAMP_ENTRY': self.ntinst.getEntry('/Cameras/LogitechReef/_timestamp'),
                             'CONNECTIONS': 0, 'CONNECTIONS_ENTRY': self.ntinst.getEntry('/Cameras/LogitechReef/_connections'), 'NICKNAME': 'LOGI REEF', 'INDICATOR': self.qlabel_logitech_reef_indicator},
            'ArducamHigh': {'URL': 'http://10.24.29.13:1187/stream.mjpg', 'IS_ALIVE': False, 'TIMESTAMP_ENTRY': self.ntinst.getEntry('/Cameras/ArducamHigh/_timestamp'),
                            'CONNECTIONS': 0, 'CONNECTIONS_ENTRY': self.ntinst.getEntry('/Cameras/ArducamHigh/_connections'), 'NICKNAME': 'ARDU HI', 'INDICATOR': self.qlabel_arducam_high_indicator},
            'Raw GeniusLow': {'URL': 'http://10.24.29.12:1181/stream.mjpg'},
            'Raw ArduBack': {'URL': 'http://10.24.29.12:1182/stream.mjpg'},
            'Raw LogiReef': {'URL': 'http://10.24.29.13:1181/stream.mjpg'},
            'Raw ArduHigh': {'URL': 'http://10.24.29.13:1182/stream.mjpg'},
            'Debug': {'URL': 'http://127.0.0.1:1186/stream.mjpg'},
        }

        # --------------  CAMERA STATUS INDICATORS  ---------------
        self.robot_timestamp_entry = self.ntinst.getEntry('/SmartDashboard/_timestamp')

        # --------------  ROBOT VOLTAGE AND CURRENT  ---------------
        # set up the warning labels - much of the formatting is handled in the widget class itself
        self.qlabel_pdh_voltage_monitor: WarningLabel
        self.qlabel_pdh_voltage_monitor.update_settings(min_val=8, max_val=12, red_high=False, display_float=True)
        self.qlabel_pdh_current_monitor.update_settings(min_val=60, max_val=160, red_high=True, display_float=False)

        self.initialize_widgets()
        #QTimer.singleShot(2000, self.initialize_widgets())  # wait 2s for NT to initialize

        # all of your setup code goes here - linking buttons to functions, etc (move to seperate funciton if too long)

        # menu items
        self.qaction_show_hide.triggered.connect(self.toggle_network_tables)  # show/hide networktables
        self.qaction_refresh.triggered.connect(self.refresh_tree)

        # widget customization
        #self.qlistwidget_commands.setStyleSheet("QListView::item:selected{background-color: rgb(255,255,255);color: rgb(0,0,0);}")
        self.qlistwidget_commands.clicked.connect(self.command_list_clicked)
        self.qcombobox_autonomous_routines.currentTextChanged.connect(self.update_routines)
        self.qt_text_entry_filter.textChanged.connect(self.filter_nt_keys_combo)
        self.qcombobox_nt_keys.currentTextChanged.connect(self.update_selected_key)
        self.qt_tree_widget_nt.clicked.connect(self.qt_tree_widget_nt_clicked)

        self.qt_text_entry_filter.installEventFilter(self)
        self.qt_text_new_value.installEventFilter(self)

        self.robot_pixmap = QtGui.QPixmap("png\\blockhead.png")  # for the field update
        self.quest_pixmap = QtGui.QPixmap("png\\quest.png")
        # Create an opacity effect with 50% transparency - quest icon
        opacity_effect = QGraphicsOpacityEffect()
        opacity_effect.setOpacity(0.5)
        self.qlabel_quest.setGraphicsEffect(opacity_effect)
        self.qlabel_robot.raise_()  # put the robot above the other labels (i.e. above the quest)

        # button connections
        self.qt_button_set_key.clicked.connect(self.update_key)
        self.qt_button_swap_sim.clicked.connect(self.increment_server)
        self.qt_button_reconnect.clicked.connect(self.reconnect)
        # self.qt_button_camera_enable.clicked.connect(lambda _: setattr(self, 'camera_enabled', not self.camera_enabled))
        self.qt_button_camera_enable.clicked.connect(self.toggle_camera_thread)

        # hide networktables
        self.qt_tree_widget_nt.hide()

        self.keys_currently_pressed = []
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)  # what happened to this in Qt6?

        # at the end of init, you need to show yourself
        self.show()

        # set up the refresh
        self.counter = 1
        self.previous_time = time.time()

        # set up the refresh
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_widgets)
        self.timer.start(self.refresh_time)



        # if you need to print out the list of children
        # children = [(child.objectName()) for child in self.findChildren(QtWidgets.QWidget) if child.objectName()]
        # children.sort()
        # for child in children:
        #    print(child)

    # ------------------- FUNCTIONS, MISC FOR NOW  --------------------------

    def reconnect(self):  # reconnect to NT4 server - do i ever need this?
        sleep_time = 0.1

        self.ntinst.stopClient()
        time.sleep(sleep_time)
        self.ntinst.disconnect()
        time.sleep(sleep_time)
        self.ntinst = NetworkTableInstance.getDefault()
        self.ntinst.startClient4(identity=f'PyQt Dashboard {datetime.today().strftime("%H%M%S")}')
        self.ntinst.setServerTeam(2429)
        time.sleep(sleep_time)
        self.connected = self.ntinst.isConnected()

    def increment_server(self):  # changes to next server in server list - TODO - figure our how to make this immediate
        current_server = self.servers[self.server_index]
        self.server_index = (self.server_index + 1) % len(self.servers)
        self.ntinst.setServer(server_name=self.servers[self.server_index])

        print(f'Changed server from {current_server} to {self.servers[self.server_index]}')
        self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Changed server from {current_server} to {self.servers[self.server_index]} ... wait 5s')

        # nothing seems to help speed this up - none of the following do anything useful
        # self.ntinst.flush()  # seems to take like 5s to change servers - flush does not help, disconnect does not help
        # self.ntinst.stopClient()  # makes things worse by hanging until the new connection is active
        # self.ntinst.startClient4(identity=f'PyQt Dashboard {datetime.today().strftime("%H%M%S")}')
        # self.connected = self.ntinst.isConnected()

    def check_url(self, url):
        try:
            code = urllib.request.urlopen(url, timeout=0.2).getcode()

            if code == 200:
                print(f'Successfully checked {url} ... return code is {code}')
                return True
            else:
                print(f'Unsuccessful check of {url} ... return code is {code}')
        except Exception as e:
            print(f'Failure: attempted to check {url} with exception {e}')
        return False

    def update_selected_key(self):
        x = self.ntinst.getEntry(self.qcombobox_nt_keys.currentText()).getValue()
        if x is not None:
            self.qt_text_current_value.setPlainText(str(x.value()))

    def convert_cv_qt(self, cv_img, qlabel):
        """Convert from an opencv image to QPixmap"""
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)

        p = convert_to_Qt_format.scaled(qlabel.width(), qlabel.height(), Qt.AspectRatioMode.KeepAspectRatio)
        return QtGui.QPixmap.fromImage(p)

    def toggle_camera_thread(self):
        # ToDo: check to see if the thread is running, then start again

        live_cams = []
        servers = []
        # check if server is running - at the moment we are focusing on genius_low
        for key in self.camera_dict.keys():
            url = self.camera_dict[key]['URL']
            status = self.check_url(url)
            live_cams.append(status)
            if status:
                pass

        if any(live_cams):
        # if self.check_url(self.camera_dict['LogitechReef']) or self.check_url(self.camera_dict['ArducamHigh']) or self.check_url(self.camera_dict['Debug']):
            if self.thread is None:  # first time through we need to make the thread
                self.thread = QThread()  # create a QThread object
                self.worker = CameraWorker(qtgui=self)  # create a CameraWorker object, pass it the main gui
                self.worker.moveToThread(self.thread)  # move the worker to the thread
                # connect signals and slots
                self.thread.started.connect(self.worker.run)
                self.worker.finished.connect(self.thread.quit)
                #self.worker.finished.connect(self.worker.deleteLater)
                #self.thread.finished.connect(self.thread.deleteLater)
                # self.worker.progress.connect(self.reportProgress)
                self.thread.start()  # start the thread
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Starting camera thread')
            elif self.thread.isRunning():
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Interrupting existing running camera thread')
                self.worker.stop()

                # quit or exit?  neither seems to do anything  TODO: add signals so this works correctly
               # self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Restarting existing running camera thread')
                self.thread.quit()
                self.thread.exit()
                # ToDo: figure out why we can't restart here but we can on the next iteration
                #self.thread.start()
            else:  # thread died for some reason
                self.thread.start()
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Restarting existing stopped camera thread')

        else:  # no valid servers
            if self.thread is not None:
                self.worker.stop()
                self.thread.quit()
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Terminating camera thread: no valid camera servers')
            else:
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: No valid camera servers, unable to start thread')

        """        if self.camera_enabled:
            if self.qradiobutton_autoswitch.isChecked():  # auto determine which camera to show
                shooter_on = self.widget_dict['qlabel_shooter_indicator']['entry'].getBoolean(False)
                url = self.camera_dict['ShooterCam'] if shooter_on else self.camera_dict['BallCam']
            else:
                url = self.camera_dict[self.qcombobox_cameras.currentText()]  # figure out which url we want
            # stream = urllib.request.urlopen('http://10.24.29.12:1187/stream.mjpg')
            # get and display image
            cap = cv2.VideoCapture(url)
            if not cap.isOpened():
                print("Cannot open stream")
                return
            ret, frame = cap.read()
            pixmap = self.convert_cv_qt(frame, self.qlabel_camera_view)
            self.qlabel_camera_view.setPixmap(pixmap)
            self.qlabel_camera_view.repaint()"""

    def test(self):  # test function for checking new signals
        print('Test was called', flush=True)

    def filter_nt_keys_combo(self):  # used to simplify the nt key list combo box entries
        if self.sorted_tree is not None:
            self.qcombobox_nt_keys.clear()
            filter = self.qt_text_entry_filter.toPlainText()
            filtered_keys = [key for key in self.sorted_tree if filter in key]
            self.qcombobox_nt_keys.addItems(filtered_keys)

    def update_key(self):  # used to modify an nt key value from the TUNING tab
        key = self.qcombobox_nt_keys.currentText()
        entry = self.ntinst.getEntry(key)
        entry_type = entry.getType()
        new_val = self.qt_text_new_value.toPlainText()
        print(f'Update key was called on {key}, which is a {entry_type}.  Setting it to {new_val}', flush=True)
        try:
            #t = QtWidgets.QPlainTextEdit()
            if entry_type == NetworkTableType.kDouble:
                new_val = float(new_val)
                entry.setDouble(new_val)
            elif entry_type == NetworkTableType.kString:
                entry.setString(new_val)
            elif entry_type == NetworkTableType.kBoolean:
                new_val = eval(new_val)
                entry.setBoolean(new_val)
            else:
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: {key} type {entry_type} not in [double, bool, string]')
        except Exception as e:
            self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Error occurred in setting {key} - {e}')
        self.qt_text_new_value.clear()
        self.refresh_tree()


    # update the autonomous routines - ToDo make this general for any chooser (pass the widget to currentTextChanged)
    def update_routines(self, text):
        key = self.widget_dict['qcombobox_autonomous_routines']['selected']
        self.ntinst.getEntry(key).setString(text)
        self.ntinst.flush()
        # print(f'Set NT value to {text}', flush=True)

    # tie commands to clicking labels - had to promote the labels to a class that has a click
    # Todo - make the commands dictionary and this use the same code - it's a bit redundant
    def label_click(self, label):
        # print(f"Running command to {label} {self.widget_dict[label]['command']}")
        toggled_state = not self.widget_dict[label]['command_entry'].getBoolean(True)
        print(f'You clicked {label} whose command is currently {not toggled_state}.  Firing command at {datetime.today().strftime("%H:%M:%S")} ...', flush=True)
        self.widget_dict[label]['command_entry'].setBoolean(toggled_state)

    # ------------------- INITIALIZING WIDGETS --------------------------
    # set up appropriate entries for all the widgets we care about
    def initialize_widgets(self):

        self.widget_dict = {
            # GUI UPDATES
        'drive_pose': {'widget': None, 'nt': '/SmartDashboard/drive_pose', 'command': None},
        'quest_pose': {'widget': None, 'nt': '/SmartDashboard/QUEST_POSE', 'command': None},
        'qcombobox_autonomous_routines': {'widget':self.qcombobox_autonomous_routines, 'nt': r'/SmartDashboard/autonomous routines/options', 'command':None, 'selected': r'/SmartDashboard/autonomous routines/selected'},
        'qlabel_nt_connected': {'widget': self.qlabel_nt_connected, 'nt': None, 'command': None},
        'qlabel_matchtime': {'widget': self.qlabel_matchtime, 'nt': '/SmartDashboard/match_time', 'command': None},
        'qlabel_alliance_indicator': {'widget': self.qlabel_alliance_indicator, 'nt': '/FMSInfo/IsRedAlliance', 'command': None,
                                            'style_on': "border: 7px; border-radius: 7px; background-color:rgb(225, 0, 0); color:rgb(200, 200, 200);",
                                            'style_off': "border: 7px; border-radius: 7px; background-color:rgb(0, 0, 225); color:rgb(200, 200, 200);"},
        'qlabel_camera_view': {'widget': self.qlabel_camera_view, 'nt': None, 'command': None},  # does this do anything? - can't remember
        'qlabel_arducam_high_indicator': {'widget': self.qlabel_arducam_high_indicator, 'nt': None, 'command': None},
        'qlabel_logitech_reef_indicator': {'widget': self.qlabel_logitech_reef_indicator, 'nt': None, 'command': None},
        'qlabel_genius_low_indicator': {'widget': self.qlabel_genius_low_indicator, 'nt': None, 'command': None},
        'qlabel_arducam_back_indicator': {'widget': self.qlabel_arducam_back_indicator, 'nt': None, 'command': None},

            # COMMANDS
        'qlabel_elevator_top_indicator': {'widget': self.qlabel_elevator_top_indicator, 'nt': '/SmartDashboard/MoveElevatorTop/running', 'command': '/SmartDashboard/MoveElevatorTop/running'},
        'qlabel_elevator_up_indicator': {'widget': self.qlabel_elevator_up_indicator, 'nt': '/SmartDashboard/MoveElevatorUp/running', 'command': '/SmartDashboard/MoveElevatorUp/running'},
        'qlabel_elevator_down_indicator': {'widget': self.qlabel_elevator_down_indicator, 'nt': '/SmartDashboard/MoveElevatorDown/running', 'command': '/SmartDashboard/MoveElevatorDown/running'},
        'qlabel_pivot_up_indicator': {'widget': self.qlabel_pivot_up_indicator, 'nt': '/SmartDashboard/MovePivotUp/running', 'command': '/SmartDashboard/MovePivotUp/running'},
        'qlabel_pivot_down_indicator': {'widget': self.qlabel_pivot_down_indicator, 'nt': '/SmartDashboard/MovePivotDown/running', 'command': '/SmartDashboard/MovePivotDown/running'},
        'qlabel_wrist_up_indicator': {'widget': self.qlabel_wrist_up_indicator, 'nt': '/SmartDashboard/MoveWristUp/running', 'command': '/SmartDashboard/MoveWristUp/running'},
        'qlabel_wrist_down_indicator': {'widget': self.qlabel_wrist_down_indicator, 'nt': '/SmartDashboard/MoveWristDown/running', 'command': '/SmartDashboard/MoveWristDown/running'},
        'qlabel_intake_on_indicator': {'widget': self.qlabel_intake_on_indicator, 'nt': '/SmartDashboard/IntakeOn/running', 'command': '/SmartDashboard/IntakeOn/running'},
        'qlabel_intake_off_indicator': {'widget': self.qlabel_intake_off_indicator, 'nt': '/SmartDashboard/IntakeOff/running','command': '/SmartDashboard/IntakeOff/running'},
        'qlabel_intake_reverse_indicator': {'widget': self.qlabel_intake_reverse_indicator, 'nt': '/SmartDashboard/IntakeReverse/running','command': '/SmartDashboard/IntakeReverse/running'},
        'qlabel_climber_down_indicator': {'widget': self.qlabel_climber_down_indicator, 'nt': '/SmartDashboard/Move climber down/running', 'command': '/SmartDashboard/Move climber down/running'},
        'qlabel_climber_up_indicator': {'widget': self.qlabel_climber_up_indicator, 'nt': '/SmartDashboard/Move climber up/running', 'command': '/SmartDashboard/Move climber up/running'},

        'qlabel_stow_indicator': {'widget': self.qlabel_stow_indicator, 'nt': '/SmartDashboard/GoToStow/running', 'command': '/SmartDashboard/GoToStow/running'},
        'qlabel_score_indicator': {'widget': self.qlabel_score_indicator, 'nt': '/SmartDashboard/Score/running', 'command': '/SmartDashboard/Score/running', 'flash':True},
        'qlabel_l1_indicator': {'widget': self.qlabel_l1_indicator, 'nt': '/SmartDashboard/GoToL1/running', 'command': '/SmartDashboard/GoToL1/running'},
        'qlabel_l2_indicator': {'widget': self.qlabel_l2_indicator, 'nt': '/SmartDashboard/GoToL2/running', 'command': '/SmartDashboard/GoToL2/running'},
        'qlabel_l3_indicator': {'widget': self.qlabel_l3_indicator, 'nt': '/SmartDashboard/GoToL3/running', 'command': '/SmartDashboard/GoToL3/running'},
        'qlabel_l4_indicator': {'widget': self.qlabel_l4_indicator, 'nt': '/SmartDashboard/GoToL4/running', 'command': '/SmartDashboard/GoToL4/running'},
        'qlabel_can_report_indicator': {'widget': self.qlabel_can_report_indicator, 'nt': '/SmartDashboard/CANStatus/running', 'command': '/SmartDashboard/CANStatus/running'},
        'qlabel_reset_flex_indicator': {'widget': self.qlabel_reset_flex_indicator, 'nt': '/SmartDashboard/ResetFlex/running', 'command': '/SmartDashboard/ResetFlex/running'},
        'qlabel_elevator_shift_up_indicator': {'widget': self.qlabel_elevator_shift_up_indicator, 'nt': '/SmartDashboard/CalElevatorUp/running', 'command': '/SmartDashboard/CalElevatorUp/running'},
        'qlabel_elevator_shift_down_indicator': {'widget': self.qlabel_elevator_shift_down_indicator, 'nt': '/SmartDashboard/CalElevatorDown/running', 'command': '/SmartDashboard/CalElevatorDown/running'},
        'qlabel_cal_wrist_indicator': {'widget': self.qlabel_cal_wrist_indicator, 'nt': '/SmartDashboard/RecalWrist/running', 'command': '/SmartDashboard/RecalWrist/running'},
        'qlabel_cal_wrist_up_indicator': {'widget': self.qlabel_cal_wrist_up_indicator, 'nt': '/SmartDashboard/CalWristUp/running', 'command': '/SmartDashboard/CalWristUp/running'},
        'qlabel_cal_wrist_down_indicator': {'widget': self.qlabel_cal_wrist_down_indicator, 'nt': '/SmartDashboard/CalWristDown/running', 'command': '/SmartDashboard/CalWristDown/running'},


        'qlabel_questnav_heartbeat_indicator': {'widget': self.qlabel_questnav_heartbeat_indicator, 'nt': '/SmartDashboard/QUEST_CONNECTED', 'command': None},
        'qlabel_questnav_inbounds_indicator': {'widget': self.qlabel_questnav_inbounds_indicator, 'nt': '/SmartDashboard/QUEST_POSE_ACCEPTED', 'command': None},
        'qlabel_questnav_tracking_indicator': {'widget': self.qlabel_questnav_tracking_indicator, 'nt': '/SmartDashboard/QUEST_TRACKING', 'command': None},
        'qlabel_questnav_sync_toggle_indicator': {'widget': self.qlabel_questnav_sync_toggle_indicator, 'nt': '/SmartDashboard/questnav_synched', 'command': '/SmartDashboard/QuestSyncToggle/running'},
        'qlabel_questnav_reset_indicator': {'widget': self.qlabel_questnav_reset_indicator, 'nt': '/SmartDashboard/QuestResetOdometry/running', 'command': '/SmartDashboard/QuestResetOdometry/running'},
        'qlabel_questnav_enabled_toggle_indicator': {'widget': self.qlabel_questnav_enabled_toggle_indicator, 'nt': '/SmartDashboard/questnav_in_use', 'command': '/SmartDashboard/QuestEnableToggle/running',
                                        # 'style_on': "border: 7px; border-radius: 7px; background-color:rgb(0, 220, 220); color:rgb(250, 250, 250);",
                                        # 'style_off': "border: 7px; border-radius: 7px; background-color:rgb(127, 127, 127); color:rgb(0, 0, 0);"},
                                             },

        'qlabel_arducam_high_target_indicator': {'widget': self.qlabel_arducam_high_target_indicator, 'nt': '/SmartDashboard/arducam_high_targets_exist', 'command': None},
        'qlabel_arducam_back_target_indicator': {'widget': self.qlabel_arducam_back_target_indicator, 'nt': '/SmartDashboard/arducam_back_targets_exist', 'command': None},
        'qlabel_logitech_reef_target_indicator': {'widget': self.qlabel_logitech_reef_target_indicator, 'nt': '/SmartDashboard/logitech_reef_targets_exist', 'command': None},
        'qlabel_genius_low_target_indicator': {'widget': self.qlabel_genius_low_target_indicator, 'nt': '/SmartDashboard/genius_low_targets_exist', 'command': None},
        'qlabel_photoncam_target_indicator': {'widget': self.qlabel_photoncam_target_indicator, 'nt': '/SmartDashboard/photoncam_targets_exist', 'command': None},
        'qlabel_pose_indicator': {'widget': self.qlabel_pose_indicator, 'nt': None, 'command': None,},
        'qlabel_game_piece_indicator': {'widget': self.qlabel_game_piece_indicator, 'nt': '/SmartDashboard/gamepiece_present', 'command': '/SmartDashboard/LedToggle/running',
                                        'style_on': "border: 7px; border-radius: 7px; background-color:rgb(0, 220, 220); color:rgb(250, 250, 250);",
                                        'style_off': "border: 7px; border-radius: 7px; background-color:rgb(127, 127, 127); color:rgb(0, 0, 0);"},
            # UNFINISHED for 2025
        'qlabel_navx_reset_indicator': {'widget': self.qlabel_navx_reset_indicator, 'nt': '/SmartDashboard/GyroReset/running', 'command': '/SmartDashboard/GyroReset/running'},
        # 'qlabel_orange_target_indicator': {'widget': self.qlabel_orange_target_indicator, 'nt': '/SmartDashboard/orange_targets_exist', 'command': None},

        'qlabel_position_indicator': {'widget': self.qlabel_position_indicator, 'nt': '/SmartDashboard/_target', 'command': None},

            # NUMERIC INDICATORS - LCDS should be phased out since they are a legacy item (and not particularly cool anyway)
        'qlcd_navx_heading': {'widget': self.qlcd_navx_heading, 'nt': '/SmartDashboard/_navx', 'command': None},
        'qlcd_elevator_height': {'widget': self.qlcd_elevator_height, 'nt': '/SmartDashboard/elevator_spark_pos', 'command': None},
        'qlcd_pivot_angle': {'widget': self.qlcd_pivot_angle, 'nt': '/SmartDashboard/profiled_pivot_spark_angle', 'command': None},
        'qlcd_wrist_angle': {'widget' :self.qlcd_wrist_angle, 'nt': '/SmartDashboard/wrist relative encoder, degrees', 'command': None},
        'qlcd_intake_speed': {'widget': self.qlcd_intake_speed, 'nt': '/SmartDashboard/intake_output', 'command': None},
        'qlcd_climber_position': {'widget': self.qlcd_climber_position, 'nt': '/SmartDashboard/climber_spark_angle', 'command': None},
        'qlabel_pdh_voltage_monitor': {'widget': self.qlabel_pdh_voltage_monitor, 'nt': '/SmartDashboard/_pdh_voltage', 'command': None},
        'qlabel_pdh_current_monitor': {'widget': self.qlabel_pdh_current_monitor, 'nt': '/SmartDashboard/_pdh_current', 'command': None},

            # LEFTOVER TO SORT FROM 2023
        #'qlabel_align_to_target_indicator': {'widget': self.qlabel_align_to_target_indicator, 'nt': '/SmartDashboard/AutoSetupScore/running', 'command': '/SmartDashboard/AutoSetupScore/running'},
        #'qlabel_arm_calibration_indicator': {'widget': self.qlabel_arm_calibration_indicator, 'nt': '/SmartDashboard/ArmCalibration/running', 'command': '/SmartDashboard/ArmCalibration/running'},
        # 'qlabel_upper_pickup_indicator': {'widget': self.qlabel_upper_pickup_indicator, 'nt': '/SmartDashboard/UpperSubstationPickup/running', 'command': '/SmartDashboard/UpperSubstationPickup/running'},
        'hub_targets': {'widget': None, 'nt': '/arducam_high//orange/targets', 'command': None},
        'hub_rotation': {'widget': None, 'nt': '/arducam_high//orange/rotation', 'command': None},
        'hub_distance': {'widget': None, 'nt': '/arducam_high//orange/distance', 'command': None},

        }

        # get all the entries and add them to the dictionary
        for key, d in self.widget_dict.items():
            if d['nt'] is not None:
                d.update({'entry': self.ntinst.getEntry(d['nt'])})
            else:
                d.update({'entry': None})
            if d['command'] is not None:
                d.update({'command_entry': self.ntinst.getEntry(d['command'])})
                # assign a command clicked to it
                d['widget'].clicked.connect(lambda label=key: self.label_click(label))
            else:
                d.update({'command_entry': None})
            #print(f'Widget {key}: {d}')

        for key in self.camera_dict.keys():
            self.qcombobox_cameras.addItem(key)

    def update_widgets(self):
        """ Main function which is looped to update the GUI with NT values"""
        # these are the styles for the gui labels
        style_on = "border: 7px; border-radius: 7px; background-color:rgb(80, 235, 0); color:rgb(0, 0, 0);"  # bright green, black letters
        style_off = "border: 7px; border-radius: 7px; background-color:rgb(220, 0, 0); color:rgb(200, 200, 200);"  # bright red, dull white letters (also flash off)
        style_high = "border: 7px; border-radius: 15px; background-color:rgb(80, 235, 0); color:rgb(0, 0, 0);"  # match time -> regular game, green
        style_low = "border: 7px; border-radius: 15px; background-color:rgb(0, 20, 255); color:rgb(255, 255, 255);"  # match time endgame - blue
        style_flash_on = "border: 7px; border-radius: 7px; background-color:rgb(0, 0, 0); color:rgb(255, 255, 255);"  # flashing on step 1 - black with thite letters
        style_flash_off = "border: 7px; border-radius: 7px; background-color:rgb(0, 20, 255); color:rgb(255, 255, 255);"  # flashing on step 2 - blue with white letters
        style_flash = style_flash_on if self.counter % 30 < 15 else style_flash_off  # get things to blink

        # update the connection indicator
        style_disconnected = "border: 7px; border-radius: 7px; background-color:rgb(180, 180, 180); color:rgb(0, 0, 0);"
        style = style_on if self.ntinst.isConnected() else style_disconnected
        self.widget_dict['qlabel_nt_connected']['widget'].setStyleSheet(style)

        # update all labels tied to NT entries
        for key, d in self.widget_dict.items():
            if d['entry'] is not None:
                if 'indicator' in key:
                    #  print(f'Indicator: {key}')
                    # allow for a custom style in the widget
                    if 'style_on' in d.keys():  # override the default styles for a specific widget
                        style = d['style_on'] if d['entry'].getBoolean(False) else d['style_off']
                    elif 'flash' in d.keys():  # flashing behaviour - not compatible with a custom style override
                        style = style_flash if (d['flash'] and d['entry'].getBoolean(False)) else style_off
                    else:  # default style behaviour
                        style = style_on if d['entry'].getBoolean(False) else style_off
                    d['widget'].setStyleSheet(style)
                elif 'lcd' in key:  # old-style LCDs
                    #  print(f'LCD: {key}')
                    value = int(d['entry'].getDouble(0))
                    d['widget'].display(str(value))
                elif 'monitor' in key:  # labels acting as numeric monitors
                    # voltage needs a decimal, current does not - handled in the WarningLabel class
                    # value = f"{d['entry'].getDouble(0):0.1f}" if 'volt' in key else f"{int(d['entry'].getDouble(0)):3d}"
                    # d['widget'].setText(f'{value}')
                    value = d['entry'].getDouble(0)
                    d['widget'].set_value(value)
                elif 'combo' in key:  # ToDo: need a simpler way to update the combo boxes
                    new_list = d['entry'].getStringArray([])
                    if new_list != self.autonomous_list:
                        d['widget'].blockSignals(True)  # don't call updates on this one
                        d['widget'].clear()
                        d['widget'].addItems(new_list)
                        d['widget'].blockSignals(False)
                        self.autonomous_list = new_list
                    selected_routine = self.ntinst.getEntry(d['selected']).getString('')
                    if selected_routine != d['widget'].currentText():
                        d['widget'].blockSignals(True)  # don't call updates on this one
                        d['widget'].setCurrentText(selected_routine)
                        d['widget'].blockSignals(False)
                elif 'time' in key:
                    match_time = d['entry'].getDouble(0)
                    d['widget'].setText(str(int(match_time)))
                    if match_time < 30:
                        d['widget'].setText(f'* {int(match_time)} *')
                        d['widget'].setStyleSheet(style_flash)
                    else:
                        d['widget'].setText(str(int(match_time)))
                        d['widget'].setStyleSheet(style_high)
                else:
                    pass
                    # print(f'Skipping: {key}')

        # update the commands list
        green = QtGui.QColor(227, 255, 227)
        white = QtGui.QColor(255, 255, 255)
        for ix, (key, d) in enumerate(self.command_dict.items()):
            bg_color = green if d['entry'].getBoolean(True) else white
            self.qlistwidget_commands.item(ix).setBackground(bg_color)

        # update the ball position on the hub target image - left over from 2022 that I need to get rid of
        hub_targets = self.widget_dict['hub_targets']['entry'].getDouble(0)
        hub_rotation = self.widget_dict['hub_rotation']['entry'].getDouble(0) - 5
        hub_distance = self.widget_dict['hub_distance']['entry'].getDouble(0)
        # print(f'hub_targets: {hub_targets} {hub_rotation:2.2f} {hub_distance:2.2f}', end='\r')

        if hub_targets > 0:
            # shooter_rpm = self.widget_dict['qlcd_shooter_rpm']['entry'].getDouble(0)
            shooter_rpm = 2000
            shooter_distance = shooter_rpm * 0.00075
            center_offset = shooter_distance * -np.sin(hub_rotation * 3.14159 / 180)
            hub_x = 205 + center_offset * (380 / 1.2) # 380 px per 1.2 m
            hub_y = 190

            self.qlabel_ball.move(int(hub_x), int(hub_y))
            if self.qlabel_ball.isHidden():
                self.qlabel_ball.show()
        else:
            #self.qlabel_ball.move(0, 0)
            if not self.qlabel_ball.isHidden():
                self.qlabel_ball.hide()

        # update the pose - some extra magic to rotate the pixmap and stretch it correctly due to the rotation
        width, height = self.qgroupbox_field.width(), self.qgroupbox_field.height()
        bot_width, bot_height = 41, 41 # self.qlabel_robot.width(), self.qlabel_robot.height()
        x_lim, y_lim = 17.6, 8.2  # 16.4, 8.2
        drive_pose = self.widget_dict['drive_pose']['entry'].getDoubleArray([0, 0, 0])
        # not sure how to get labels to justify strings properly - not true to total string length somehow
        x_pad = 1 if drive_pose[0] < 10 else 0
        theta_pad = 0  # must be a quicker way to do this
        theta_pad = theta_pad + 1 if drive_pose[2] < 0 else theta_pad
        theta_pad = theta_pad + 1 if abs(drive_pose[2]) < 100 else theta_pad
        theta_pad = theta_pad + 1 if abs(drive_pose[2]) < 10 else theta_pad
        pose_msg = f'POSE\n{" " * x_pad}{drive_pose[0]:>5.2f}m {drive_pose[1]:>4.2f}m {" " * theta_pad}{drive_pose[2]:>4.0f}°'
        self.qlabel_pose_indicator.setText(pose_msg)

        # quick quest pose hack
        quest_pose = self.widget_dict['quest_pose']['entry'].getString('No quest pose')
        pattern = r"x=(?P<x>-?\d+\.\d+).*y=(?P<y>-?\d+\.\d+).*Rotation2d\((?P<rotation>-?\d+\.\d+)\)"
        match = re.search(pattern, quest_pose)
        if match:
            quest_x, quest_y, quest_rot = float(match.group("x")), float(match.group("y")), float(match.group("rotation"))
            quest_rot = math.degrees(quest_rot)
        else:
            quest_x, quest_y, quest_rot = -1, -1, -1
        pose_msg = f'QUEST POSE\n{" " * x_pad}{quest_x:>5.2f}m {quest_y:>4.2f}m {" " * theta_pad}{quest_rot:>4.0f}°'
        self.qlabel_quest_pose_indicator.setText(pose_msg)

        pixmap_rotated = self.robot_pixmap.transformed(QtGui.QTransform().rotate(90-drive_pose[2]), QtCore.Qt.TransformationMode.SmoothTransformation)
        new_size = int(41 * (1 + 0.41 * np.abs(np.sin(2 * drive_pose[2] * np.pi / 180.0))))
        self.qlabel_robot.resize(new_size, new_size)  # take account of rotation shrinkage
        self.qlabel_robot.setPixmap(pixmap_rotated)  # this does rotate successfully
        # self.qlabel_robot.move(int(-bot_width / 2 + width * drive_pose[0] / x_lim), int(-bot_height / 2 + height * (1 - drive_pose[1] / y_lim)))
        self.qlabel_robot.move(int(-new_size / 2 + width * drive_pose[0] / x_lim), int(-new_size / 2 + height * (1 - drive_pose[1] / y_lim)))

        # duplicate for the quest odometry 20250923 CJH
        quest_pixmap_rotated = self.quest_pixmap.transformed(QtGui.QTransform().rotate(90-quest_rot), QtCore.Qt.TransformationMode.SmoothTransformation)
        quest_new_size = int(41 * (1 + 0.41 * np.abs(np.sin(2 * quest_rot * np.pi / 180.0))))
        self.qlabel_quest.resize(quest_new_size, quest_new_size)
        self.qlabel_quest.setPixmap(quest_pixmap_rotated)
        self.qlabel_quest.move(int(-new_size/2 + width * quest_x / x_lim ), int(-new_size/2 + height * (1 - quest_y / y_lim)))


        ## print(f'Pose X:{drive_pose[0]:2.2f} Pose Y:{drive_pose[1]:2.2f} Pose R:{drive_pose[2]:2.2f}', end='\r', flush=True)

        # --------------  CAMERA STATUS INDICATORS  ---------------
        # set indicators to green if their timestamp matches the timestamp of the robot, tally the connections
        # todo - see if these are updated from the widget dictionary loop, and if so, maybe remove them?
        allowed_delay = 0.5  # how long before we call a camera dead
        timestamp = self.robot_timestamp_entry.getDouble(1)

        # look for a disconnect - just in arducam_high for now since that's the one we watch
        cam_keys = [key for key in self.camera_dict.keys() if 'IS_ALIVE' in self.camera_dict[key]]  # the ones we care about

        if self.thread is not None:  # camera stream view has been started
            if self.thread.isRunning():  # camera is on
                for key in cam_keys:
                    if self.camera_dict[key]['IS_ALIVE'] and timestamp - self.camera_dict[key]['TIMESTAMP_ENTRY'].getDouble(-1) > allowed_delay:  # cam died
                        self.camera_dict[key]['IS_ALIVE'] = False
                        self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Detected loss of {self.camera_dict[key]["NICKNAME"]} - information only')
                        if key == 'LogitechReef':  # TODO - figure out if we only toggle on one specific camera, or ANY live camera DONT want to hardcode this
                            self.toggle_camera_thread()

            else:  # we started the camera but the thread is not running
                restart_thread = False
                for key in cam_keys:
                    if not self.camera_dict[key]['IS_ALIVE'] and timestamp - self.camera_dict[key]['TIMESTAMP_ENTRY'].getDouble(-1) < allowed_delay:
                        self.camera_dict[key]['IS_ALIVE'] = True
                        self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Detected {self.camera_dict[key]["NICKNAME"]}')
                        restart_thread = True
                if restart_thread:
                    self.toggle_camera_thread()
                    self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: - RESTARTING camera thread')

        # check for camera presence based on timestamp delay, update widgets accordingly - cleaner dictionary based approach we can loop over
        for key in cam_keys:
            self.camera_dict[key]['CONNECTIONS'] = int(self.camera_dict[key]['CONNECTIONS_ENTRY'].getDouble(0))
            self.camera_dict[key]['IS_ALIVE'] = timestamp - self.camera_dict[key]['TIMESTAMP_ENTRY'].getDouble(-1) < allowed_delay  # redundant from above?
            current_style = style_on if self.camera_dict[key]['IS_ALIVE'] else style_off
            self.camera_dict[key]['INDICATOR'].setStyleSheet(current_style)
            self.camera_dict[key]['INDICATOR'].setText(f'{self.camera_dict[key]["NICKNAME"]}: {self.camera_dict[key]["CONNECTIONS"]:2d}')


        # --------------  SPEAKER POSITION CALCULATIONS  ---------------
        k_blue_speaker = [0, 5.55, 180]  # (x, y, rotation)
        k_red_speaker = [16.5, 5.555, 0]  # (x, y, rotation)
        if self.widget_dict['qlabel_alliance_indicator']['entry'].getBoolean(False):
            translation_origin_to_speaker = geo.Translation2d(k_red_speaker[0], k_red_speaker[1])
        else:
            translation_origin_to_speaker = geo.Translation2d(k_blue_speaker[0], k_blue_speaker[1])
        translation_origin_to_robot = geo.Translation2d(drive_pose[0], drive_pose[1])
        translation_robot_to_speaker = translation_origin_to_speaker - translation_origin_to_robot
        desired_angle = translation_robot_to_speaker.angle().rotateBy(geo.Rotation2d(np.radians(180)))  # shooting back
        angle_to_speaker = drive_pose[2] - desired_angle.degrees()
        angle_tolerance = 10

        # update the 2024 shot distance indicator
        best_distance = 1.7  # is 2m our best distance?
        distance_tolerance = 0.4
        speaker_blue = (0, 5.56)
        speaker_red = (16.54, 5.56)
        speaker = speaker_red if self.widget_dict['qlabel_alliance_indicator']['entry'].getBoolean(False) else speaker_blue
        shot_distance = np.sqrt((speaker[0]-drive_pose[0])**2 + (speaker[1]-drive_pose[1])**2)  # robot to speaker center
        if shot_distance <= best_distance - distance_tolerance:  # too close
            shot_style = "border: 7px; border-radius: 7px; background-color:rgb(225, 0, 0); color:rgb(200, 200, 200);"  # bright red
        elif shot_distance > best_distance - distance_tolerance and shot_distance < best_distance + distance_tolerance:  # just right?
            grey_val = int(225 * np.abs(best_distance-shot_distance))  # make it a more saturated green
            # blink if the angle is good
            if np.abs(angle_to_speaker) < angle_tolerance:
                text_color = '(0,0,0)' if self.counter % 10 < 5 else '(255,255,255)'  # make it blink
                border_color = 'solid blue' if self.counter % 10 < 5 else 'solid black'  # make it blink
                border_size = 6
            else:
                text_color = '(200, 200, 200)'  # still black if we don't have the shot
                border_color = 'solid red'
                border_size = 8
            shot_style = f"border: {border_size}px {border_color}; border-radius: 7px; background-color:rgb({grey_val}, {int(225-grey_val)}, {grey_val}); color:rgb{text_color};"
        elif shot_distance >= best_distance + distance_tolerance:
            shot_style = "border: 7px; border-radius: 7px; background-color:rgb(180, 180, 180); color:rgb(200, 200, 200);"
        else:
            shot_style = "border: 7px; border-radius: 7px; background-color:rgb(0, 0, 0); color:rgb(200, 200, 200);"
        self.qlabel_shot_distance: QtWidgets.QLabel
        # self.qlabel_shot_distance.setText(f'SHOT DIST\n{shot_distance:.1f}')  # updated below
        self.qlabel_shot_distance.setText(f'SHOT DIST\n{shot_distance:.1f}m  {int(angle_to_speaker):>+3d}°')
        self.qlabel_shot_distance.setStyleSheet(shot_style)

        # update the PDH measurements colors - all of this can actually be done in the WarningLabel class except the blinking
        # voltage = self.widget_dict['qlabel_pdh_voltage_monitor']['entry'].getDouble(0)
        # text_color = '(0,0,0)'
        # if voltage < 8:
        #     hue = 0  # red
        #     text_color = '(0,0,0)' if self.counter % 10 < 5 else '(255,255,255)'  # make it blink
        # elif voltage > 12:
        #     hue = 100  # green
        # else:
        #     hue = int(100 - 100 * (12 - voltage) / 4 )
        # voltage_style = f"border: 7px; border-radius: 7px; background-color:hsv({hue}, 240, 240); color:rgb{text_color};"
        # self.qlabel_pdh_voltage_monitor.setStyleSheet(voltage_style)
        #
        # current = self.widget_dict['qlabel_pdh_current_monitor']['entry'].getDouble(0)
        # if current > 160:
        #     hue = 0  # red
        # elif current < 60:
        #     hue = 100  # green
        # else:
        #     hue = max(0, min(100, 100 - int(current - 60)))  # lock the current hue between 0 (red) and 100 (green)
        # current_style = f"border: 7px; border-radius: 7px; background-color:hsv({hue}, 240, 240); color:rgb(0, 0, 0);"
        # self.qlabel_pdh_current_monitor.setStyleSheet(current_style)

        # update the 2024 arm configuration indicator
        config = self.widget_dict['qlabel_position_indicator']['entry'].getString('?')
        if config.upper() not in ['LOW_SHOOT', 'INTAKE']:  # these two positions drive under the stage
            # text_color = '(0,0,0)' if self.counter % 30 < 15 else '(255,255,255)'  # make it blink
            # postion_style = f"border: 7px; border-radius: 7px; background-color:rgb(220, 0, 0); color:rgb{text_color};"
            postion_style = style_on
        else:
            postion_style = style_on
        self.qlabel_position_indicator.setText(f'POS: {config.upper()}')
        self.qlabel_position_indicator.setStyleSheet(postion_style)

        self.counter += 1
        if self.counter % 80 == 0:  # display an FPS every 2s or so  REMEMBER THIS MAX IS SET BY THE STREAMER
            current_time = time.time()
            time_delta = current_time - self.previous_time
            frames = self.worker.frames if self.worker is not None else 0

            msg = f'Current Gui Updates/s : {100/(time_delta):.1f}, Camera updates/s : {(frames-self.previous_frames)/time_delta:.1f} (server enforced BW limit)'
            self.statusBar().showMessage(msg)
            self.previous_time = current_time
            self.previous_frames = frames

    def qt_tree_widget_nt_clicked(self, item):
        # send the clicked item from the tree to the filter for the nt selction combo box
        # print(f' Item clicked is: {item.data()}', flush=True)
        self.qt_text_entry_filter.clear()
        self.qt_text_entry_filter.setPlainText(item.data())

    def command_list_clicked(self, item):
        # shortcut where we click the command list, fire off (or end) the command
        cell_content = item.data()
        toggled_state = not self.command_dict[cell_content]['entry'].getBoolean(True)
        print(f'You clicked {cell_content} which is currently {not toggled_state}.  Firing command...', flush=True)
        self.command_dict[cell_content]['entry'].setBoolean(toggled_state)

    # -------------------  UPDATING NETWORK TABLES DISPLAY --------------------------
    def toggle_network_tables(self):
        # tree = QtWidgets.QTreeWidget
        if self.qt_tree_widget_nt.isHidden():
            self.refresh_tree()
            self.qt_tree_widget_nt.show()
        else:
            self.qt_tree_widget_nt.hide()

    def report_nt_status(self):
        id, ip = self.ntinst.getConnections()[0].remote_id, self.ntinst.getConnections()[0].remote_ip
        self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: NT status: id={id}, ip={ip}')

    def refresh_tree(self):
        """  Read networktables and update tree and combo widgets
        """
        self.connected = self.ntinst.isConnected()
        if self.connected:
            self.report_nt_status()
            self.qt_tree_widget_nt.clear()
            entries = self.ntinst.getEntries('/', types=0)
            self.sorted_tree = sorted([e.getName() for e in entries])

            # update the dropdown combo box with all keys
            self.filter_nt_keys_combo()
            # self.qcombobox_nt_keys.clear()
            # self.qcombobox_nt_keys.addItems(self.sorted_tree)

            # generate the dictionary - some magic I found on the internet
            nt_dict = {}
            levels = [s[1:].split('/') for s in self.sorted_tree]
            for path in levels:
                current_level = nt_dict
                for part in path:
                    if part not in current_level:
                        current_level[part] = {}
                    current_level = current_level[part]

            self.qlistwidget_commands.clear()
            for item in self.sorted_tree:
                # print(item)
                if 'running' in item:  # quick test of the list view for commands
                    # print(f'Command found: {item}')
                    command_name = item.split('/')[2]
                    self.qlistwidget_commands.addItem(command_name)
                    self.command_dict.update({command_name: {'nt':item, 'entry': self.ntinst.getEntry(item)}})

                entry_value = self.ntinst.getEntry(item).getValue()
                value = entry_value.value()
                age = int(time.time() - entry_value.last_change()/1E6)
                levels = item[1:].split('/')
                if len(levels) == 2:
                    nt_dict[levels[0]][levels[1]] = value, age
                elif len(levels) == 3:
                    nt_dict[levels[0]][levels[1]][levels[2]] = value, age
                elif len(levels) == 4:
                    nt_dict[levels[0]][levels[1]][levels[2]][levels[3]] = value, age

            self.fill_item(self.qt_tree_widget_nt.invisibleRootItem(), nt_dict)
            self.qt_tree_widget_nt.resizeColumnToContents(0)
            self.qt_tree_widget_nt.setColumnWidth(1, 100)
        else:
            self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Unable to connect to server')

    def keyPressEvent(self, a0):
        print(f"adding key {a0.text()} whose code is {a0.key()}")
        print(type(a0))
        self.keys_currently_pressed.append(a0.key())
        print(f"keys currently pressed: {self.keys_currently_pressed}")
        self.ntinst.getEntry("SmartDashboard/keys_pressed").setIntegerArray(self.keys_currently_pressed)
        self.ntinst.getEntry("SmartDashboard/key_pressed").setInteger(a0.key())

    def keyReleaseEvent(self, a0):
        # TODO: only release after it's been held for 1/50s so the robot can "catch" the keypress
        try:
            while a0.key() in self.keys_currently_pressed:
                # sometimes we get duplicates from clicking off the window while holding a key because it doesn't
                # catch the key being released
                print(f"attempting to remove key {a0.key()} ({a0.text()}) from list {self.keys_currently_pressed} ({[chr(i) for i in self.keys_currently_pressed if 32 <= i <= 126]})")
                self.keys_currently_pressed.remove(a0.key())
        except:
            pass
        print(f"keys currently pressed: {self.keys_currently_pressed}")
        self.ntinst.getEntry("SmartDashboard/key_pressed").setInteger(-999)
        self.ntinst.getEntry("SmartDashboard/keys_pressed").setIntegerArray(self.keys_currently_pressed)

    def focusOutEvent(self, a0):
        # clear because we probably don't want anything happening when we're not focused
        # and also to help with the duplicates when clicking off window issue described in keyReleaseEvent
        self.keys_currently_pressed = []
        print(f"keys currently pressed: {self.keys_currently_pressed}")



    # -------------------  HELPER FUNCTIONS FOR THE DICTIONARIES AND WIDGETS --------------------------
    def eventFilter(self, obj, event):
        if (obj is self.qt_text_entry_filter or obj is self.qt_text_new_value) and event.type() == QEvent.Type.KeyPress:
            if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                return True
        return super().eventFilter(obj, event)

    def depth(self, d):
        if isinstance(d, dict):
            return 1 + (max(map(self.depth, d.values())) if d else 0)
        return 0

    ## helper functions for filling the NT tree widget
    def fill_item(self, widget, value):
        if value is None:
            # keep recursing until nothing is passed
            return
        elif isinstance(value, dict) and self.depth(value) > 1:
            for key, val in sorted(value.items()):
                self.new_item(parent=widget, text=str(key), val=val)
        elif isinstance(value, dict):
            # now we actually add the bottom level item
            #self.new_item(parent=widget, text=str(value))
            for key, val in sorted(value.items()):
                child = QtWidgets.QTreeWidgetItem([str(key), str(val[0]), str(val[1])])
                self.fill_item(child, val)
                widget.addChild(child)
        else:
            pass

    def new_item(self, parent, text, val=None):
        if val is None:
            child = QtWidgets.QTreeWidgetItem([text, 'noval'])
        else:
            if isinstance(val,dict):
                child = QtWidgets.QTreeWidgetItem([text])
            else:
                child = QtWidgets.QTreeWidgetItem([text, str(val[0]), str(val[1])])
        self.fill_item(child, val)
        parent.addChild(child)
        child.setExpanded(True)



# -------------------  MAIN --------------------------
if __name__ == "__main__":
    import sys

    # attempt to set high dpi scaling if it is detected - possible fix for high-def laptop and destop displays
    # if hasattr(QtCore.Qt, 'AA_EnableHighDpiScaling'):
    #     QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    if hasattr(QtCore.Qt, 'AA_UseHighDpiPixmaps'):
        QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    # compensate for dpi scaling way up above with the setAttribute calls - don't really need this now
    screen = app.screens()[0]
    dpi_logical = int(screen.logicalDotsPerInchX())
    if dpi_logical > 96:  # 150% scaling on AVIT North, vs 96 for unscaled
        print(f"We're on a scaled screen: logical dpi is {dpi_logical}")
    else:
        print(f"We're not on a scaled screen: logical dpi is {dpi_logical}")

    ui = Ui()

    try:
        # sys.exit(app.exec_())  # pyqt5 way
        sys.exit(app.exec())
    except SystemExit:
        print('Still has garbage collection issues. Closing.')
