# pyqt example for teaching GUI development for FRC dashboard
# make sure to pip install pyqt6 pyqt6-tools

# print(f'Loading Modules ...', flush=True)
import os
import cv2
import time
from datetime import datetime
from pathlib import Path

from PyQt6 import QtCore, QtGui, QtWidgets, uic
from PyQt6.QtCore import Qt, QTimer, QEvent
from PyQt6.QtWidgets import QGraphicsOpacityEffect

# logical chunking of the gui's functional components
from config import WIDGET_CONFIG, CAMERA_CONFIG
from nt_manager import NTManager
from camera_manager import CameraManager
from ui_updater import UIUpdater
from nt_tree import NTTreeManager  # eventually i will make this work again (my own NT tree)

# I don't think I need these here - but load ui may need them?
#from widgets.clickable_qlabel import ClickableQLabel
#from widgets.warning_label import WarningLabel


os.environ["OPENCV_LOG_LEVEL"] = "DEBUG"  # Options: INFO, WARNING, ERROR, DEBUG
print("[DEBUG] OpenCV version:", cv2.__version__)
cv2.setNumThreads(4)  # Disable multithreading
cv2.ocl.setUseOpenCL(True)

#print(f'Initializing GUI ...', flush=True)

class Ui(QtWidgets.QMainWindow):
    root_dir = Path('.').absolute()
    png_dir = root_dir / 'png'
    save_dir = root_dir / 'save'

    def __init__(self):
        super(Ui, self).__init__()
        uic.loadUi('layout_2025.ui', self)

        self.nt_manager = NTManager(self)
        self.camera_manager = CameraManager(self)
        self.ui_updater = UIUpdater(self)
        self.nt_tree_manager = NTTreeManager(self)
        self.ntinst = self.nt_manager.ntinst  # Keep for compatibility for now

        self.sorted_tree = None
        self.autonomous_list = []

        self.refresh_time = 50
        self.previous_frames = 0
        self.widget_dict = {}
        self.command_dict = {}
        self.camera_enabled = False
        self.worker = None
        self.thread = None

        # Build the runtime dictionaries from the static config
        self.widget_dict = self.build_widget_dict()
        self.camera_dict = self.build_camera_dict()

        self.robot_timestamp_entry = self.nt_manager.getEntry('/SmartDashboard/_timestamp')

        self.qlabel_pdh_voltage_monitor.update_settings(min_val=8, max_val=12, red_high=False, display_float=True)
        self.qlabel_pdh_current_monitor.update_settings(min_val=60, max_val=160, red_high=True, display_float=False)

        self.initialize_widgets()

        # Connections for the NT Tree are now managed by the NTTreeManager
        self.qaction_show_hide.triggered.connect(self.nt_tree_manager.toggle_network_tables)
        self.qaction_refresh.triggered.connect(self.nt_tree_manager.refresh_tree)
        self.qlistwidget_commands.clicked.connect(self.nt_tree_manager.command_list_clicked)
        self.qcombobox_nt_keys.currentTextChanged.connect(self.nt_tree_manager.update_selected_key)
        self.qt_tree_widget_nt.clicked.connect(self.nt_tree_manager.qt_tree_widget_nt_clicked)
        self.qt_text_entry_filter.textChanged.connect(self.nt_tree_manager.filter_nt_keys_combo)
        self.qt_button_set_key.clicked.connect(self.nt_tree_manager.update_key)

        # Other UI connections
        self.qcombobox_autonomous_routines.currentTextChanged.connect(self.update_routines)
        self.qt_text_entry_filter.installEventFilter(self)
        self.qt_text_new_value.installEventFilter(self)

        self.robot_pixmap = QtGui.QPixmap("png\\blockhead.png")
        self.quest_pixmap = QtGui.QPixmap("png\\quest.png")
        opacity_effect = QGraphicsOpacityEffect()
        opacity_effect.setOpacity(0.5)
        self.qlabel_quest.setGraphicsEffect(opacity_effect)
        self.qlabel_robot.raise_()

        self.qt_button_swap_sim.clicked.connect(self.nt_manager.increment_server)
        self.qt_button_reconnect.clicked.connect(self.nt_manager.reconnect)
        self.qt_button_camera_enable.clicked.connect(self.camera_manager.toggle_camera_thread)

        self.qt_tree_widget_nt.hide()

        self.keys_currently_pressed = []
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

        self.show()

        self.counter = 1
        self.previous_time = time.time()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.ui_updater.update_widgets)
        self.timer.start(self.refresh_time)

    def build_widget_dict(self):
        """Builds the runtime widget dictionary from the static configuration."""
        widget_dict = {}
        for key, config in WIDGET_CONFIG.items():
            new_entry = config.copy()
            widget_name = config.get('widget_name')
            if widget_name:
                new_entry['widget'] = getattr(self, widget_name, None)
            
            nt_topic = config.get('nt_topic')
            if nt_topic:
                new_entry['entry'] = self.nt_manager.getEntry(nt_topic)

            command_topic = config.get('command_topic')
            if command_topic:
                new_entry['command_entry'] = self.nt_manager.getEntry(command_topic)

            selected_topic = config.get('selected_topic')
            if 'selected_topic' in new_entry:
                new_entry['selected'] = self.nt_manager.getEntry(selected_topic)
                # print(f'{key} has selected topic: {selected_topic} with value {new_entry["selected"].getStringArray([])}')

            widget_dict[key] = new_entry
        # print(widget_dict)
        return widget_dict

    def build_camera_dict(self):
        """Builds the runtime camera dictionary from the static configuration."""
        camera_dict = {}
        for key, config in CAMERA_CONFIG.items():
            new_entry = config.copy()
            
            indicator_name = config.get('INDICATOR_NAME')
            if indicator_name:
                new_entry['INDICATOR'] = getattr(self, indicator_name, None)

            timestamp_topic = config.get('TIMESTAMP_TOPIC')
            if timestamp_topic:
                new_entry['IS_ALIVE'] = False
                new_entry['TIMESTAMP_ENTRY'] = self.nt_manager.getEntry(timestamp_topic)

            connections_topic = config.get('CONNECTIONS_TOPIC')
            if connections_topic:
                new_entry['CONNECTIONS_ENTRY'] = self.nt_manager.getEntry(connections_topic)

            camera_dict[key] = new_entry
        return camera_dict

    def convert_cv_qt(self, cv_img, qlabel):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        p = convert_to_Qt_format.scaled(qlabel.width(), qlabel.height(), Qt.AspectRatioMode.KeepAspectRatio)
        return QtGui.QPixmap.fromImage(p)

    def update_routines(self, text):
        entry = self.widget_dict['qcombobox_autonomous_routines']['selected']
        entry.setString(text)
        self.nt_manager.flush()

    def label_click(self, label):
        command_entry = self.widget_dict[label].get('command_entry')
        if command_entry:
            toggled_state = not command_entry.getBoolean(True)
            print(f'You clicked {label}. Firing command at {datetime.today().strftime("%H:%M:%S")} ...', flush=True)
            command_entry.setBoolean(toggled_state)

    def initialize_widgets(self):
        """Connects widget signals and populates the camera combobox."""
        # Connect clickable label signals
        for key, d in self.widget_dict.items():
            if d.get('command_entry') and d.get('widget'):
                d['widget'].clicked.connect(lambda label=key: self.label_click(label))
        
        # Populate camera combobox
        for key in self.camera_dict.keys():
            self.qcombobox_cameras.addItem(key)

    def keyPressEvent(self, event):
        self.keys_currently_pressed.append(event.key())
        self.nt_manager.getEntry("SmartDashboard/keys_pressed").setIntegerArray(self.keys_currently_pressed)
        self.nt_manager.getEntry("SmartDashboard/key_pressed").setInteger(event.key())

    def keyReleaseEvent(self, event):
        try:
            while event.key() in self.keys_currently_pressed:
                self.keys_currently_pressed.remove(event.key())
        except ValueError:
            pass
        self.nt_manager.getEntry("SmartDashboard/key_pressed").setInteger(-999)
        self.nt_manager.getEntry("SmartDashboard/keys_pressed").setIntegerArray(self.keys_currently_pressed)

    def focusOutEvent(self, event):
        self.keys_currently_pressed = []
        self.nt_manager.getEntry("SmartDashboard/keys_pressed").setIntegerArray(self.keys_currently_pressed)

    def eventFilter(self, obj, event):
        if (obj is self.qt_text_entry_filter or obj is self.qt_text_new_value) and event.type() == QEvent.Type.KeyPress:
            if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                return True
        return super().eventFilter(obj, event)
