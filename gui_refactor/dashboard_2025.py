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
import ntcore
import wpimath.geometry as geo

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

        # camera stuff - probably not needed
        self.worker = None
        # self.camera_enabled = False
        # self.thread = None

        # Build the runtime dictionaries from the static config
        self.widget_dict = self.build_widget_dict()
        self.camera_dict = self.build_camera_dict()
        # print(self.widget_dict)

        self.robot_timestamp_sub = self.ntinst.getDoubleTopic('/SmartDashboard/_timestamp').subscribe(0)

        # set the colors for the warning labels
        self.qlabel_pdh_voltage_monitor.update_settings(min_val=8, max_val=12, red_high=False, display_float=True)
        self.qlabel_pdh_current_monitor.update_settings(min_val=60, max_val=160, red_high=True, display_float=False)

        # Publishers for key presses
        self.keys_pressed_pub = self.ntinst.getIntegerArrayTopic("/SmartDashboard/keys_pressed").publish()
        self.key_pressed_pub = self.ntinst.getIntegerTopic("/SmartDashboard/key_pressed").publish()

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

        self.robot_pixmap = QtGui.QPixmap(str(self.png_dir / 'blockhead.png'))
        self.quest_pixmap = QtGui.QPixmap(str(self.png_dir / 'quest.png'))
        self.ghost_pixmap = QtGui.QPixmap(str(self.png_dir / 'ghost.png'))

        opacity_effect = QGraphicsOpacityEffect()
        opacity_effect.setOpacity(0.5)
        self.qlabel_quest.setGraphicsEffect(opacity_effect)

        ghost_effect = QGraphicsOpacityEffect()
        ghost_effect.setOpacity(0.25)
        self.qlabel_ghost.setGraphicsEffect(ghost_effect)
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
            
            new_entry['last_value'] = None
            # Determine Topic Type based on update_style
            style = config.get('update_style')
            nt_topic = config.get('nt_topic')
            
            if nt_topic:
                if style == 'indicator':
                    new_entry['subscriber'] = self.ntinst.getBooleanTopic(nt_topic).subscribe(False)
                elif style in ['lcd', 'monitor', 'time', 'hub']:
                    new_entry['subscriber'] = self.ntinst.getDoubleTopic(nt_topic).subscribe(0.0)
                elif style == 'pose':
                    new_entry['subscriber'] = self.ntinst.getStructTopic(nt_topic, geo.Pose2d).subscribe(geo.Pose2d())
                elif style == 'combo':
                    new_entry['subscriber'] = self.ntinst.getStringArrayTopic(nt_topic).subscribe([])
                elif style == 'position':
                    new_entry['subscriber'] = self.ntinst.getStringTopic(nt_topic).subscribe("")

            command_topic = config.get('command_topic')
            if command_topic:
                new_entry['publisher'] = self.ntinst.getBooleanTopic(command_topic).publish()
                new_entry['command_subscriber'] = self.ntinst.getBooleanTopic(command_topic).subscribe(False)

            selected_topic = config.get('selected_topic')
            if selected_topic:
                new_entry['selected_subscriber'] = self.ntinst.getStringTopic(selected_topic).subscribe("")
                new_entry['last_selected_value'] = None
                new_entry['selected_publisher'] = self.ntinst.getStringTopic(selected_topic).publish()
                # print(f'{key} has selected topic: {selected_topic} with value {new_entry[selected_subscriber].get()'}

            visible_topic = config.get('visible_topic')  # sometimes we want to hide things
            if visible_topic:
                new_entry['visible_subscriber'] = self.ntinst.getBooleanTopic(visible_topic).subscribe(False)
                new_entry['last_visible_value'] = None

            widget_dict[key] = new_entry
        # print(widget_dict)
        return widget_dict

    def build_camera_dict(self):
        """Builds the runtime camera dictionary from the static configuration."""
        camera_dict = {}
        for key, config in CAMERA_CONFIG.items():
            new_entry = config.copy()
            
            indicator_name = config.get('HEARTBEAT_INDICATOR_NAME')
            if indicator_name:
                new_entry['INDICATOR'] = getattr(self, indicator_name, None)

            # update the text on the target indicators when we start - now we don't have to edit UI if we change names
            target_indicator_name = config.get('TARGET_INDICATOR_NAME')
            if target_indicator_name:
                target_widget = getattr(self, target_indicator_name, None)
                if target_widget:
                    nickname = config.get('NICKNAME', key)
                    if target_widget.width() < 100:
                        nickname = nickname.replace(' ', '\n')
                    target_widget.setText(nickname)

            timestamp_topic = config.get('TIMESTAMP_TOPIC')
            if timestamp_topic:
                new_entry['IS_ALIVE'] = False
                new_entry['RECONNECTION_COUNT'] = 0
                new_entry['last_is_alive'] = None
                new_entry['last_connections'] = None
                new_entry['TIMESTAMP_SUB'] = self.ntinst.getDoubleTopic(timestamp_topic).subscribe(-1)

            connections_topic = config.get('CONNECTIONS_TOPIC')
            if connections_topic:
                new_entry['CONNECTIONS_SUB'] = self.ntinst.getDoubleTopic(connections_topic).subscribe(0)

            camera_dict[key] = new_entry
        return camera_dict

    def update_routines(self, text):
        pub = self.widget_dict['qcombobox_autonomous_routines'].get('selected_publisher')
        if pub:
            pub.set(text)
        self.ntinst.flush()

    def label_click(self, label):
        props = self.widget_dict[label]
        pub = props.get('publisher')
        sub = props.get('command_subscriber')
        
        if pub and sub:
            # Check if the topic has data. If time is 0, it means no value has been set (by robot or us).
            atomic_val = sub.getAtomic()
            if atomic_val.time == 0:
                print(f'Warning: {label} clicked but NT topic {sub.getTopic().getName()} has no value (Robot not connected?).', flush=True)
                return

            # Toggle based on current state of the command topic
            toggled_state = not atomic_val.value
            print(f'You clicked {label}. Firing command at {datetime.today().strftime("%H:%M:%S")} ...', flush=True)
            pub.set(toggled_state)
        else:
            print(f'Warning: {label} clicked but NT Publisher or Subscriber is missing.', flush=True)


    def initialize_widgets(self):
        """Connects widget signals and populates the camera combobox."""
        # Connect clickable label signals
        for key, widget_props in self.widget_dict.items():
            if widget_props.get('publisher') and widget_props.get('widget'):
                widget_props['widget'].clicked.connect(lambda label=key: self.label_click(label))
        
        # Populate camera combobox
        for key in self.camera_dict.keys():
            self.qcombobox_cameras.addItem(key)

    def keyPressEvent(self, event):
        if event.key() not in self.keys_currently_pressed:
            self.keys_currently_pressed.append(event.key())
        self.keys_pressed_pub.set(self.keys_currently_pressed)
        self.key_pressed_pub.set(event.key())

    def keyReleaseEvent(self, event):
        try:
            while event.key() in self.keys_currently_pressed:
                self.keys_currently_pressed.remove(event.key())
        except ValueError:
            pass
        self.key_pressed_pub.set(-999)
        self.keys_pressed_pub.set(self.keys_currently_pressed)

    def focusOutEvent(self, event):
        self.keys_currently_pressed = []
        self.keys_pressed_pub.set(self.keys_currently_pressed)

    def eventFilter(self, obj, event):
        if (obj is self.qt_text_entry_filter or obj is self.qt_text_new_value) and event.type() == QEvent.Type.KeyPress:
            if event.key() in (Qt.Key.Key_Return, Qt.Key.Key_Enter):
                return True
        return super().eventFilter(obj, event)
