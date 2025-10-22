# pyqt example for teaching GUI development for FRC dashboard
# make sure to pip install pyqt6 pyqt6-tools

# print(f'Loading Modules ...', flush=True)
import os
os.environ["OPENCV_LOG_LEVEL"] = "DEBUG"  # Options: INFO, WARNING, ERROR, DEBUG

import cv2
print("[DEBUG] OpenCV version:", cv2.__version__)
cv2.setNumThreads(4)  # Disable multithreading
cv2.ocl.setUseOpenCL(True)

import time
from datetime import datetime
from pathlib import Path

from PyQt6 import QtCore, QtGui, QtWidgets, uic
from PyQt6.QtCore import Qt, QTimer, QEvent
from PyQt6.QtWidgets import QGraphicsOpacityEffect
from ntcore import NetworkTableType

import qlabel2
from warning_label import WarningLabel

# logical chunking of the gui's functional components
from config import get_widget_dict, get_camera_dict
from nt_manager import NTManager
from camera_manager import CameraManager
from ui_updater import UIUpdater

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
        self.ntinst = self.nt_manager.ntinst  # Keep for compatibility for now

        self.sorted_tree = None
        self.autonomous_list = []

        self.refresh_time = 50
        self.previous_frames = 0
        self.widget_dict = {}
        self.command_dict = {}
        
        self.camera_dict = get_camera_dict(self)
        self.widget_dict = get_widget_dict(self)

        self.robot_timestamp_entry = self.nt_manager.getEntry('/SmartDashboard/_timestamp')

        self.qlabel_pdh_voltage_monitor.update_settings(min_val=8, max_val=12, red_high=False, display_float=True)
        self.qlabel_pdh_current_monitor.update_settings(min_val=60, max_val=160, red_high=True, display_float=False)

        self.initialize_widgets()

        self.qaction_show_hide.triggered.connect(self.toggle_network_tables)
        self.qaction_refresh.triggered.connect(self.refresh_tree)

        self.qlistwidget_commands.clicked.connect(self.command_list_clicked)
        self.qcombobox_autonomous_routines.currentTextChanged.connect(self.update_routines)
        self.qt_text_entry_filter.textChanged.connect(self.filter_nt_keys_combo)
        self.qcombobox_nt_keys.currentTextChanged.connect(self.update_selected_key)
        self.qt_tree_widget_nt.clicked.connect(self.qt_tree_widget_nt_clicked)

        self.qt_text_entry_filter.installEventFilter(self)
        self.qt_text_new_value.installEventFilter(self)

        self.robot_pixmap = QtGui.QPixmap("png\\blockhead.png")
        self.quest_pixmap = QtGui.QPixmap("png\\quest.png")
        opacity_effect = QGraphicsOpacityEffect()
        opacity_effect.setOpacity(0.5)
        self.qlabel_quest.setGraphicsEffect(opacity_effect)
        self.qlabel_robot.raise_()

        self.qt_button_set_key.clicked.connect(self.update_key)
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

    def update_selected_key(self):
        x = self.nt_manager.getEntry(self.qcombobox_nt_keys.currentText()).getValue()
        if x is not None:
            self.qt_text_current_value.setPlainText(str(x.value()))

    def convert_cv_qt(self, cv_img, qlabel):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format.Format_RGB888)
        p = convert_to_Qt_format.scaled(qlabel.width(), qlabel.height(), Qt.AspectRatioMode.KeepAspectRatio)
        return QtGui.QPixmap.fromImage(p)

    def filter_nt_keys_combo(self):
        if self.sorted_tree is not None:
            self.qcombobox_nt_keys.clear()
            filter_text = self.qt_text_entry_filter.toPlainText()
            filtered_keys = [key for key in self.sorted_tree if filter_text in key]
            self.qcombobox_nt_keys.addItems(filtered_keys)

    def update_key(self):
        key = self.qcombobox_nt_keys.currentText()
        entry = self.nt_manager.getEntry(key)
        entry_type = entry.getType()
        new_val_str = self.qt_text_new_value.toPlainText()
        print(f'Update key was called on {key}, which is a {entry_type}. Setting it to {new_val_str}', flush=True)
        try:
            if entry_type == NetworkTableType.kDouble:
                entry.setDouble(float(new_val_str))
            elif entry_type == NetworkTableType.kString:
                entry.setString(new_val_str)
            elif entry_type == NetworkTableType.kBoolean:
                entry.setBoolean(eval(new_val_str))
            else:
                self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: {key} type {entry_type} not in [double, bool, string]')
        except Exception as e:
            self.qt_text_status.appendPlainText(f'{datetime.today().strftime("%H:%M:%S")}: Error occurred in setting {key} - {e}')
        self.qt_text_new_value.clear()
        self.refresh_tree()

    def update_routines(self, text):
        key = self.widget_dict['qcombobox_autonomous_routines']['selected']
        self.nt_manager.getEntry(key).setString(text)
        self.nt_manager.flush()

    def label_click(self, label):
        command_entry = self.widget_dict[label].get('command_entry')
        if command_entry:
            toggled_state = not command_entry.getBoolean(True)
            print(f'You clicked {label}. Firing command at {datetime.today().strftime("%H:%M:%S")} ...', flush=True)
            command_entry.setBoolean(toggled_state)

    def initialize_widgets(self):
        # Populate NT entries for widgets
        for key, d in self.widget_dict.items():
            if d.get('nt'):
                d['entry'] = self.nt_manager.getEntry(d['nt'])
            if d.get('command'):
                d['command_entry'] = self.nt_manager.getEntry(d['command'])
                if d.get('widget'):
                    d['widget'].clicked.connect(lambda label=key: self.label_click(label))
        
        # Populate camera combobox
        for key in self.camera_dict.keys():
            self.qcombobox_cameras.addItem(key)

    def qt_tree_widget_nt_clicked(self, item):
        self.qt_text_entry_filter.clear()
        self.qt_text_entry_filter.setPlainText(item.data())

    def command_list_clicked(self, item):
        cell_content = item.data()
        command_entry = self.command_dict.get(cell_content, {}).get('entry')
        if command_entry:
            toggled_state = not command_entry.getBoolean(True)
            print(f'You clicked {cell_content}. Firing command...', flush=True)
            command_entry.setBoolean(toggled_state)

    def toggle_network_tables(self):
        if self.qt_tree_widget_nt.isHidden():
            self.refresh_tree()
            self.qt_tree_widget_nt.show()
        else:
            self.qt_tree_widget_nt.hide()

    def refresh_tree(self):
        """  Read networktables and update tree and combo widgets
        """
        if self.nt_manager.isConnected():
            self.nt_manager.report_nt_status()
            self.qt_tree_widget_nt.clear()
            entries = self.nt_manager.getEntries('/', 0)
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
                    self.command_dict.update({command_name: {'nt': item, 'entry': self.nt_manager.getEntry(item)}})

                entry_value = self.nt_manager.getEntry(item).getValue()
                value = entry_value.value()
                age = int(time.time() - entry_value.last_change() / 1E6)
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

    def test(self):  # test function for checking new signals
        print('Test was called', flush=True)

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
                self.fill_item(child, v)
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

# ... (depth, fill_item, new_item methods remain the same)
