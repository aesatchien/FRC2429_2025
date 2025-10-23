# This file contains the UIUpdater class, which is responsible for updating the widgets in the main UI.

import time
import math
import re
import numpy as np
import wpimath.geometry as geo
from PyQt6 import QtGui, QtCore, QtWidgets

class UIUpdater:
    # Define styles as class constants to avoid magic strings
    STYLE_ON = "border: 7px; border-radius: 7px; background-color:rgb(80, 235, 0); color:rgb(0, 0, 0);"
    STYLE_OFF = "border: 7px; border-radius: 7px; background-color:rgb(220, 0, 0); color:rgb(200, 200, 200);"
    STYLE_HIGH = "border: 7px; border-radius: 15px; background-color:rgb(80, 235, 0); color:rgb(0, 0, 0);"
    STYLE_LOW = "border: 7px; border-radius: 15px; background-color:rgb(0, 20, 255); color:rgb(255, 255, 255);"
    STYLE_FLASH_ON = "border: 7px; border-radius: 7px; background-color:rgb(0, 0, 0); color:rgb(255, 255, 255);"
    STYLE_FLASH_OFF = "border: 7px; border-radius: 7px; background-color:rgb(0, 20, 255); color:rgb(255, 255, 255);"
    STYLE_DISCONNECTED = "border: 7px; border-radius: 7px; background-color:rgb(180, 180, 180); color:rgb(0, 0, 0);"

    def __init__(self, ui):
        self.ui = ui
        self.drive_pose = [0, 0, 0]  # Store pose for other calculations

        # Map update styles from config to specific update functions
        self.updaters = {
            'indicator': self._update_indicator,
            'combo': self._update_combo,
            'time': self._update_time,
            'monitor': self._update_monitor,
            'lcd': self._update_lcd,
            'position': self._update_position,
            'hub': self._update_hub,  # Legacy from 2023
        }

    def update_widgets(self):
        """Main update loop. Orchestrates calls to specialized update functions."""
        self._update_connection_status()  # check if NT is connected

        # Functions with dependencies are called in a specific order
        self._update_pose_and_field()  # Must be called first to get latest pose
        self._update_camera_indicators()  # Check cameara connection states
        self._update_shot_calculations()  # Depends on pose data

        # Use the dispatcher for all other standard widgets based on their update style
        for widget_props in self.ui.widget_dict.values():
            update_style = widget_props.get('update_style')
            if update_style in self.updaters:
                self.updaters[update_style](widget_props)  # call the appropriate update method

        self._update_command_list()  # autonomous commands
        self._update_fps_counter()  # status bar updates

        self.ui.counter += 1

    # --------------------------------------------------------------------------
    # Specialized Update Functions (Called from the main update_widgets loop)
    # --------------------------------------------------------------------------

    def _update_connection_status(self):
        """Updates the NT connection status indicator."""
        widget = self.ui.widget_dict['qlabel_nt_connected']['widget']
        style = self.STYLE_ON if self.ui.nt_manager.isConnected() else self.STYLE_DISCONNECTED
        widget.setStyleSheet(style)

    def _update_pose_and_field(self):
        """Updates the robot and quest pose on the field graphic."""
        drive_pose_entry = self.ui.widget_dict['drive_pose'].get('nt_entry')
        if not drive_pose_entry:
            return

        self.drive_pose = drive_pose_entry.getDoubleArray([0, 0, 0])
        width, height = self.ui.qgroupbox_field.width(), self.ui.qgroupbox_field.height()

        # Update Robot Pose Text and Graphic
        x_pad = 1 if self.drive_pose[0] < 10 else 0
        theta_pad = sum([1 for t in [0, 100, 10] if abs(self.drive_pose[2]) < t])
        pose_msg = f'POSE\n{" " * x_pad}{self.drive_pose[0]:>5.2f}m {self.drive_pose[1]:>4.2f}m {" " * theta_pad}{self.drive_pose[2]:>4.0f}°'
        self.ui.qlabel_pose_indicator.setText(pose_msg)

        pixmap_rotated = self.ui.robot_pixmap.transformed(QtGui.QTransform().rotate(90 - self.drive_pose[2]), QtCore.Qt.TransformationMode.SmoothTransformation)
        new_size = int(41 * (1 + 0.41 * np.abs(np.sin(2 * self.drive_pose[2] * np.pi / 180.0))))
        self.ui.qlabel_robot.resize(new_size, new_size)
        self.ui.qlabel_robot.setPixmap(pixmap_rotated)
        self.ui.qlabel_robot.move(int(-new_size / 2 + width * self.drive_pose[0] / 17.6), int(-new_size / 2 + height * (1 - self.drive_pose[1] / 8.2)))

        # Update Quest Pose Text and Graphic
        quest_pose_entry = self.ui.widget_dict['quest_pose'].get('nt_entry')
        quest_pose_str = quest_pose_entry.getString('No quest pose')
        match = re.search(r"x=(?P<x>-?\d+\.\d+).*y=(?P<y>-?\d+\.\d+).*Rotation2d\((?P<rotation>-?\d+\.\d+)\)", quest_pose_str)
        if match:
            quest_x, quest_y, quest_rot = float(match.group("x")), float(match.group("y")), math.degrees(float(match.group("rotation")))
        else:
            quest_x, quest_y, quest_rot = -1, -1, -1
        
        quest_pose_msg = f'QUEST POSE\n{" " * x_pad}{quest_x:>5.2f}m {quest_y:>4.2f}m {" " * theta_pad}{quest_rot:>4.0f}°'
        self.ui.qlabel_quest_pose_indicator.setText(quest_pose_msg)

        quest_pixmap_rotated = self.ui.quest_pixmap.transformed(QtGui.QTransform().rotate(90 - quest_rot), QtCore.Qt.TransformationMode.SmoothTransformation)
        quest_new_size = int(41 * (1 + 0.41 * np.abs(np.sin(2 * quest_rot * np.pi / 180.0))))
        self.ui.qlabel_quest.resize(quest_new_size, quest_new_size)
        self.ui.qlabel_quest.setPixmap(quest_pixmap_rotated)
        self.ui.qlabel_quest.move(int(-quest_new_size/2 + width * quest_x / 17.6), int(-quest_new_size/2 + height * (1 - quest_y / 8.2)))

    def _update_camera_indicators(self):
        """Updates the status indicators for all cameras."""
        allowed_delay = 0.5
        timestamp = self.ui.robot_timestamp_entry.getDouble(1)

        for cam_props in self.ui.camera_dict.values():
            if 'TIMESTAMP_ENTRY' in cam_props:
                is_alive = (timestamp - cam_props['TIMESTAMP_ENTRY'].getDouble(-1)) < allowed_delay
                cam_props['IS_ALIVE'] = is_alive
                connections = int(cam_props['CONNECTIONS_ENTRY'].getDouble(0))
                style = self.STYLE_ON if is_alive else self.STYLE_OFF
                indicator = cam_props['INDICATOR']
                if indicator:
                    indicator.setStyleSheet(style)
                    indicator.setText(f'{cam_props["NICKNAME"]}: {connections:2d}')

    def _update_shot_calculations(self):
        """ Calculates and displays shot distance and angle to speaker - legacy from 2024. """
        alliance_entry = self.ui.widget_dict['qlabel_alliance_indicator'].get('nt_entry')
        if not alliance_entry:
            return

        is_red_alliance = alliance_entry.getBoolean(False)
        k_speaker = [16.5, 5.555, 0] if is_red_alliance else [0, 5.55, 180]
        speaker_coords = (16.54, 5.56) if is_red_alliance else (0, 5.56)

        translation_origin_to_speaker = geo.Translation2d(k_speaker[0], k_speaker[1])
        translation_origin_to_robot = geo.Translation2d(self.drive_pose[0], self.drive_pose[1])
        translation_robot_to_speaker = translation_origin_to_speaker - translation_origin_to_robot
        desired_angle = translation_robot_to_speaker.angle().rotateBy(geo.Rotation2d(np.radians(180)))
        angle_to_speaker = self.drive_pose[2] - desired_angle.degrees()
        shot_distance = np.sqrt((speaker_coords[0] - self.drive_pose[0])**2 + (speaker_coords[1] - self.drive_pose[1])**2)

        best_distance, dist_tolerance, angle_tolerance = 1.7, 0.4, 10
        shot_style = self.STYLE_OFF
        if best_distance - dist_tolerance < shot_distance < best_distance + dist_tolerance:
            grey_val = int(225 * abs(best_distance - shot_distance))
            in_angle = abs(angle_to_speaker) < angle_tolerance
            text_color = '(0,0,0)' if self.ui.counter % 10 < 5 and in_angle else '(255,255,255)'
            border_color = 'solid blue' if self.ui.counter % 10 < 5 and in_angle else 'solid black'
            border_size = 6 if in_angle else 8
            shot_style = f"border: {border_size}px {border_color}; border-radius: 7px; background-color:rgb({grey_val}, {int(225-grey_val)}, {grey_val}); color:rgb{text_color};"
        elif shot_distance >= best_distance + dist_tolerance:
            shot_style = self.STYLE_DISCONNECTED

        self.ui.qlabel_shot_distance.setText(f'SHOT DIST\n{shot_distance:.1f}m  {int(angle_to_speaker):>+3d}°')
        self.ui.qlabel_shot_distance.setStyleSheet(shot_style)

    def _update_command_list(self):
        """Updates the background color of the command list widget in the NT tree - which is currently broken"""
        green, white = QtGui.QColor(227, 255, 227), QtGui.QColor(255, 255, 255)
        for i, props in enumerate(self.ui.command_dict.values()):
            nt_entry = props.get('nt_entry')
            if nt_entry:
                is_running = nt_entry.getBoolean(False)
                self.ui.qlistwidget_commands.item(i).setBackground(green if is_running else white)

    def _update_fps_counter(self):
        """Updates the status bar with GUI and camera FPS."""
        if self.ui.counter % 80 == 0:
            current_time = time.time()
            time_delta = current_time - self.ui.previous_time
            if time_delta > 0:
                frames = self.ui.camera_manager.worker.frames if self.ui.camera_manager.worker else 0
                msg = f'Gui Updates/s: {80/time_delta:.1f}, Cam Updates/s: {(frames - self.ui.previous_frames)/time_delta:.1f}'
                self.ui.statusBar().showMessage(msg)
            self.ui.previous_time = current_time
            self.ui.previous_frames = frames

    # --------------------------------------------------------------------------
    # Dispatcher Functions (Called from the self.updaters dictionary)
    # --------------------------------------------------------------------------

    def _update_indicator(self, props):
        """ Updates a label with a style based on state of the boolean tied to it """
        # TODO - maybe separate this based on whether it's monitoring a robot state boolean vs a command currently running
        nt_entry, widget = props.get('nt_entry'), props.get('widget')
        if not (nt_entry and widget):
            return

        is_on = nt_entry.getBoolean(False)
        style_flash = self.STYLE_FLASH_ON if self.ui.counter % 30 < 15 else self.STYLE_FLASH_OFF

        if 'style_on' in props:  # allow a custom style for each indicator
            style = props['style_on'] if is_on else props['style_off']
        elif 'flash' in props and is_on:
            style = style_flash  #  use the flashing style
        else:
            style = self.STYLE_ON if is_on else self.STYLE_OFF  # go with the default
        widget.setStyleSheet(style)

    def _update_lcd(self, props):
        """ Updates a numeric LCD widget with an integer """
        # TODO - replace these with a better custom LCD-font widget
        nt_entry, widget = props.get('nt_entry'), props.get('widget')
        if nt_entry and widget:
            value = int(nt_entry.getDouble(0))
            widget.display(str(value))

    def _update_monitor(self, props):
        """ Updates a warninglabel with "good" and "bad" colors (set in UI's init) based on the value """
        nt_entry, widget = props.get('nt_entry'), props.get('widget')
        if nt_entry and widget:
            value = nt_entry.getDouble(0)
            widget.set_value(value)

    def _update_combo(self, props):
        """ At the moment this is just for the autonomous routines combo box but it could be for any dropdown """
        nt_entry, widget = props.get('nt_entry'), props.get('widget')
        # print(f'found combo on {props.get("widget_name")} at {self.ui.counter} with nt_entry {nt_entry} and widget {widget}')
        # there is a problem with using shortcut checks for validity - the combobox is not True if it is empty (len 0)
        # so just check for None
        if nt_entry is None or widget is None:
            return

        new_list = nt_entry.getStringArray([])
        if new_list != self.ui.autonomous_list:
            widget.blockSignals(True)
            widget.clear()
            widget.addItems(new_list)
            widget.blockSignals(False)
            self.ui.autonomous_list = new_list

        selected_nt_entry = props.get('selected_nt_entry')
        if selected_nt_entry:
            selected_routine = selected_nt_entry.getString('')
            if selected_routine != widget.currentText():
                widget.blockSignals(True)
                widget.setCurrentText(selected_routine)
                widget.blockSignals(False)

    def _update_time(self, props):
        """ This is for the match time remaining widget - lame in sim but correct for matches """
        nt_entry, widget = props.get('nt_entry'), props.get('widget')
        if not (nt_entry and widget):
            return

        match_time = nt_entry.getDouble(0)
        style_flash = self.STYLE_FLASH_ON if self.ui.counter % 30 < 15 else self.STYLE_FLASH_OFF
        if match_time < 30:
            widget.setText(f'* {int(match_time)} *')
            widget.setStyleSheet(style_flash)
        else:
            widget.setText(str(int(match_time)))
            widget.setStyleSheet(self.STYLE_HIGH)

    def _update_position(self, props):
        """  This was specific for CrankSinatra's shot position"""
        nt_entry, widget = props.get('nt_entry'), props.get('widget')
        if not (nt_entry and widget):
            return

        config = nt_entry.getString('?')
        # This logic seems to always result in STYLE_ON, may need review
        position_style = self.STYLE_ON if config.upper() not in ['LOW_SHOOT', 'INTAKE'] else self.STYLE_ON
        widget.setText(f'POS: {config.upper()}')
        widget.setStyleSheet(position_style)

    def _update_hub(self, props):
        """Legacy hub update logic from 2023. May need removal."""
        hub_targets_entry = self.ui.widget_dict['hub_targets'].get('nt_entry')
        if not hub_targets_entry:
            return

        hub_targets = hub_targets_entry.getDouble(0)
        if hub_targets > 0:
            hub_rotation = self.ui.widget_dict['hub_rotation']['nt_entry'].getDouble(0) - 5
            shooter_rpm = 2000
            shooter_distance = shooter_rpm * 0.00075
            center_offset = shooter_distance * -np.sin(hub_rotation * 3.14159 / 180)
            hub_x = 205 + center_offset * (380 / 1.2)
            self.ui.qlabel_ball.move(int(hub_x), 190)
            if self.ui.qlabel_ball.isHidden():
                self.ui.qlabel_ball.show()
        else:
            if not self.ui.qlabel_ball.isHidden():
                self.ui.qlabel_ball.hide()
