# This file contains the UIUpdater class, which is responsible for updating the widgets in the main UI.

import time
from datetime import datetime
import numpy as np
import wpimath.geometry as geo
from PyQt6 import QtGui, QtCore, QtWidgets
import math
import re

class UIUpdater:
    def __init__(self, ui):
        self.ui = ui

    def update_widgets(self):
        """ Main function which is looped to update the GUI with NT values"""
        # these are the styles for the gui labels
        style_on = "border: 7px; border-radius: 7px; background-color:rgb(80, 235, 0); color:rgb(0, 0, 0);"  # bright green, black letters
        style_off = "border: 7px; border-radius: 7px; background-color:rgb(220, 0, 0); color:rgb(200, 200, 200);"  # bright red, dull white letters (also flash off)
        style_high = "border: 7px; border-radius: 15px; background-color:rgb(80, 235, 0); color:rgb(0, 0, 0);"  # match time -> regular game, green
        style_low = "border: 7px; border-radius: 15px; background-color:rgb(0, 20, 255); color:rgb(255, 255, 255);"  # match time endgame - blue
        style_flash_on = "border: 7px; border-radius: 7px; background-color:rgb(0, 0, 0); color:rgb(255, 255, 255);"  # flashing on step 1 - black with thite letters
        style_flash_off = "border: 7px; border-radius: 7px; background-color:rgb(0, 20, 255); color:rgb(255, 255, 255);"  # flashing on step 2 - blue with white letters
        style_flash = style_flash_on if self.ui.counter % 30 < 15 else style_flash_off  # get things to blink

        # update the connection indicator
        style_disconnected = "border: 7px; border-radius: 7px; background-color:rgb(180, 180, 180); color:rgb(0, 0, 0);"
        style = style_on if self.ui.nt_manager.isConnected() else style_disconnected
        self.ui.widget_dict['qlabel_nt_connected']['widget'].setStyleSheet(style)

        # update all labels tied to NT entries
        for key, d in self.ui.widget_dict.items():
            if d.get('entry') is not None:
                widget = d.get('widget')
                if not widget:
                    continue

                if 'indicator' in key:
                    is_on = d['entry'].getBoolean(False)
                    if 'style_on' in d.keys():
                        style = d['style_on'] if is_on else d['style_off']
                    elif 'flash' in d.keys() and is_on:
                        style = style_flash
                    else:
                        style = style_on if is_on else style_off
                    widget.setStyleSheet(style)
                elif 'lcd' in key:
                    value = int(d['entry'].getDouble(0))
                    widget.display(str(value))
                elif 'monitor' in key:
                    value = d['entry'].getDouble(0)
                    widget.set_value(value)
                elif 'combo' in key:
                    new_list = d['entry'].getStringArray([])
                    if new_list != self.ui.autonomous_list:
                        widget.blockSignals(True)
                        widget.clear()
                        widget.addItems(new_list)
                        widget.blockSignals(False)
                        self.ui.autonomous_list = new_list
                    selected_routine = self.ui.nt_manager.getEntry(d['selected']).getString('')
                    if selected_routine != widget.currentText():
                        widget.blockSignals(True)
                        widget.setCurrentText(selected_routine)
                        widget.blockSignals(False)
                elif 'time' in key:
                    match_time = d['entry'].getDouble(0)
                    widget.setText(str(int(match_time)))
                    if match_time < 30:
                        widget.setText(f'* {int(match_time)} *')
                        widget.setStyleSheet(style_flash)
                    else:
                        widget.setText(str(int(match_time)))
                        widget.setStyleSheet(style_high)

        # update the commands list
        green = QtGui.QColor(227, 255, 227)
        white = QtGui.QColor(255, 255, 255)
        for ix, (key, d) in enumerate(self.ui.command_dict.items()):
            bg_color = green if d['entry'].getBoolean(True) else white
            self.ui.qlistwidget_commands.item(ix).setBackground(bg_color)

        # update the ball position on the hub target image - left over from 2022 that I need to get rid of
        hub_targets = self.ui.widget_dict['hub_targets']['entry'].getDouble(0)
        hub_rotation = self.ui.widget_dict['hub_rotation']['entry'].getDouble(0) - 5
        hub_distance = self.ui.widget_dict['hub_distance']['entry'].getDouble(0)

        if hub_targets > 0:
            shooter_rpm = 2000
            shooter_distance = shooter_rpm * 0.00075
            center_offset = shooter_distance * -np.sin(hub_rotation * 3.14159 / 180)
            hub_x = 205 + center_offset * (380 / 1.2) # 380 px per 1.2 m
            hub_y = 190
            self.ui.qlabel_ball.move(int(hub_x), int(hub_y))
            if self.ui.qlabel_ball.isHidden():
                self.ui.qlabel_ball.show()
        else:
            if not self.ui.qlabel_ball.isHidden():
                self.ui.qlabel_ball.hide()

        # update the pose
        width, height = self.ui.qgroupbox_field.width(), self.ui.qgroupbox_field.height()
        drive_pose = self.ui.widget_dict['drive_pose']['entry'].getDoubleArray([0, 0, 0])
        x_pad = 1 if drive_pose[0] < 10 else 0
        theta_pad = 0
        theta_pad = theta_pad + 1 if drive_pose[2] < 0 else theta_pad
        theta_pad = theta_pad + 1 if abs(drive_pose[2]) < 100 else theta_pad
        theta_pad = theta_pad + 1 if abs(drive_pose[2]) < 10 else theta_pad
        pose_msg = f'POSE\n{" " * x_pad}{drive_pose[0]:>5.2f}m {drive_pose[1]:>4.2f}m {" " * theta_pad}{drive_pose[2]:>4.0f}°'
        self.ui.qlabel_pose_indicator.setText(pose_msg)

        # quick quest pose hack
        quest_pose = self.ui.widget_dict['quest_pose']['entry'].getString('No quest pose')
        pattern = r"x=(?P<x>-?\d+\.\d+).*y=(?P<y>-?\d+\.\d+).*Rotation2d\((?P<rotation>-?\d+\.\d+)\)"
        match = re.search(pattern, quest_pose)
        if match:
            quest_x, quest_y, quest_rot = float(match.group("x")), float(match.group("y")), float(match.group("rotation"))
            quest_rot = math.degrees(quest_rot)
        else:
            quest_x, quest_y, quest_rot = -1, -1, -1
        pose_msg = f'QUEST POSE\n{" " * x_pad}{quest_x:>5.2f}m {quest_y:>4.2f}m {" " * theta_pad}{quest_rot:>4.0f}°'
        self.ui.qlabel_quest_pose_indicator.setText(pose_msg)

        pixmap_rotated = self.ui.robot_pixmap.transformed(QtGui.QTransform().rotate(90-drive_pose[2]), QtCore.Qt.TransformationMode.SmoothTransformation)
        new_size = int(41 * (1 + 0.41 * np.abs(np.sin(2 * drive_pose[2] * np.pi / 180.0))))
        self.ui.qlabel_robot.resize(new_size, new_size)
        self.ui.qlabel_robot.setPixmap(pixmap_rotated)
        self.ui.qlabel_robot.move(int(-new_size / 2 + width * drive_pose[0] / 17.6), int(-new_size / 2 + height * (1 - drive_pose[1] / 8.2)))

        quest_pixmap_rotated = self.ui.quest_pixmap.transformed(QtGui.QTransform().rotate(90-quest_rot), QtCore.Qt.TransformationMode.SmoothTransformation)
        quest_new_size = int(41 * (1 + 0.41 * np.abs(np.sin(2 * quest_rot * np.pi / 180.0))))
        self.ui.qlabel_quest.resize(quest_new_size, quest_new_size)
        self.ui.qlabel_quest.setPixmap(quest_pixmap_rotated)
        self.ui.qlabel_quest.move(int(-new_size/2 + width * quest_x / 17.6 ), int(-new_size/2 + height * (1 - quest_y / 8.2)))

        # --------------  CAMERA STATUS INDICATORS  ---------------
        allowed_delay = 0.5
        timestamp = self.ui.robot_timestamp_entry.getDouble(1)
        cam_keys = [key for key in self.ui.camera_dict.keys() if 'IS_ALIVE' in self.ui.camera_dict[key]]

        for key in cam_keys:
            is_alive = (timestamp - self.ui.camera_dict[key]['TIMESTAMP_ENTRY'].getDouble(-1)) < allowed_delay
            self.ui.camera_dict[key]['IS_ALIVE'] = is_alive
            connections = int(self.ui.camera_dict[key]['CONNECTIONS_ENTRY'].getDouble(0))
            style = style_on if is_alive else style_off
            indicator = self.ui.camera_dict[key]['INDICATOR']
            indicator.setStyleSheet(style)
            indicator.setText(f'{self.ui.camera_dict[key]["NICKNAME"]}: {connections:2d}')

        # --------------  SPEAKER POSITION CALCULATIONS  ---------------
        k_blue_speaker = [0, 5.55, 180]
        k_red_speaker = [16.5, 5.555, 0]
        if self.ui.widget_dict['qlabel_alliance_indicator']['entry'].getBoolean(False):
            translation_origin_to_speaker = geo.Translation2d(k_red_speaker[0], k_red_speaker[1])
        else:
            translation_origin_to_speaker = geo.Translation2d(k_blue_speaker[0], k_blue_speaker[1])
        translation_origin_to_robot = geo.Translation2d(drive_pose[0], drive_pose[1])
        translation_robot_to_speaker = translation_origin_to_speaker - translation_origin_to_robot
        desired_angle = translation_robot_to_speaker.angle().rotateBy(geo.Rotation2d(np.radians(180)))
        angle_to_speaker = drive_pose[2] - desired_angle.degrees()
        angle_tolerance = 10

        best_distance = 1.7
        distance_tolerance = 0.4
        speaker_blue = (0, 5.56)
        speaker_red = (16.54, 5.56)
        speaker = speaker_red if self.ui.widget_dict['qlabel_alliance_indicator']['entry'].getBoolean(False) else speaker_blue
        shot_distance = np.sqrt((speaker[0]-drive_pose[0])**2 + (speaker[1]-drive_pose[1])**2)
        if shot_distance <= best_distance - distance_tolerance:
            shot_style = "border: 7px; border-radius: 7px; background-color:rgb(225, 0, 0); color:rgb(200, 200, 200);"
        elif shot_distance > best_distance - distance_tolerance and shot_distance < best_distance + distance_tolerance:
            grey_val = int(225 * np.abs(best_distance-shot_distance))
            if np.abs(angle_to_speaker) < angle_tolerance:
                text_color = '(0,0,0)' if self.ui.counter % 10 < 5 else '(255,255,255)'
                border_color = 'solid blue' if self.ui.counter % 10 < 5 else 'solid black'
                border_size = 6
            else:
                text_color = '(200, 200, 200)'
                border_color = 'solid red'
                border_size = 8
            shot_style = f"border: {border_size}px {border_color}; border-radius: 7px; background-color:rgb({grey_val}, {int(225-grey_val)}, {grey_val}); color:rgb{text_color};"
        else:
            shot_style = "border: 7px; border-radius: 7px; background-color:rgb(180, 180, 180); color:rgb(200, 200, 200);"
        self.ui.qlabel_shot_distance.setText(f'SHOT DIST\n{shot_distance:.1f}m  {int(angle_to_speaker):>+3d}°')
        self.ui.qlabel_shot_distance.setStyleSheet(shot_style)

        # update the 2024 arm configuration indicator
        config = self.ui.widget_dict['qlabel_position_indicator']['entry'].getString('?')
        if config.upper() not in ['LOW_SHOOT', 'INTAKE']:
            postion_style = style_on
        else:
            postion_style = style_on
        self.ui.qlabel_position_indicator.setText(f'POS: {config.upper()}')
        self.ui.qlabel_position_indicator.setStyleSheet(postion_style)

        self.ui.counter += 1
        if self.ui.counter % 80 == 0:
            current_time = time.time()
            time_delta = current_time - self.ui.previous_time
            if time_delta > 0: # Prevent ZeroDivisionError
                frames = self.ui.camera_manager.worker.frames if self.ui.camera_manager.worker is not None else 0
                msg = f'Gui Updates/s: {80/time_delta:.1f}, Cam Updates/s: {(frames-self.ui.previous_frames)/time_delta:.1f}'
                self.ui.statusBar().showMessage(msg)
            self.ui.previous_time = current_time
            self.ui.previous_frames = frames
