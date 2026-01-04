# This file contains the UIUpdater class, which is responsible for updating the widgets in the main UI.

import time
import math
import numpy as np
import wpimath.geometry as geo
import ntcore
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
        self.drive_pose = geo.Pose2d()  # Store pose for other calculations
        self.flash_on = False

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
        
        # Calculate flash state once per cycle
        self.flash_on = self.ui.counter % 30 < 15

        # Functions with dependencies are called in a specific order
        self._update_pose_and_field()  # Must be called first to get latest pose
        self._update_camera_indicators()  # Check camera connection states
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
        style = self.STYLE_ON if self.ui.ntinst.isConnected() else self.STYLE_DISCONNECTED
        widget.setStyleSheet(style)

    def _update_pose_and_field(self):
        """Updates the robot and quest pose on the field graphic."""
        drive_props = self.ui.widget_dict['drive_pose']
        drive_pose_sub = drive_props.get('subscriber')
        if not drive_pose_sub:
            return # Can't do anything without the robot pose
        
        self.drive_pose = drive_pose_sub.get()
        quest_props = self.ui.widget_dict['quest_pose']
        quest_pose_sub = quest_props.get('subscriber')
        self.quest_pose = quest_pose_sub.get()
        field_dims = (self.ui.qgroupbox_field.width(), self.ui.qgroupbox_field.height())

        # Update Robot Pose if changed
        if self.drive_pose != drive_props.get('last_value'):
            drive_props['last_value'] = self.drive_pose
            self.ui.qlabel_pose_indicator.setText(self._format_pose_string("POSE", self.drive_pose))
            self._update_pose_widget(self.ui.qlabel_robot, self.ui.robot_pixmap, self.drive_pose, field_dims)

        # Update Quest Pose if changed
        if self.quest_pose != quest_props.get('last_value'):
            quest_props['last_value'] = self.quest_pose
            self.ui.qlabel_quest_pose_indicator.setText(self._format_pose_string("QUEST POSE", self.quest_pose))
            self._update_pose_widget(self.ui.qlabel_quest, self.ui.quest_pixmap, self.quest_pose, field_dims)

        # Update Ghost Pose
        ghost_props = self.ui.widget_dict['ghost_pose']
        ghost_sub = ghost_props.get('subscriber')
        visible_sub = ghost_props.get('visible_subscriber')

        is_visible = visible_sub.get() if visible_sub else False

        if is_visible and ghost_sub:
            ghost_pose = ghost_sub.get()
            if ghost_pose != ghost_props.get('last_value') or is_visible != ghost_props.get('last_visible_value'):
                ghost_props['last_value'] = ghost_pose
                self.ui.qlabel_ghost.show()
                self._update_pose_widget(self.ui.qlabel_ghost, self.ui.ghost_pixmap, ghost_pose, field_dims)
        else:
            if self.ui.qlabel_ghost.isVisible():
                self.ui.qlabel_ghost.hide()
        
        ghost_props['last_visible_value'] = is_visible

        # Update Target Pose (Array)
        target_props = self.ui.widget_dict['target_pose']
        target_sub = target_props.get('subscriber')
        visible_sub = target_props.get('visible_subscriber')
        target_widgets = target_props.get('widgets', [self.ui.qlabel_target])

        is_visible = visible_sub.get() if visible_sub else False

        if is_visible and target_sub:
            target_poses = target_sub.get() # List of Pose2d
            if target_poses != target_props.get('last_value') or is_visible != target_props.get('last_visible_value'):
                target_props['last_value'] = target_poses
                for i, widget in enumerate(target_widgets):
                    if i < len(target_poses):
                        widget.show()
                        self._update_pose_widget(widget, self.ui.target_pixmap, target_poses[i], field_dims)
                    else:
                        widget.hide()
        else:
            if target_props.get('last_visible_value') != False:
                for widget in target_widgets:
                    if widget.isVisible():
                        widget.hide()

        target_props['last_visible_value'] = is_visible

    def _update_camera_indicators(self):
        """Updates the status indicators for all cameras.
        We check the NT timestamp from the camera framecount from the camera"""
        # 1.5 seconds allowed delay (500,000 microseconds) - we update the framecount once per second
        allowed_delay_us = 1500000
        now = ntcore._now()

        for cam_props in self.ui.camera_dict.values():
            if 'FRAMECOUNT_SUB' in cam_props:
                # Check how long ago the camera updated its _frames topic
                is_alive = (now - cam_props['FRAMECOUNT_SUB'].getAtomic().time) < allowed_delay_us
                
                # Check for rising edge (False -> True) to count reconnections
                if is_alive and not cam_props['IS_ALIVE']:
                    cam_props['RECONNECTION_COUNT'] += 1

                cam_props['IS_ALIVE'] = is_alive

                # if we ever start serving _connections from the robot, then this will override the local count
                connections = cam_props['RECONNECTION_COUNT']
                if 'CONNECTIONS_SUB' in cam_props:
                    atomic_conn = cam_props['CONNECTIONS_SUB'].getAtomic()
                    if atomic_conn.time != 0:
                        connections = int(atomic_conn.value)

                # Only update widget if visual state changed
                if is_alive == cam_props.get('last_is_alive') and connections == cam_props.get('last_connections'):
                    continue
                cam_props['last_is_alive'] = is_alive
                cam_props['last_connections'] = connections

                style = self.STYLE_ON if is_alive else self.STYLE_OFF
                indicator = cam_props.get('INDICATOR')  # switch to safe access since we can have cameras w/o heartbeat indicators
                if indicator:
                    indicator.setStyleSheet(style)
                    indicator.setText(f'{cam_props["NICKNAME"]}: {connections:2d}')

    def _update_shot_calculations(self):
        """ Calculates and displays shot distance and angle to speaker - legacy from 2024. """
        alliance_sub = self.ui.widget_dict['qlabel_alliance_indicator'].get('subscriber')
        if not alliance_sub:
            return

        is_red_alliance = alliance_sub.get()
        k_speaker = [16.5, 5.555, 0] if is_red_alliance else [0, 5.55, 180]
        speaker_coords = (16.54, 5.56) if is_red_alliance else (0, 5.56)

        translation_origin_to_speaker = geo.Translation2d(k_speaker[0], k_speaker[1])
        translation_origin_to_robot = self.drive_pose.translation()
        translation_robot_to_speaker = translation_origin_to_speaker - translation_origin_to_robot
        desired_angle = translation_robot_to_speaker.angle().rotateBy(geo.Rotation2d(np.radians(180)))
        angle_to_speaker = self.drive_pose.rotation().degrees() - desired_angle.degrees()
        shot_distance = np.sqrt((speaker_coords[0] - self.drive_pose.X())**2 + (speaker_coords[1] - self.drive_pose.Y())**2)

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

    def _format_pose_string(self, label, pose):
        """Formats a pose array into a display string with appropriate padding."""
        x, y, rot = pose.X(), pose.Y(), pose.rotation().degrees()
        x_pad = 1 if x < 10 else 0
        # This logic determines padding based on the number of digits in the angle
        theta_pad = sum([1 for t in [0, 100, 10] if abs(rot) < t])
        return f'{label}\n{" " * x_pad}{x:>5.2f}m {y:>4.2f}m {" " * theta_pad}{rot:>4.0f}°'

    def _update_pose_widget(self, widget, pixmap, pose, field_dims):
        """Updates a QLabel on the field graphic with a rotated pixmap and new position."""
        x, y, rot = pose.X(), pose.Y(), pose.rotation().degrees()
        width, height = field_dims

        pixmap_rotated = pixmap.transformed(QtGui.QTransform().rotate(90 - rot), QtCore.Qt.TransformationMode.SmoothTransformation)
        # This formula creates a pulsating effect as the robot rotates
        new_size = int(41 * (1 + 0.41 * np.abs(np.sin(2 * rot * np.pi / 180.0))))

        widget.resize(new_size, new_size)
        widget.setPixmap(pixmap_rotated)

        # Convert pose (meters) to widget coordinates (pixels)
        # Field dimensions in meters: 17.6 x 8.2
        widget_x = int(-new_size / 2 + width * x / 17.6)
        widget_y = int(-new_size / 2 + height * (1 - y / 8.2))
        widget.move(widget_x, widget_y)

    def _update_command_list(self):
        """Updates the background color of the command list widget in the NT tree - which is currently broken"""
        green, white = QtGui.QColor(227, 255, 227), QtGui.QColor(255, 255, 255)
        for i, props in enumerate(self.ui.command_dict.values()):
            sub = props.get('subscriber')
            if sub:
                is_running = sub.get()
                self.ui.qlistwidget_commands.item(i).setBackground(green if is_running else white)

    def _update_fps_counter(self):
        """Updates the status bar with GUI and camera FPS."""
        if self.ui.counter % 80 == 0:
            current_time = time.time()
            time_delta = current_time - self.ui.previous_time
            if time_delta > 0:
                frames = self.ui.camera_manager.worker.frames if self.ui.camera_manager.worker else 0
                
                # Calculate NT Latency (Data Age)
                # This measures time in ms since the last timestamp update was received
                # Use match_time as the heartbeat since _timestamp is gone
                match_time_sub = self.ui.widget_dict['qlabel_matchtime'].get('subscriber')
                latency = -1.0
                if match_time_sub:
                    atomic_ts = match_time_sub.getAtomic()
                    if atomic_ts.time != 0:
                        # ntcore._now() and atomic_ts.time are both in microseconds. Divide by 1000 for ms.
                        latency = (ntcore._now() - atomic_ts.time) / 1000.0
                
                msg = f'GUI: {80/time_delta:.1f}Hz | CAM: {(frames - self.ui.previous_frames)/time_delta:.1f}Hz | LATENCY: {latency:.1f}ms'
                self.ui.statusBar().showMessage(msg)
            self.ui.previous_time = current_time
            self.ui.previous_frames = frames

    # --------------------------------------------------------------------------
    # Dispatcher Functions (Called from the self.updaters dictionary)
    # --------------------------------------------------------------------------

    def _update_indicator(self, props):
        """ Updates a label with a style based on state of the boolean tied to it """
        # TODO - maybe separate this based on whether it's monitoring a robot state boolean vs a command currently running
        sub, widget = props.get('subscriber'), props.get('widget')
        if not (sub and widget):
            return

        is_on = sub.get()
        
        # Optimization: If value hasn't changed, skip update.
        # EXCEPTION: If the widget is ON and has 'flash' enabled, we must update because self.flash_on toggles.
        if is_on == props.get('last_value') and not (props.get('flash') and is_on):
            return
        props['last_value'] = is_on

        if 'style_on' in props:  # allow a custom style for each indicator
            style = props['style_on'] if is_on else props['style_off']
        elif 'flash' in props and is_on:
            style = self.STYLE_FLASH_ON if self.flash_on else self.STYLE_FLASH_OFF
        else:
            style = self.STYLE_ON if is_on else self.STYLE_OFF  # go with the default
        widget.setStyleSheet(style)

    def _update_lcd(self, props):
        """ Updates a numeric LCD widget with an integer """
        # TODO - replace these with a better custom LCD-font widget
        sub, widget = props.get('subscriber'), props.get('widget')
        if sub and widget:
            value = int(sub.get()) # Convert to int for display and comparison
            if value == props.get('last_value'):
                return
            props['last_value'] = value
            widget.display(str(value))

    def _update_monitor(self, props):
        """ Updates a warninglabel with "good" and "bad" colors (set in UI's init) based on the value """
        sub, widget = props.get('subscriber'), props.get('widget')
        if sub and widget:
            value = sub.get()
            if value == props.get('last_value'):
                return
            props['last_value'] = value
            widget.set_value(value)

    def _update_combo(self, props):
        """ At the moment this is just for the autonomous routines combo box but it could be for any dropdown """
        sub, widget = props.get('subscriber'), props.get('widget')
        # print(f'found combo on {props.get("widget_name")} at {self.ui.counter} with sub {sub} and widget {widget}')
        # there is a problem with using shortcut checks for validity - the combobox is not True if it is empty (len 0)
        # so just check for None
        if sub is None or widget is None:
            return

        new_list = sub.get()
        if new_list != props.get('last_value'):
            props['last_value'] = new_list
            widget.blockSignals(True)
            widget.clear()
            widget.addItems(new_list)
            widget.blockSignals(False)
            self.ui.autonomous_list = new_list

        selected_sub = props.get('selected_subscriber')
        if selected_sub:
            selected_routine = selected_sub.get()
            if selected_routine != props.get('last_selected_value'):
                props['last_selected_value'] = selected_routine
                widget.blockSignals(True)
                widget.setCurrentText(selected_routine)
                widget.blockSignals(False)

    def _update_time(self, props):
        """ This is for the match time remaining widget - lame in sim but correct for matches """
        sub, widget = props.get('subscriber'), props.get('widget')
        if not (sub and widget):
            return

        match_time = sub.get()
        # If time is low, we flash, so we must update. If time is high, we only update if the integer second changes.
        val_int = int(match_time)
        if match_time >= 30 and val_int == props.get('last_value'):
            return
        props['last_value'] = val_int

        if match_time < 30:
            widget.setText(f'* {int(match_time)} *')
            widget.setStyleSheet(self.STYLE_FLASH_ON if self.flash_on else self.STYLE_FLASH_OFF)
        else:
            widget.setText(str(int(match_time)))
            widget.setStyleSheet(self.STYLE_HIGH)

    def _update_position(self, props):
        """  This was specific for CrankSinatra's shot position """
        sub, widget = props.get('subscriber'), props.get('widget')
        if not (sub and widget):
            return

        config = sub.get()
        if config == props.get('last_value'):
            return
        props['last_value'] = config
        # This logic seems to always result in STYLE_ON, may need review
        position_style = self.STYLE_ON if config.upper() not in ['LOW_SHOOT', 'INTAKE'] else self.STYLE_ON
        widget.setText(f'POS: {config.upper()}')
        widget.setStyleSheet(position_style)

    def _update_hub(self, props):
        """Legacy hub update logic from 2023. May need removal."""
        hub_targets_sub = self.ui.widget_dict['hub_targets'].get('subscriber')
        if not hub_targets_sub:
            return

        hub_targets = hub_targets_sub.get()
        if hub_targets > 0:
            hub_rotation = self.ui.widget_dict['hub_rotation']['subscriber'].get() - 5
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
