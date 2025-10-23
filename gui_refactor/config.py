# This file contains the static configuration for the dashboard widgets and cameras.
# It should not import any runtime modules like PyQt or ntcore.

# this config will be used to bind the NT topics to entries we can use later
CAMERA_CONFIG = {
    'GeniusLow': {'URL': 'http://10.24.29.12:1186/stream.mjpg',
                  'TIMESTAMP_TOPIC': '/Cameras/GeniusLow/_timestamp',
                  'CONNECTIONS_TOPIC': '/Cameras/GeniusLow/_connections',
                  'NICKNAME': 'GENIUS LO',
                  'INDICATOR_NAME': 'qlabel_genius_low_indicator'},
    'ArducamBack': {'URL': 'http://10.24.29.12:1187/stream.mjpg',
                    'TIMESTAMP_TOPIC': '/Cameras/ArducamBack/_timestamp',
                    'CONNECTIONS_TOPIC': '/Cameras/ArducamBack/_connections',
                    'NICKNAME': 'ARDU BACK',
                    'INDICATOR_NAME': 'qlabel_arducam_back_indicator'},
    'LogitechReef': {'URL': 'http://10.24.29.13:1186/stream.mjpg',
                     'TIMESTAMP_TOPIC': '/Cameras/LogitechReef/_timestamp',
                     'CONNECTIONS_TOPIC': '/Cameras/LogitechReef/_connections',
                     'NICKNAME': 'LOGI REEF',
                     'INDICATOR_NAME': 'qlabel_logitech_reef_indicator'},
    'ArducamHigh': {'URL': 'http://10.24.29.13:1187/stream.mjpg',
                    'TIMESTAMP_TOPIC': '/Cameras/ArducamHigh/_timestamp',
                    'CONNECTIONS_TOPIC': '/Cameras/ArducamHigh/_connections',
                    'NICKNAME': 'ARDU HI',
                    'INDICATOR_NAME': 'qlabel_arducam_high_indicator'},
    'Raw GeniusLow': {'URL': 'http://10.24.29.12:1181/stream.mjpg'},
    'Raw ArduBack': {'URL': 'http://10.24.29.12:1182/stream.mjpg'},
    'Raw LogiReef': {'URL': 'http://10.24.29.13:1181/stream.mjpg'},
    'Raw ArduHigh': {'URL': 'http://10.24.29.13:1182/stream.mjpg'},
    'Debug': {'URL': 'http://127.0.0.1:1186/stream.mjpg'},
}

WIDGET_CONFIG = {
    # GUI UPDATES - NEED THIS PART FOR EVERY YEAR
    'drive_pose': {'widget_name': None, 'nt_topic': '/SmartDashboard/drive_pose', 'command_topic': None},
    'qcombobox_autonomous_routines': {'widget_name': 'qcombobox_autonomous_routines', 'nt_topic': r'/SmartDashboard/autonomous routines/options',
                                      'command_topic': None, 'selected_topic': r'/SmartDashboard/autonomous routines/selected'},
    'qlabel_nt_connected': {'widget_name': 'qlabel_nt_connected', 'nt_topic': None, 'command_topic': None},
    'qlabel_matchtime': {'widget_name': 'qlabel_matchtime', 'nt_topic': '/SmartDashboard/match_time', 'command_topic': None},
    'qlabel_alliance_indicator': {'widget_name': 'qlabel_alliance_indicator', 'nt_topic': '/FMSInfo/IsRedAlliance', 'command_topic': None,
                                        'style_on': "border: 7px; border-radius: 7px; background-color:rgb(225, 0, 0); color:rgb(200, 200, 200);",
                                        'style_off': "border: 7px; border-radius: 7px; background-color:rgb(0, 0, 225); color:rgb(200, 200, 200);"},
    'qlabel_camera_view': {'widget_name': 'qlabel_camera_view', 'nt_topic': None, 'command_topic': None},
    'qlabel_pose_indicator': {'widget_name': 'qlabel_pose_indicator', 'nt_topic': None, 'command_topic': None},
    'qlabel_pdh_voltage_monitor': {'widget_name': 'qlabel_pdh_voltage_monitor', 'nt_topic': '/SmartDashboard/_pdh_voltage', 'command_topic': None},
    'qlabel_pdh_current_monitor': {'widget_name': 'qlabel_pdh_current_monitor', 'nt_topic': '/SmartDashboard/_pdh_current', 'command_topic': None},

    # QUESTNAV STUFF
    'quest_pose': {'widget_name': None, 'nt_topic': '/SmartDashboard/QUEST_POSE', 'command_topic': None},
    'qlabel_questnav_heartbeat_indicator': {'widget_name': 'qlabel_questnav_heartbeat_indicator', 'nt_topic': '/SmartDashboard/QUEST_CONNECTED', 'command_topic': None},
    'qlabel_questnav_inbounds_indicator': {'widget_name': 'qlabel_questnav_inbounds_indicator', 'nt_topic': '/SmartDashboard/QUEST_POSE_ACCEPTED', 'command_topic': None},
    'qlabel_questnav_tracking_indicator': {'widget_name': 'qlabel_questnav_tracking_indicator', 'nt_topic': '/SmartDashboard/QUEST_TRACKING', 'command_topic': None},
    'qlabel_questnav_sync_toggle_indicator': {'widget_name': 'qlabel_questnav_sync_toggle_indicator', 'nt_topic': '/SmartDashboard/questnav_synched', 'command_topic': '/SmartDashboard/QuestSyncToggle/running'},
    'qlabel_questnav_reset_indicator': {'widget_name': 'qlabel_questnav_reset_indicator', 'nt_topic': '/SmartDashboard/QuestResetOdometry/running', 'command_topic': '/SmartDashboard/QuestResetOdometry/running'},
    'qlabel_questnav_enabled_toggle_indicator': {'widget_name': 'qlabel_questnav_enabled_toggle_indicator', 'nt_topic': '/SmartDashboard/questnav_in_use', 'command_topic': '/SmartDashboard/QuestEnableToggle/running'},

    # CAMERA INDICATORS - HEARTBEAT AND TARGETS AVAILABLE
    'qlabel_arducam_high_indicator': {'widget_name': 'qlabel_arducam_high_indicator', 'nt_topic': None, 'command_topic': None},
    'qlabel_logitech_reef_indicator': {'widget_name': 'qlabel_logitech_reef_indicator', 'nt_topic': None, 'command_topic': None},
    'qlabel_genius_low_indicator': {'widget_name': 'qlabel_genius_low_indicator', 'nt_topic': None, 'command_topic': None},
    'qlabel_arducam_back_indicator': {'widget_name': 'qlabel_arducam_back_indicator', 'nt_topic': None, 'command_topic': None},

    'qlabel_arducam_high_target_indicator': {'widget_name': 'qlabel_arducam_high_target_indicator', 'nt_topic': '/SmartDashboard/arducam_high_targets_exist', 'command_topic': None},
    'qlabel_arducam_back_target_indicator': {'widget_name': 'qlabel_arducam_back_target_indicator', 'nt_topic': '/SmartDashboard/arducam_back_targets_exist', 'command_topic': None},
    'qlabel_logitech_reef_target_indicator': {'widget_name': 'qlabel_logitech_reef_target_indicator', 'nt_topic': '/SmartDashboard/logitech_reef_targets_exist', 'command_topic': None},
    'qlabel_genius_low_target_indicator': {'widget_name': 'qlabel_genius_low_target_indicator', 'nt_topic': '/SmartDashboard/genius_low_targets_exist', 'command_topic': None},
    'qlabel_photoncam_target_indicator': {'widget_name': 'qlabel_photoncam_target_indicator', 'nt_topic': '/SmartDashboard/photoncam_targets_exist', 'command_topic': None},


    # COMMANDS  - MOST LIKELY WILL CHANGE EVERY YEAR
    'qlabel_elevator_top_indicator': {'widget_name': 'qlabel_elevator_top_indicator', 'nt_topic': '/SmartDashboard/MoveElevatorTop/running', 'command_topic': '/SmartDashboard/MoveElevatorTop/running'},
    'qlabel_elevator_up_indicator': {'widget_name': 'qlabel_elevator_up_indicator', 'nt_topic': '/SmartDashboard/MoveElevatorUp/running', 'command_topic': '/SmartDashboard/MoveElevatorUp/running'},
    'qlabel_elevator_down_indicator': {'widget_name': 'qlabel_elevator_down_indicator', 'nt_topic': '/SmartDashboard/MoveElevatorDown/running', 'command_topic': '/SmartDashboard/MoveElevatorDown/running'},
    'qlabel_pivot_up_indicator': {'widget_name': 'qlabel_pivot_up_indicator', 'nt_topic': '/SmartDashboard/MovePivotUp/running', 'command_topic': '/SmartDashboard/MovePivotUp/running'},
    'qlabel_pivot_down_indicator': {'widget_name': 'qlabel_pivot_down_indicator', 'nt_topic': '/SmartDashboard/MovePivotDown/running', 'command_topic': '/SmartDashboard/MovePivotDown/running'},
    'qlabel_wrist_up_indicator': {'widget_name': 'qlabel_wrist_up_indicator', 'nt_topic': '/SmartDashboard/MoveWristUp/running', 'command_topic': '/SmartDashboard/MoveWristUp/running'},
    'qlabel_wrist_down_indicator': {'widget_name': 'qlabel_wrist_down_indicator', 'nt_topic': '/SmartDashboard/MoveWristDown/running', 'command_topic': '/SmartDashboard/MoveWristDown/running'},
    'qlabel_intake_on_indicator': {'widget_name': 'qlabel_intake_on_indicator', 'nt_topic': '/SmartDashboard/IntakeOn/running', 'command_topic': '/SmartDashboard/IntakeOn/running'},
    'qlabel_intake_off_indicator': {'widget_name': 'qlabel_intake_off_indicator', 'nt_topic': '/SmartDashboard/IntakeOff/running','command_topic': '/SmartDashboard/IntakeOff/running'},
    'qlabel_intake_reverse_indicator': {'widget_name': 'qlabel_intake_reverse_indicator', 'nt_topic': '/SmartDashboard/IntakeReverse/running','command_topic': '/SmartDashboard/IntakeReverse/running'},
    'qlabel_climber_down_indicator': {'widget_name': 'qlabel_climber_down_indicator', 'nt_topic': '/SmartDashboard/Move climber down/running', 'command_topic': '/SmartDashboard/Move climber down/running'},
    'qlabel_climber_up_indicator': {'widget_name': 'qlabel_climber_up_indicator', 'nt_topic': '/SmartDashboard/Move climber up/running', 'command_topic': '/SmartDashboard/Move climber up/running'},
    'qlabel_stow_indicator': {'widget_name': 'qlabel_stow_indicator', 'nt_topic': '/SmartDashboard/GoToStow/running', 'command_topic': '/SmartDashboard/GoToStow/running'},
    'qlabel_score_indicator': {'widget_name': 'qlabel_score_indicator', 'nt_topic': '/SmartDashboard/Score/running', 'command_topic': '/SmartDashboard/Score/running', 'flash':True},
    'qlabel_l1_indicator': {'widget_name': 'qlabel_l1_indicator', 'nt_topic': '/SmartDashboard/GoToL1/running', 'command_topic': '/SmartDashboard/GoToL1/running'},
    'qlabel_l2_indicator': {'widget_name': 'qlabel_l2_indicator', 'nt_topic': '/SmartDashboard/GoToL2/running', 'command_topic': '/SmartDashboard/GoToL2/running'},
    'qlabel_l3_indicator': {'widget_name': 'qlabel_l3_indicator', 'nt_topic': '/SmartDashboard/GoToL3/running', 'command_topic': '/SmartDashboard/GoToL3/running'},
    'qlabel_l4_indicator': {'widget_name': 'qlabel_l4_indicator', 'nt_topic': '/SmartDashboard/GoToL4/running', 'command_topic': '/SmartDashboard/GoToL4/running'},
    'qlabel_can_report_indicator': {'widget_name': 'qlabel_can_report_indicator', 'nt_topic': '/SmartDashboard/CANStatus/running', 'command_topic': '/SmartDashboard/CANStatus/running'},
    'qlabel_reset_flex_indicator': {'widget_name': 'qlabel_reset_flex_indicator', 'nt_topic': '/SmartDashboard/ResetFlex/running', 'command_topic': '/SmartDashboard/ResetFlex/running'},
    'qlabel_elevator_shift_up_indicator': {'widget_name': 'qlabel_elevator_shift_up_indicator', 'nt_topic': '/SmartDashboard/CalElevatorUp/running', 'command_topic': '/SmartDashboard/CalElevatorUp/running'},
    'qlabel_elevator_shift_down_indicator': {'widget_name': 'qlabel_elevator_shift_down_indicator', 'nt_topic': '/SmartDashboard/CalElevatorDown/running', 'command_topic': '/SmartDashboard/CalElevatorDown/running'},
    'qlabel_cal_wrist_indicator': {'widget_name': 'qlabel_cal_wrist_indicator', 'nt_topic': '/SmartDashboard/RecalWrist/running', 'command_topic': '/SmartDashboard/RecalWrist/running'},
    'qlabel_cal_wrist_up_indicator': {'widget_name': 'qlabel_cal_wrist_up_indicator', 'nt_topic': '/SmartDashboard/CalWristUp/running', 'command_topic': '/SmartDashboard/CalWristUp/running'},
    'qlabel_cal_wrist_down_indicator': {'widget_name': 'qlabel_cal_wrist_down_indicator', 'nt_topic': '/SmartDashboard/CalWristDown/running', 'command_topic': '/SmartDashboard/CalWristDown/running'},

    'qlabel_game_piece_indicator': {'widget_name': 'qlabel_game_piece_indicator', 'nt_topic': '/SmartDashboard/gamepiece_present', 'command_topic': '/SmartDashboard/LedToggle/running',
                                        'style_on': "border: 7px; border-radius: 7px; background-color:rgb(0, 220, 220); color:rgb(250, 250, 250);",
                                        'style_off': "border: 7px; border-radius: 7px; background-color:rgb(127, 127, 127); color:rgb(0, 0, 0);"},
    'qlabel_navx_reset_indicator': {'widget_name': 'qlabel_navx_reset_indicator', 'nt_topic': '/SmartDashboard/GyroReset/running', 'command_topic': '/SmartDashboard/GyroReset/running'},
    'qlabel_position_indicator': {'widget_name': 'qlabel_position_indicator', 'nt_topic': '/SmartDashboard/_target', 'command_topic': None},

    # NUMERIC INDICATORS - I HAVE BEEN USING THE LCD FOR THIS BUT THERE BUST BE A BETTER WAY TO SHOW NUMBERS
    'qlcd_navx_heading': {'widget_name': 'qlcd_navx_heading', 'nt_topic': '/SmartDashboard/_navx', 'command_topic': None},
    'qlcd_elevator_height': {'widget_name': 'qlcd_elevator_height', 'nt_topic': '/SmartDashboard/elevator_spark_pos', 'command_topic': None},
    'qlcd_pivot_angle': {'widget_name': 'qlcd_pivot_angle', 'nt_topic': '/SmartDashboard/profiled_pivot_spark_angle', 'command_topic': None},
    'qlcd_wrist_angle': {'widget_name' :'qlcd_wrist_angle', 'nt_topic': '/SmartDashboard/wrist relative encoder, degrees', 'command_topic': None},
    'qlcd_intake_speed': {'widget_name': 'qlcd_intake_speed', 'nt_topic': '/SmartDashboard/intake_output', 'command_topic': None},
    'qlcd_climber_position': {'widget_name': 'qlcd_climber_position', 'nt_topic': '/SmartDashboard/climber_spark_angle', 'command_topic': None},

    # LEFTOVER TO SORT FROM 2023
    'hub_targets': {'widget_name': None, 'nt_topic': '/arducam_high//orange/targets', 'command_topic': None},
    'hub_rotation': {'widget_name': None, 'nt_topic': '/arducam_high//orange/rotation', 'command_topic': None},
    'hub_distance': {'widget_name': None, 'nt_topic': '/arducam_high//orange/distance', 'command_topic': None},
}
