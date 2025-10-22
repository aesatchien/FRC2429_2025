
def get_camera_dict(ui):
    """
    the camera dictionary shows the processed and raw cameras
    :param ui:
    :return:
    """
    return {
        'GeniusLow': {'URL': 'http://10.24.29.12:1186/stream.mjpg', 'IS_ALIVE': False, 'TIMESTAMP_ENTRY': ui.ntinst.getEntry('/Cameras/GeniusLow/_timestamp'),
                      'CONNECTIONS': 0, 'CONNECTIONS_ENTRY': ui.ntinst.getEntry('/Cameras/GeniusLow/_connections'), 'NICKNAME': 'GENIUS LO', 'INDICATOR': ui.qlabel_genius_low_indicator},
        'ArducamBack': {'URL': 'http://10.24.29.12:1187/stream.mjpg', 'IS_ALIVE': False, 'TIMESTAMP_ENTRY': ui.ntinst.getEntry('/Cameras/ArducamBack/_timestamp'),
                        'CONNECTIONS': 0, 'CONNECTIONS_ENTRY': ui.ntinst.getEntry('/Cameras/ArducamBack/_connections'), 'NICKNAME': 'ARDU BACK', 'INDICATOR': ui.qlabel_arducam_back_indicator},
        'LogitechReef': {'URL': 'http://10.24.29.13:1186/stream.mjpg', 'IS_ALIVE': False, 'TIMESTAMP_ENTRY': ui.ntinst.getEntry('/Cameras/LogitechReef/_timestamp'),
                         'CONNECTIONS': 0, 'CONNECTIONS_ENTRY': ui.ntinst.getEntry('/Cameras/LogitechReef/_connections'), 'NICKNAME': 'LOGI REEF', 'INDICATOR': ui.qlabel_logitech_reef_indicator},
        'ArducamHigh': {'URL': 'http://10.24.29.13:1187/stream.mjpg', 'IS_ALIVE': False, 'TIMESTAMP_ENTRY': ui.ntinst.getEntry('/Cameras/ArducamHigh/_timestamp'),
                        'CONNECTIONS': 0, 'CONNECTIONS_ENTRY': ui.ntinst.getEntry('/Cameras/ArducamHigh/_connections'), 'NICKNAME': 'ARDU HI', 'INDICATOR': ui.qlabel_arducam_high_indicator},
        'Raw GeniusLow': {'URL': 'http://10.24.29.12:1181/stream.mjpg'},
        'Raw ArduBack': {'URL': 'http://10.24.29.12:1182/stream.mjpg'},
        'Raw LogiReef': {'URL': 'http://10.24.29.13:1181/stream.mjpg'},
        'Raw ArduHigh': {'URL': 'http://10.24.29.13:1182/stream.mjpg'},
        'Debug': {'URL': 'http://127.0.0.1:1186/stream.mjpg'},
    }

def get_widget_dict(ui):
    return {
        # GUI UPDATES
        'drive_pose': {'widget': None, 'nt': '/SmartDashboard/drive_pose', 'command': None},
        'quest_pose': {'widget': None, 'nt': '/SmartDashboard/QUEST_POSE', 'command': None},
        'qcombobox_autonomous_routines': {'widget':ui.qcombobox_autonomous_routines, 'nt': r'/SmartDashboard/autonomous routines/options', 'command':None, 'selected': r'/SmartDashboard/autonomous routines/selected'},
        'qlabel_nt_connected': {'widget': ui.qlabel_nt_connected, 'nt': None, 'command': None},
        'qlabel_matchtime': {'widget': ui.qlabel_matchtime, 'nt': '/SmartDashboard/match_time', 'command': None},
        'qlabel_alliance_indicator': {'widget': ui.qlabel_alliance_indicator, 'nt': '/FMSInfo/IsRedAlliance', 'command': None,
                                            'style_on': "border: 7px; border-radius: 7px; background-color:rgb(225, 0, 0); color:rgb(200, 200, 200);",
                                            'style_off': "border: 7px; border-radius: 7px; background-color:rgb(0, 0, 225); color:rgb(200, 200, 200);"},
        'qlabel_camera_view': {'widget': ui.qlabel_camera_view, 'nt': None, 'command': None},
        'qlabel_arducam_high_indicator': {'widget': ui.qlabel_arducam_high_indicator, 'nt': None, 'command': None},
        'qlabel_logitech_reef_indicator': {'widget': ui.qlabel_logitech_reef_indicator, 'nt': None, 'command': None},
        'qlabel_genius_low_indicator': {'widget': ui.qlabel_genius_low_indicator, 'nt': None, 'command': None},
        'qlabel_arducam_back_indicator': {'widget': ui.qlabel_arducam_back_indicator, 'nt': None, 'command': None},

        # COMMANDS
        'qlabel_elevator_top_indicator': {'widget': ui.qlabel_elevator_top_indicator, 'nt': '/SmartDashboard/MoveElevatorTop/running', 'command': '/SmartDashboard/MoveElevatorTop/running'},
        'qlabel_elevator_up_indicator': {'widget': ui.qlabel_elevator_up_indicator, 'nt': '/SmartDashboard/MoveElevatorUp/running', 'command': '/SmartDashboard/MoveElevatorUp/running'},
        'qlabel_elevator_down_indicator': {'widget': ui.qlabel_elevator_down_indicator, 'nt': '/SmartDashboard/MoveElevatorDown/running', 'command': '/SmartDashboard/MoveElevatorDown/running'},
        'qlabel_pivot_up_indicator': {'widget': ui.qlabel_pivot_up_indicator, 'nt': '/SmartDashboard/MovePivotUp/running', 'command': '/SmartDashboard/MovePivotUp/running'},
        'qlabel_pivot_down_indicator': {'widget': ui.qlabel_pivot_down_indicator, 'nt': '/SmartDashboard/MovePivotDown/running', 'command': '/SmartDashboard/MovePivotDown/running'},
        'qlabel_wrist_up_indicator': {'widget': ui.qlabel_wrist_up_indicator, 'nt': '/SmartDashboard/MoveWristUp/running', 'command': '/SmartDashboard/MoveWristUp/running'},
        'qlabel_wrist_down_indicator': {'widget': ui.qlabel_wrist_down_indicator, 'nt': '/SmartDashboard/MoveWristDown/running', 'command': '/SmartDashboard/MoveWristDown/running'},
        'qlabel_intake_on_indicator': {'widget': ui.qlabel_intake_on_indicator, 'nt': '/SmartDashboard/IntakeOn/running', 'command': '/SmartDashboard/IntakeOn/running'},
        'qlabel_intake_off_indicator': {'widget': ui.qlabel_intake_off_indicator, 'nt': '/SmartDashboard/IntakeOff/running','command': '/SmartDashboard/IntakeOff/running'},
        'qlabel_intake_reverse_indicator': {'widget': ui.qlabel_intake_reverse_indicator, 'nt': '/SmartDashboard/IntakeReverse/running','command': '/SmartDashboard/IntakeReverse/running'},
        'qlabel_climber_down_indicator': {'widget': ui.qlabel_climber_down_indicator, 'nt': '/SmartDashboard/Move climber down/running', 'command': '/SmartDashboard/Move climber down/running'},
        'qlabel_climber_up_indicator': {'widget': ui.qlabel_climber_up_indicator, 'nt': '/SmartDashboard/Move climber up/running', 'command': '/SmartDashboard/Move climber up/running'},

        'qlabel_stow_indicator': {'widget': ui.qlabel_stow_indicator, 'nt': '/SmartDashboard/GoToStow/running', 'command': '/SmartDashboard/GoToStow/running'},
        'qlabel_score_indicator': {'widget': ui.qlabel_score_indicator, 'nt': '/SmartDashboard/Score/running', 'command': '/SmartDashboard/Score/running', 'flash':True},
        'qlabel_l1_indicator': {'widget': ui.qlabel_l1_indicator, 'nt': '/SmartDashboard/GoToL1/running', 'command': '/SmartDashboard/GoToL1/running'},
        'qlabel_l2_indicator': {'widget': ui.qlabel_l2_indicator, 'nt': '/SmartDashboard/GoToL2/running', 'command': '/SmartDashboard/GoToL2/running'},
        'qlabel_l3_indicator': {'widget': ui.qlabel_l3_indicator, 'nt': '/SmartDashboard/GoToL3/running', 'command': '/SmartDashboard/GoToL3/running'},
        'qlabel_l4_indicator': {'widget': ui.qlabel_l4_indicator, 'nt': '/SmartDashboard/GoToL4/running', 'command': '/SmartDashboard/GoToL4/running'},
        'qlabel_can_report_indicator': {'widget': ui.qlabel_can_report_indicator, 'nt': '/SmartDashboard/CANStatus/running', 'command': '/SmartDashboard/CANStatus/running'},
        'qlabel_reset_flex_indicator': {'widget': ui.qlabel_reset_flex_indicator, 'nt': '/SmartDashboard/ResetFlex/running', 'command': '/SmartDashboard/ResetFlex/running'},
        'qlabel_elevator_shift_up_indicator': {'widget': ui.qlabel_elevator_shift_up_indicator, 'nt': '/SmartDashboard/CalElevatorUp/running', 'command': '/SmartDashboard/CalElevatorUp/running'},
        'qlabel_elevator_shift_down_indicator': {'widget': ui.qlabel_elevator_shift_down_indicator, 'nt': '/SmartDashboard/CalElevatorDown/running', 'command': '/SmartDashboard/CalElevatorDown/running'},
        'qlabel_cal_wrist_indicator': {'widget': ui.qlabel_cal_wrist_indicator, 'nt': '/SmartDashboard/RecalWrist/running', 'command': '/SmartDashboard/RecalWrist/running'},
        'qlabel_cal_wrist_up_indicator': {'widget': ui.qlabel_cal_wrist_up_indicator, 'nt': '/SmartDashboard/CalWristUp/running', 'command': '/SmartDashboard/CalWristUp/running'},
        'qlabel_cal_wrist_down_indicator': {'widget': ui.qlabel_cal_wrist_down_indicator, 'nt': '/SmartDashboard/CalWristDown/running', 'command': '/SmartDashboard/CalWristDown/running'},

        'qlabel_questnav_heartbeat_indicator': {'widget': ui.qlabel_questnav_heartbeat_indicator, 'nt': '/SmartDashboard/QUEST_CONNECTED', 'command': None},
        'qlabel_questnav_inbounds_indicator': {'widget': ui.qlabel_questnav_inbounds_indicator, 'nt': '/SmartDashboard/QUEST_POSE_ACCEPTED', 'command': None},
        'qlabel_questnav_tracking_indicator': {'widget': ui.qlabel_questnav_tracking_indicator, 'nt': '/SmartDashboard/QUEST_TRACKING', 'command': None},
        'qlabel_questnav_sync_toggle_indicator': {'widget': ui.qlabel_questnav_sync_toggle_indicator, 'nt': '/SmartDashboard/questnav_synched', 'command': '/SmartDashboard/QuestSyncToggle/running'},
        'qlabel_questnav_reset_indicator': {'widget': ui.qlabel_questnav_reset_indicator, 'nt': '/SmartDashboard/QuestResetOdometry/running', 'command': '/SmartDashboard/QuestResetOdometry/running'},
        'qlabel_questnav_enabled_toggle_indicator': {'widget': ui.qlabel_questnav_enabled_toggle_indicator, 'nt': '/SmartDashboard/questnav_in_use', 'command': '/SmartDashboard/QuestEnableToggle/running',
                                        # 'style_on': "border: 7px; border-radius: 7px; background-color:rgb(0, 220, 220); color:rgb(250, 250, 250);",
                                        # 'style_off': "border: 7px; border-radius: 7px; background-color:rgb(127, 127, 127); color:rgb(0, 0, 0);"},
                                             },

        'qlabel_arducam_high_target_indicator': {'widget': ui.qlabel_arducam_high_target_indicator, 'nt': '/SmartDashboard/arducam_high_targets_exist', 'command': None},
        'qlabel_arducam_back_target_indicator': {'widget': ui.qlabel_arducam_back_target_indicator, 'nt': '/SmartDashboard/arducam_back_targets_exist', 'command': None},
        'qlabel_logitech_reef_target_indicator': {'widget': ui.qlabel_logitech_reef_target_indicator, 'nt': '/SmartDashboard/logitech_reef_targets_exist', 'command': None},
        'qlabel_genius_low_target_indicator': {'widget': ui.qlabel_genius_low_target_indicator, 'nt': '/SmartDashboard/genius_low_targets_exist', 'command': None},
        'qlabel_photoncam_target_indicator': {'widget': ui.qlabel_photoncam_target_indicator, 'nt': '/SmartDashboard/photoncam_targets_exist', 'command': None},
        'qlabel_pose_indicator': {'widget': ui.qlabel_pose_indicator, 'nt': None, 'command': None,},
        'qlabel_game_piece_indicator': {'widget': ui.qlabel_game_piece_indicator, 'nt': '/SmartDashboard/gamepiece_present', 'command': '/SmartDashboard/LedToggle/running',
                                        'style_on': "border: 7px; border-radius: 7px; background-color:rgb(0, 220, 220); color:rgb(250, 250, 250);",
                                        'style_off': "border: 7px; border-radius: 7px; background-color:rgb(127, 127, 127); color:rgb(0, 0, 0);"},
        # UNFINISHED for 2025
        'qlabel_navx_reset_indicator': {'widget': ui.qlabel_navx_reset_indicator, 'nt': '/SmartDashboard/GyroReset/running', 'command': '/SmartDashboard/GyroReset/running'},
        # 'qlabel_orange_target_indicator': {'widget': ui.qlabel_orange_target_indicator, 'nt': '/SmartDashboard/orange_targets_exist', 'command': None},

        'qlabel_position_indicator': {'widget': ui.qlabel_position_indicator, 'nt': '/SmartDashboard/_target', 'command': None},

        # NUMERIC INDICATORS - LCDS should be phased out since they are a legacy item (and not particularly cool anyway)
        'qlcd_navx_heading': {'widget': ui.qlcd_navx_heading, 'nt': '/SmartDashboard/_navx', 'command': None},
        'qlcd_elevator_height': {'widget': ui.qlcd_elevator_height, 'nt': '/SmartDashboard/elevator_spark_pos', 'command': None},
        'qlcd_pivot_angle': {'widget': ui.qlcd_pivot_angle, 'nt': '/SmartDashboard/profiled_pivot_spark_angle', 'command': None},
        'qlcd_wrist_angle': {'widget' :ui.qlcd_wrist_angle, 'nt': '/SmartDashboard/wrist relative encoder, degrees', 'command': None},
        'qlcd_intake_speed': {'widget': ui.qlcd_intake_speed, 'nt': '/SmartDashboard/intake_output', 'command': None},
        'qlcd_climber_position': {'widget': ui.qlcd_climber_position, 'nt': '/SmartDashboard/climber_spark_angle', 'command': None},
        'qlabel_pdh_voltage_monitor': {'widget': ui.qlabel_pdh_voltage_monitor, 'nt': '/SmartDashboard/_pdh_voltage', 'command': None},
        'qlabel_pdh_current_monitor': {'widget': ui.qlabel_pdh_current_monitor, 'nt': '/SmartDashboard/_pdh_current', 'command': None},

        # LEFTOVER TO SORT FROM 2023
        #'qlabel_align_to_target_indicator': {'widget': ui.qlabel_align_to_target_indicator, 'nt': '/SmartDashboard/AutoSetupScore/running', 'command': '/SmartDashboard/AutoSetupScore/running'},
        #'qlabel_arm_calibration_indicator': {'widget': ui.qlabel_arm_calibration_indicator, 'nt': '/SmartDashboard/ArmCalibration/running', 'command': '/SmartDashboard/ArmCalibration/running'},
        # 'qlabel_upper_pickup_indicator': {'widget': ui.qlabel_upper_pickup_indicator, 'nt': '/SmartDashboard/UpperSubstationPickup/running', 'command': '/SmartDashboard/UpperSubstationPickup/running'},
        'hub_targets': {'widget': None, 'nt': '/arducam_high//orange/targets', 'command': None},
        'hub_rotation': {'widget': None, 'nt': '/arducam_high//orange/rotation', 'command': None},
        'hub_distance': {'widget': None, 'nt': '/arducam_high//orange/distance', 'command': None},
    }
