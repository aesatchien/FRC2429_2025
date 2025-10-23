"""
This file contains the static configuration for the dashboard widgets and cameras.
It decouples the widget definitions from the application logic, allowing for easier
maintenance and modification.

WIDGET_CONFIG Structure:
------------------------
The WIDGET_CONFIG dictionary is the central registry for all interactive or
data-driven widgets in the UI. Each key in the dictionary is a unique identifier
for a widget configuration, and the value is another dictionary containing the
properties that define the widget's behavior.

Each widget property dictionary can contain the following keys:

- widget_name (str): The 'objectName' of the widget as defined in Qt Designer.
  This is used to find the actual widget instance in the UI.

- nt_topic (str): The primary NetworkTables topic that the widget subscribes to
  for data. For example, a voltage monitor would subscribe to '/SmartDashboard/_pdh_voltage'.

- command_topic (str, optional): For clickable widgets, this is the NetworkTables
  topic to which a new value is written when the widget is clicked. Typically,
  this is the '/running' topic of a command published with `SmartDashboard.putData()`.

- selected_topic (str, optional): Specifically for chooser/combo box widgets, this
  is the NT topic that stores the currently selected option. The `nt_topic` will
  contain the list of options, while this topic holds the active choice.

- update_style (str): A key that maps to a specific function in the UIUpdater class.
  This determines *how* the widget is updated. Examples include:
    - 'indicator': For boolean status labels (on/off).
    - 'lcd': For QLCDNumber widgets.
    - 'monitor': For custom WarningLabel widgets that display numeric values and colored backgrounds.
    - 'time': For the match timer.
    - 'combo': For QComboBox autonomous choosers.
    - 'pose': For updating the robot's position on the field view.
    - 'position': For displaying the robot's arm/mechanism configuration.

- style_on (str, optional): A custom CSS string for an 'indicator' widget when its
  state is ON (True). If not provided, a default green style is used.

- style_off (str, optional): A custom CSS string for an 'indicator' widget when its
  state is OFF (False). If not provided, a default red style is used.

- flash (bool, optional): If set to True for an 'indicator' widget, it will
  flash when its state is ON.

"""
# This file should not import any runtime modules like PyQt or ntcore.

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
    'drive_pose': {'widget_name': 'qlabel_pose_indicator', 'nt_topic': '/SmartDashboard/drive_pose', 'update_style': 'pose'},
    'qcombobox_autonomous_routines': {'widget_name': 'qcombobox_autonomous_routines', 'nt_topic': r'/SmartDashboard/autonomous routines/options',
                                      'selected_topic': r'/SmartDashboard/autonomous routines/selected', 'update_style': 'combo'},
    'qlabel_nt_connected': {'widget_name': 'qlabel_nt_connected', 'update_style': 'connection'},
    'qlabel_matchtime': {'widget_name': 'qlabel_matchtime', 'nt_topic': '/SmartDashboard/match_time', 'update_style': 'time'},
    'qlabel_alliance_indicator': {'widget_name': 'qlabel_alliance_indicator', 'nt_topic': '/FMSInfo/IsRedAlliance', 'update_style': 'indicator',
                                        'style_on': "border: 7px; border-radius: 7px; background-color:rgb(225, 0, 0); color:rgb(200, 200, 200);",
                                        'style_off': "border: 7px; border-radius: 7px; background-color:rgb(0, 0, 225); color:rgb(200, 200, 200);"},
    # 'qlabel_camera_view': {'widget_name': 'qlabel_camera_view'},  # this isn't necessary, is it?
    'qlabel_pdh_voltage_monitor': {'widget_name': 'qlabel_pdh_voltage_monitor', 'nt_topic': '/SmartDashboard/_pdh_voltage', 'update_style': 'monitor'},
    'qlabel_pdh_current_monitor': {'widget_name': 'qlabel_pdh_current_monitor', 'nt_topic': '/SmartDashboard/_pdh_current', 'update_style': 'monitor'},

    # QUESTNAV STUFF
    'quest_pose': {'widget_name': 'qlabel_quest_pose_indicator', 'nt_topic': '/SmartDashboard/QUEST_POSE', 'update_style': 'pose'},
    'qlabel_questnav_heartbeat_indicator': {'widget_name': 'qlabel_questnav_heartbeat_indicator', 'nt_topic': '/SmartDashboard/QUEST_CONNECTED', 'update_style': 'indicator'},
    'qlabel_questnav_inbounds_indicator': {'widget_name': 'qlabel_questnav_inbounds_indicator', 'nt_topic': '/SmartDashboard/QUEST_POSE_ACCEPTED', 'update_style': 'indicator'},
    'qlabel_questnav_tracking_indicator': {'widget_name': 'qlabel_questnav_tracking_indicator', 'nt_topic': '/SmartDashboard/QUEST_TRACKING', 'update_style': 'indicator'},
    'qlabel_questnav_sync_toggle_indicator': {'widget_name': 'qlabel_questnav_sync_toggle_indicator', 'nt_topic': '/SmartDashboard/questnav_synched', 'command_topic': '/SmartDashboard/QuestSyncToggle/running', 'update_style': 'indicator'},
    'qlabel_questnav_reset_indicator': {'widget_name': 'qlabel_questnav_reset_indicator', 'nt_topic': '/SmartDashboard/QuestResetOdometry/running', 'command_topic': '/SmartDashboard/QuestResetOdometry/running', 'update_style': 'indicator'},
    'qlabel_questnav_enabled_toggle_indicator': {'widget_name': 'qlabel_questnav_enabled_toggle_indicator', 'nt_topic': '/SmartDashboard/questnav_in_use', 'command_topic': '/SmartDashboard/QuestEnableToggle/running', 'update_style': 'indicator'},

    # CAMERA INDICATORS - HEARTBEAT AND TARGETS AVAILABLE
    'qlabel_arducam_high_indicator': {'widget_name': 'qlabel_arducam_high_indicator', 'update_style': 'camera_indicator'},
    'qlabel_logitech_reef_indicator': {'widget_name': 'qlabel_logitech_reef_indicator', 'update_style': 'camera_indicator'},
    'qlabel_genius_low_indicator': {'widget_name': 'qlabel_genius_low_indicator', 'update_style': 'camera_indicator'},
    'qlabel_arducam_back_indicator': {'widget_name': 'qlabel_arducam_back_indicator', 'update_style': 'camera_indicator'},

    'qlabel_arducam_high_target_indicator': {'widget_name': 'qlabel_arducam_high_target_indicator', 'nt_topic': '/SmartDashboard/arducam_high_targets_exist', 'update_style': 'indicator'},
    'qlabel_arducam_back_target_indicator': {'widget_name': 'qlabel_arducam_back_target_indicator', 'nt_topic': '/SmartDashboard/arducam_back_targets_exist', 'update_style': 'indicator'},
    'qlabel_logitech_reef_target_indicator': {'widget_name': 'qlabel_logitech_reef_target_indicator', 'nt_topic': '/SmartDashboard/logitech_reef_targets_exist', 'update_style': 'indicator'},
    'qlabel_genius_low_target_indicator': {'widget_name': 'qlabel_genius_low_target_indicator', 'nt_topic': '/SmartDashboard/genius_low_targets_exist', 'update_style': 'indicator'},
    'qlabel_photoncam_target_indicator': {'widget_name': 'qlabel_photoncam_target_indicator', 'nt_topic': '/SmartDashboard/photoncam_targets_exist', 'update_style': 'indicator'},


    # COMMANDS  - MOST LIKELY WILL CHANGE EVERY YEAR
    'qlabel_elevator_top_indicator': {'widget_name': 'qlabel_elevator_top_indicator', 'nt_topic': '/SmartDashboard/MoveElevatorTop/running', 'command_topic': '/SmartDashboard/MoveElevatorTop/running', 'update_style': 'indicator'},
    'qlabel_elevator_up_indicator': {'widget_name': 'qlabel_elevator_up_indicator', 'nt_topic': '/SmartDashboard/MoveElevatorUp/running', 'command_topic': '/SmartDashboard/MoveElevatorUp/running', 'update_style': 'indicator'},
    'qlabel_elevator_down_indicator': {'widget_name': 'qlabel_elevator_down_indicator', 'nt_topic': '/SmartDashboard/MoveElevatorDown/running', 'command_topic': '/SmartDashboard/MoveElevatorDown/running', 'update_style': 'indicator'},
    'qlabel_pivot_up_indicator': {'widget_name': 'qlabel_pivot_up_indicator', 'nt_topic': '/SmartDashboard/MovePivotUp/running', 'command_topic': '/SmartDashboard/MovePivotUp/running', 'update_style': 'indicator'},
    'qlabel_pivot_down_indicator': {'widget_name': 'qlabel_pivot_down_indicator', 'nt_topic': '/SmartDashboard/MovePivotDown/running', 'command_topic': '/SmartDashboard/MovePivotDown/running', 'update_style': 'indicator'},
    'qlabel_wrist_up_indicator': {'widget_name': 'qlabel_wrist_up_indicator', 'nt_topic': '/SmartDashboard/MoveWristUp/running', 'command_topic': '/SmartDashboard/MoveWristUp/running', 'update_style': 'indicator'},
    'qlabel_wrist_down_indicator': {'widget_name': 'qlabel_wrist_down_indicator', 'nt_topic': '/SmartDashboard/MoveWristDown/running', 'command_topic': '/SmartDashboard/MoveWristDown/running', 'update_style': 'indicator'},
    'qlabel_intake_on_indicator': {'widget_name': 'qlabel_intake_on_indicator', 'nt_topic': '/SmartDashboard/IntakeOn/running', 'command_topic': '/SmartDashboard/IntakeOn/running', 'update_style': 'indicator'},
    'qlabel_intake_off_indicator': {'widget_name': 'qlabel_intake_off_indicator', 'nt_topic': '/SmartDashboard/IntakeOff/running','command_topic': '/SmartDashboard/IntakeOff/running', 'update_style': 'indicator'},
    'qlabel_intake_reverse_indicator': {'widget_name': 'qlabel_intake_reverse_indicator', 'nt_topic': '/SmartDashboard/IntakeReverse/running','command_topic': '/SmartDashboard/IntakeReverse/running', 'update_style': 'indicator'},
    'qlabel_climber_down_indicator': {'widget_name': 'qlabel_climber_down_indicator', 'nt_topic': '/SmartDashboard/Move climber down/running', 'command_topic': '/SmartDashboard/Move climber down/running', 'update_style': 'indicator'},
    'qlabel_climber_up_indicator': {'widget_name': 'qlabel_climber_up_indicator', 'nt_topic': '/SmartDashboard/Move climber up/running', 'command_topic': '/SmartDashboard/Move climber up/running', 'update_style': 'indicator'},
    'qlabel_stow_indicator': {'widget_name': 'qlabel_stow_indicator', 'nt_topic': '/SmartDashboard/GoToStow/running', 'command_topic': '/SmartDashboard/GoToStow/running', 'update_style': 'indicator'},
    'qlabel_score_indicator': {'widget_name': 'qlabel_score_indicator', 'nt_topic': '/SmartDashboard/Score/running', 'command_topic': '/SmartDashboard/Score/running', 'flash':True, 'update_style': 'indicator'},
    'qlabel_l1_indicator': {'widget_name': 'qlabel_l1_indicator', 'nt_topic': '/SmartDashboard/GoToL1/running', 'command_topic': '/SmartDashboard/GoToL1/running', 'update_style': 'indicator'},
    'qlabel_l2_indicator': {'widget_name': 'qlabel_l2_indicator', 'nt_topic': '/SmartDashboard/GoToL2/running', 'command_topic': '/SmartDashboard/GoToL2/running', 'update_style': 'indicator'},
    'qlabel_l3_indicator': {'widget_name': 'qlabel_l3_indicator', 'nt_topic': '/SmartDashboard/GoToL3/running', 'command_topic': '/SmartDashboard/GoToL3/running', 'update_style': 'indicator'},
    'qlabel_l4_indicator': {'widget_name': 'qlabel_l4_indicator', 'nt_topic': '/SmartDashboard/GoToL4/running', 'command_topic': '/SmartDashboard/GoToL4/running', 'update_style': 'indicator'},
    'qlabel_can_report_indicator': {'widget_name': 'qlabel_can_report_indicator', 'nt_topic': '/SmartDashboard/CANStatus/running', 'command_topic': '/SmartDashboard/CANStatus/running', 'update_style': 'indicator'},
    'qlabel_reset_flex_indicator': {'widget_name': 'qlabel_reset_flex_indicator', 'nt_topic': '/SmartDashboard/ResetFlex/running', 'command_topic': '/SmartDashboard/ResetFlex/running', 'update_style': 'indicator'},
    'qlabel_elevator_shift_up_indicator': {'widget_name': 'qlabel_elevator_shift_up_indicator', 'nt_topic': '/SmartDashboard/CalElevatorUp/running', 'command_topic': '/SmartDashboard/CalElevatorUp/running', 'update_style': 'indicator'},
    'qlabel_elevator_shift_down_indicator': {'widget_name': 'qlabel_elevator_shift_down_indicator', 'nt_topic': '/SmartDashboard/CalElevatorDown/running', 'command_topic': '/SmartDashboard/CalElevatorDown/running', 'update_style': 'indicator'},
    'qlabel_cal_wrist_indicator': {'widget_name': 'qlabel_cal_wrist_indicator', 'nt_topic': '/SmartDashboard/RecalWrist/running', 'command_topic': '/SmartDashboard/RecalWrist/running', 'update_style': 'indicator'},
    'qlabel_cal_wrist_up_indicator': {'widget_name': 'qlabel_cal_wrist_up_indicator', 'nt_topic': '/SmartDashboard/CalWristUp/running', 'command_topic': '/SmartDashboard/CalWristUp/running', 'update_style': 'indicator'},
    'qlabel_cal_wrist_down_indicator': {'widget_name': 'qlabel_cal_wrist_down_indicator', 'nt_topic': '/SmartDashboard/CalWristDown/running', 'command_topic': '/SmartDashboard/CalWristDown/running', 'update_style': 'indicator'},

    'qlabel_game_piece_indicator': {'widget_name': 'qlabel_game_piece_indicator', 'nt_topic': '/SmartDashboard/gamepiece_present', 'command_topic': '/SmartDashboard/LedToggle/running', 'update_style': 'indicator',
                                        'style_on': "border: 7px; border-radius: 7px; background-color:rgb(0, 220, 220); color:rgb(250, 250, 250);",
                                        'style_off': "border: 7px; border-radius: 7px; background-color:rgb(127, 127, 127); color:rgb(0, 0, 0);"},
    'qlabel_navx_reset_indicator': {'widget_name': 'qlabel_navx_reset_indicator', 'nt_topic': '/SmartDashboard/GyroReset/running', 'command_topic': '/SmartDashboard/GyroReset/running', 'update_style': 'indicator'},


    # NUMERIC INDICATORS - I HAVE BEEN USING THE LCD FOR THIS BUT THERE BUST BE A BETTER WAY TO SHOW NUMBERS
    # TODO - write a pretty custom indicator for these numbers using a robot font, black background, and red numbers
    'qlcd_navx_heading': {'widget_name': 'qlcd_navx_heading', 'nt_topic': '/SmartDashboard/_navx', 'update_style': 'lcd'},
    'qlcd_elevator_height': {'widget_name': 'qlcd_elevator_height', 'nt_topic': '/SmartDashboard/elevator_spark_pos', 'update_style': 'lcd'},
    'qlcd_pivot_angle': {'widget_name': 'qlcd_pivot_angle', 'nt_topic': '/SmartDashboard/profiled_pivot_spark_angle', 'update_style': 'lcd'},
    'qlcd_wrist_angle': {'widget_name' :'qlcd_wrist_angle', 'nt_topic': '/SmartDashboard/wrist relative encoder, degrees', 'update_style': 'lcd'},
    'qlcd_intake_speed': {'widget_name': 'qlcd_intake_speed', 'nt_topic': '/SmartDashboard/intake_output', 'update_style': 'lcd'},
    'qlcd_climber_position': {'widget_name': 'qlcd_climber_position', 'nt_topic': '/SmartDashboard/climber_spark_angle', 'update_style': 'lcd'},

    # LEFTOVER TO SORT FROM previous years - legacy but infrastructure is there
    'qlabel_position_indicator': {'widget_name': 'qlabel_position_indicator', 'nt_topic': '/SmartDashboard/_target', 'update_style': 'position'},
    'hub_targets': {'widget_name': None, 'nt_topic': '/arducam_high//orange/targets', 'update_style': 'hub'},
    'hub_rotation': {'widget_name': None, 'nt_topic': '/arducam_high//orange/rotation', 'update_style': 'hub'},
    'hub_distance': {'widget_name': None, 'nt_topic': '/arducam_high//orange/distance', 'update_style': 'hub'},
}
