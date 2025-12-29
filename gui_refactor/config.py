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
# todo - somehow make the camera names all update from a config file, but that means ui and robot code need to know
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

camera_prefix = r'/Cameras'  # from the pis
quest_prefix = r'/QuestNav'  # putting this on par with the cameras as an external system
# systems inside/from the robot
status_prefix = r'/SmartDashboard/RobotStatus'  # the default for any status message
vision_prefix = r'/SmartDashboard/Vision'  # from the robot
swerve_prefix = r'/SmartDashboard/Swerve'  # from the robot
sim_prefix = r'/SmartDashboard/Sim'  # from the sim (still from the robot)
command_prefix = r'/SmartDashboard/Command'  # DIFFERENT FROM ROBOT CODE: the robot SmartDashboard.putData auto prepends /SmartDashboard to the key
base_prefix = '/SmartDashboard'  #  TODO - eventually nothing should be in here

WIDGET_CONFIG = {
    # GUI UPDATES - NEED THIS PART FOR EVERY YEAR  - AT THE MOMENT I AM LEAVING A FEW OF THEM AS THE BASE PREFIX
    'drive_pose': {'widget_name': 'qlabel_pose_indicator', 'nt_topic': f'{swerve_prefix}/drive_pose', 'update_style': 'pose'},
    'qcombobox_autonomous_routines': {'widget_name': 'qcombobox_autonomous_routines', 'nt_topic': rf'{base_prefix}/autonomous routines/options',
                                      'selected_topic': rf'{base_prefix}/autonomous routines/selected', 'update_style': 'combo'},
    'qlabel_nt_connected': {'widget_name': 'qlabel_nt_connected', 'update_style': 'connection'},
    'qlabel_matchtime': {'widget_name': 'qlabel_matchtime', 'nt_topic': f'{base_prefix}/match_time', 'update_style': 'time'},
    'qlabel_alliance_indicator': {'widget_name': 'qlabel_alliance_indicator', 'nt_topic': '/FMSInfo/IsRedAlliance', 'update_style': 'indicator',
                                        'style_on': "border: 7px; border-radius: 7px; background-color:rgb(225, 0, 0); color:rgb(200, 200, 200);",
                                        'style_off': "border: 7px; border-radius: 7px; background-color:rgb(0, 0, 225); color:rgb(200, 200, 200);"},
    # 'qlabel_camera_view': {'widget_name': 'qlabel_camera_view'},  # this isn't necessary, is it?

    # ROBOT STATUS
    'qlabel_pdh_voltage_monitor': {'widget_name': 'qlabel_pdh_voltage_monitor', 'nt_topic': f'{status_prefix}/_pdh_voltage', 'update_style': 'monitor'},
    'qlabel_pdh_current_monitor': {'widget_name': 'qlabel_pdh_current_monitor', 'nt_topic': f'{status_prefix}/_pdh_current', 'update_style': 'monitor'},

    # QUESTNAV STUFF  TODO - decide if quest commands should live in robot tree or questnav tree
    'quest_pose': {'widget_name': 'qlabel_quest_pose_indicator', 'nt_topic': f'{quest_prefix}/QUEST_POSE', 'update_style': 'pose'},
    'qlabel_questnav_heartbeat_indicator': {'widget_name': 'qlabel_questnav_heartbeat_indicator', 'nt_topic': f'{quest_prefix}/QUEST_CONNECTED', 'update_style': 'indicator'},
    'qlabel_questnav_inbounds_indicator': {'widget_name': 'qlabel_questnav_inbounds_indicator', 'nt_topic': f'{quest_prefix}/QUEST_POSE_ACCEPTED', 'update_style': 'indicator'},
    'qlabel_questnav_tracking_indicator': {'widget_name': 'qlabel_questnav_tracking_indicator', 'nt_topic': f'{quest_prefix}/QUEST_TRACKING', 'update_style': 'indicator'},
    'qlabel_questnav_sync_toggle_indicator': {'widget_name': 'qlabel_questnav_sync_toggle_indicator', 'nt_topic': f'{quest_prefix}/questnav_synched', 'command_topic': f'{command_prefix}/QuestSyncToggle/running', 'update_style': 'indicator'},
    'qlabel_questnav_reset_indicator': {'widget_name': 'qlabel_questnav_reset_indicator', 'nt_topic': f'{quest_prefix}/QuestResetOdometry/running', 'command_topic': f'{command_prefix}/QuestResetOdometry/running', 'update_style': 'indicator'},
    'qlabel_questnav_enabled_toggle_indicator': {'widget_name': 'qlabel_questnav_enabled_toggle_indicator', 'nt_topic': f'{quest_prefix}/questnav_in_use', 'command_topic': f'{command_prefix}/QuestEnableToggle/running', 'update_style': 'indicator'},

    # CAMERA INDICATORS - HEARTBEAT AND TARGETS AVAILABLE  -  THESE HAVE NO NT TOPICS BECAUSE WE DO IT IN _update_camera_indicators
    # HEARTBEATS
    'qlabel_arducam_high_indicator': {'widget_name': 'qlabel_arducam_high_indicator', 'update_style': 'camera_indicator'},
    'qlabel_logitech_reef_indicator': {'widget_name': 'qlabel_logitech_reef_indicator', 'update_style': 'camera_indicator'},
    'qlabel_genius_low_indicator': {'widget_name': 'qlabel_genius_low_indicator', 'update_style': 'camera_indicator'},
    'qlabel_arducam_back_indicator': {'widget_name': 'qlabel_arducam_back_indicator', 'update_style': 'camera_indicator'},

    # TARGETS AVAILABLE
    'qlabel_arducam_high_target_indicator': {'widget_name': 'qlabel_arducam_high_target_indicator', 'nt_topic': f'{vision_prefix}/arducam_high_targets_exist', 'update_style': 'indicator'},
    'qlabel_arducam_back_target_indicator': {'widget_name': 'qlabel_arducam_back_target_indicator', 'nt_topic': f'{vision_prefix}/arducam_back_targets_exist', 'update_style': 'indicator'},
    'qlabel_logitech_reef_target_indicator': {'widget_name': 'qlabel_logitech_reef_target_indicator', 'nt_topic': f'{vision_prefix}/logitech_reef_targets_exist', 'update_style': 'indicator'},
    'qlabel_genius_low_target_indicator': {'widget_name': 'qlabel_genius_low_target_indicator', 'nt_topic': f'{vision_prefix}/genius_low_targets_exist', 'update_style': 'indicator'},
    'qlabel_photoncam_target_indicator': {'widget_name': 'qlabel_photoncam_target_indicator', 'nt_topic': f'{vision_prefix}/photoncam_targets_exist', 'update_style': 'indicator'},


    # COMMANDS  - MOST LIKELY WILL CHANGE EVERY YEAR BUT GOOD TO GROUP IN ONE PLACE
    'qlabel_elevator_top_indicator': {'widget_name': 'qlabel_elevator_top_indicator', 'nt_topic': f'{command_prefix}/MoveElevatorTop/running', 'command_topic': f'{command_prefix}/MoveElevatorTop/running', 'update_style': 'indicator'},
    'qlabel_elevator_up_indicator': {'widget_name': 'qlabel_elevator_up_indicator', 'nt_topic': f'{command_prefix}/MoveElevatorUp/running', 'command_topic': f'{command_prefix}/MoveElevatorUp/running', 'update_style': 'indicator'},
    'qlabel_elevator_down_indicator': {'widget_name': 'qlabel_elevator_down_indicator', 'nt_topic': f'{command_prefix}/MoveElevatorDown/running', 'command_topic': f'{command_prefix}/MoveElevatorDown/running', 'update_style': 'indicator'},
    'qlabel_pivot_up_indicator': {'widget_name': 'qlabel_pivot_up_indicator', 'nt_topic': f'{command_prefix}/MovePivotUp/running', 'command_topic': f'{command_prefix}/MovePivotUp/running', 'update_style': 'indicator'},
    'qlabel_pivot_down_indicator': {'widget_name': 'qlabel_pivot_down_indicator', 'nt_topic': f'{command_prefix}/MovePivotDown/running', 'command_topic': f'{command_prefix}/MovePivotDown/running', 'update_style': 'indicator'},
    'qlabel_wrist_up_indicator': {'widget_name': 'qlabel_wrist_up_indicator', 'nt_topic': f'{command_prefix}/MoveWristUp/running', 'command_topic': f'{command_prefix}/MoveWristUp/running', 'update_style': 'indicator'},
    'qlabel_wrist_down_indicator': {'widget_name': 'qlabel_wrist_down_indicator', 'nt_topic': f'{command_prefix}/MoveWristDown/running', 'command_topic': f'{command_prefix}/MoveWristDown/running', 'update_style': 'indicator'},
    'qlabel_intake_on_indicator': {'widget_name': 'qlabel_intake_on_indicator', 'nt_topic': f'{command_prefix}/IntakeOn/running', 'command_topic': f'{command_prefix}/IntakeOn/running', 'update_style': 'indicator'},
    'qlabel_intake_off_indicator': {'widget_name': 'qlabel_intake_off_indicator', 'nt_topic': f'{command_prefix}/IntakeOff/running','command_topic': f'{command_prefix}/IntakeOff/running', 'update_style': 'indicator'},
    'qlabel_intake_reverse_indicator': {'widget_name': 'qlabel_intake_reverse_indicator', 'nt_topic': f'{command_prefix}/IntakeReverse/running','command_topic': f'{command_prefix}/IntakeReverse/running', 'update_style': 'indicator'},
    'qlabel_climber_down_indicator': {'widget_name': 'qlabel_climber_down_indicator', 'nt_topic': f'{command_prefix}/Move climber down/running', 'command_topic': f'{command_prefix}/Move climber down/running', 'update_style': 'indicator'},
    'qlabel_climber_up_indicator': {'widget_name': 'qlabel_climber_up_indicator', 'nt_topic': f'{command_prefix}/Move climber up/running', 'command_topic': f'{command_prefix}/Move climber up/running', 'update_style': 'indicator'},
    'qlabel_stow_indicator': {'widget_name': 'qlabel_stow_indicator', 'nt_topic': f'{command_prefix}/GoToStow/running', 'command_topic': f'{command_prefix}/GoToStow/running', 'update_style': 'indicator'},
    'qlabel_score_indicator': {'widget_name': 'qlabel_score_indicator', 'nt_topic': f'{command_prefix}/Score/running', 'command_topic': f'{command_prefix}/Score/running', 'flash':True, 'update_style': 'indicator'},
    'qlabel_l1_indicator': {'widget_name': 'qlabel_l1_indicator', 'nt_topic': f'{command_prefix}/GoToL1/running', 'command_topic': f'{command_prefix}/GoToL1/running', 'update_style': 'indicator'},
    'qlabel_l2_indicator': {'widget_name': 'qlabel_l2_indicator', 'nt_topic': f'{command_prefix}/GoToL2/running', 'command_topic': f'{command_prefix}/GoToL2/running', 'update_style': 'indicator'},
    'qlabel_l3_indicator': {'widget_name': 'qlabel_l3_indicator', 'nt_topic': f'{command_prefix}/GoToL3/running', 'command_topic': f'{command_prefix}/GoToL3/running', 'update_style': 'indicator'},
    'qlabel_l4_indicator': {'widget_name': 'qlabel_l4_indicator', 'nt_topic': f'{command_prefix}/GoToL4/running', 'command_topic': f'{command_prefix}/GoToL4/running', 'update_style': 'indicator'},
    'qlabel_can_report_indicator': {'widget_name': 'qlabel_can_report_indicator', 'nt_topic': f'{command_prefix}/CANStatus/running', 'command_topic': f'{command_prefix}/CANStatus/running', 'update_style': 'indicator'},
    'qlabel_reset_flex_indicator': {'widget_name': 'qlabel_reset_flex_indicator', 'nt_topic': f'{command_prefix}/ResetFlex/running', 'command_topic': f'{command_prefix}/ResetFlex/running', 'update_style': 'indicator'},
    'qlabel_elevator_shift_up_indicator': {'widget_name': 'qlabel_elevator_shift_up_indicator', 'nt_topic': f'{command_prefix}/CalElevatorUp/running', 'command_topic': f'{command_prefix}/CalElevatorUp/running', 'update_style': 'indicator'},
    'qlabel_elevator_shift_down_indicator': {'widget_name': 'qlabel_elevator_shift_down_indicator', 'nt_topic': f'{command_prefix}/CalElevatorDown/running', 'command_topic': f'{command_prefix}/CalElevatorDown/running', 'update_style': 'indicator'},
    'qlabel_cal_wrist_indicator': {'widget_name': 'qlabel_cal_wrist_indicator', 'nt_topic': f'{command_prefix}/RecalWrist/running', 'command_topic': f'{command_prefix}/RecalWrist/running', 'update_style': 'indicator'},
    'qlabel_cal_wrist_up_indicator': {'widget_name': 'qlabel_cal_wrist_up_indicator', 'nt_topic': f'{command_prefix}/CalWristUp/running', 'command_topic': f'{command_prefix}/CalWristUp/running', 'update_style': 'indicator'},
    'qlabel_cal_wrist_down_indicator': {'widget_name': 'qlabel_cal_wrist_down_indicator', 'nt_topic': f'{command_prefix}/CalWristDown/running', 'command_topic': f'{command_prefix}/CalWristDown/running', 'update_style': 'indicator'},

    'qlabel_game_piece_indicator': {'widget_name': 'qlabel_game_piece_indicator', 'nt_topic': f'{command_prefix}/gamepiece_present', 'command_topic': f'{command_prefix}/LedToggle/running', 'update_style': 'indicator',
                                        'style_on': "border: 7px; border-radius: 7px; background-color:rgb(0, 220, 220); color:rgb(250, 250, 250);",
                                        'style_off': "border: 7px; border-radius: 7px; background-color:rgb(127, 127, 127); color:rgb(0, 0, 0);"},
    'qlabel_navx_reset_indicator': {'widget_name': 'qlabel_navx_reset_indicator', 'nt_topic': f'{command_prefix}/GyroReset/running', 'command_topic': f'{command_prefix}/GyroReset/running', 'update_style': 'indicator'},


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
