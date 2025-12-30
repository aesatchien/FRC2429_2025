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
import re

# NT PREFIXES WE MAY NEED
camera_prefix = r'/Cameras'  # from the pis
quest_prefix = r'/QuestNav'  # putting this on par with the cameras as an external system
# systems inside/from the robot
status_prefix = r'/SmartDashboard/RobotStatus'  # the default for any status message
vision_prefix = r'/SmartDashboard/Vision'  # from the robot
swerve_prefix = r'/SmartDashboard/Swerve'  # from the robot
sim_prefix = r'/SmartDashboard/Sim'  # from the sim (still from the robot)
command_prefix = r'/SmartDashboard/Command'  # DIFFERENT FROM ROBOT CODE: the robot SmartDashboard.putData auto prepends /SmartDashboard to the key
base_prefix = '/SmartDashboard'  #  TODO - eventually nothing should be in here

# A list of command names used to generate widget configurations for simple, clickable indicators.
# Commands with special properties (e.g., custom topics, flash behavior) are defined separately.
COMMAND_LIST = [
    'MoveElevatorTop', 'MoveElevatorUp', 'MoveElevatorDown', 'MovePivotUp', 'MovePivotDown',
    'MoveWristUp', 'MoveWristDown', 'IntakeOn', 'IntakeOff', 'IntakeReverse',
    'Move climber down', 'Move climber up', 'GoToStow', 'GoToL1', 'GoToL2', 'GoToL3', 'GoToL4',
    'CanStatus', 'ResetFlex', 'CalElevatorUp', 'CalElevatorDown', 'RecalWrist', 'CalWristUp', 'CalWristDown', 'GyroReset'
]

# this config will be used to bind the NT topics to entries we can use later
# todo - somehow make the camera names all update from a config file, but that means ui and robot code need to know
CAMERA_BASE_CONFIG = {
    'genius_low': {'URL': 'http://10.24.29.12:1186/stream.mjpg',
                   'BASE_TOPIC': 'GeniusLow',
                  'NICKNAME': 'GENIUS LO',
                  'INDICATOR_INDEX': 0},
    'arducam_back': {'URL': 'http://10.24.29.12:1187/stream.mjpg',
                    'BASE_TOPIC': 'ArducamBack',
                    'NICKNAME': 'ARDU BACK',
                  'INDICATOR_INDEX': 1},
    'logitech_reef': {'URL': 'http://10.24.29.13:1186/stream.mjpg',
                   'BASE_TOPIC': 'LogitechReef',
                     'NICKNAME': 'LOGI REEF',
                      'INDICATOR_INDEX': 2},
    'logitech_reef_hsv': {'URL': 'http://10.24.29.13:1186/stream.mjpg',  # has no index, so no duplicate heartbeat
                   'BASE_TOPIC': 'LogitechReef',
                     'NICKNAME': 'LOGI HSV',
                      'TARGET_INDICATOR_NAME': 'qlabel_hsv_target_indicator'},  # has custom target indicator
    'arducam_high': {'URL': 'http://10.24.29.13:1187/stream.mjpg',
                   'BASE_TOPIC': 'ArducamHigh',
                    'NICKNAME': 'ARDU HI',
                    'INDICATOR_INDEX': 3},
    'Raw genius_low': {'URL': 'http://10.24.29.12:1181/stream.mjpg', 'skip':True},
    'Raw arducam_back': {'URL': 'http://10.24.29.12:1182/stream.mjpg', 'skip':True},
    'Raw logitech_reef': {'URL': 'http://10.24.29.13:1181/stream.mjpg', 'skip':True},
    'Raw arducam_high': {'URL': 'http://10.24.29.13:1182/stream.mjpg', 'skip':True},
    'Debug': {'URL': 'http://127.0.0.1:1186/stream.mjpg', 'skip':True},
}

# Add timestamp topics, connection topics, and set indicators if not provided above
# This is so i don't have to redo the camera indicators all the time below - just change the camera names and go home
CAMERA_CONFIG = {}
for key, value in CAMERA_BASE_CONFIG.items():
    # print(f"{key}  {value.get('TIMESTAMP_TOPIC')}  {value.get('INDICATOR_NAME')}  {value.get('INDICATOR_INDEX')}")
    d = {key:value.copy()}  # don't forget the copy
    base_topic = d[key].get('BASE_TOPIC')
    if not d[key].get('TIMESTAMP_TOPIC') and not 'skip' in d[key]:
        d[key]['TIMESTAMP_TOPIC'] = fr'{camera_prefix}/{base_topic}/_timestamp'
        # print(f' *  updating timestamp topic for {key}')
    if not d[key].get('CONNECTIONS_TOPIC') and not 'skip' in d[key]:
        d[key]['CONNECTIONS_TOPIC'] = fr'{camera_prefix}/{base_topic}/_connections'
        # print(f' *  updating connections topic for {key}')
    if not d[key].get('HEARTBEAT_INDICATOR_NAME') and 'INDICATOR_INDEX' in d[key]:
        index = d[key]['INDICATOR_INDEX']
        d[key]['HEARTBEAT_INDICATOR_NAME'] = f'qlabel_cam{index}_indicator'
    if not d[key].get('TARGET_INDICATOR_NAME') and 'INDICATOR_INDEX' in d[key]:
        index = d[key]['INDICATOR_INDEX']
        d[key]['TARGET_INDICATOR_NAME'] = f'qlabel_cam{index}_target_indicator'
        # print(f' ** updating indicator index topic for {key}')
    CAMERA_CONFIG.update(d)

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

    # COMMANDS - Special cases defined explicitly - the rest of the vanilla ones just respect the naming conventions in the UI designer and bob's your uncle
    'qlabel_score_indicator': {'widget_name': 'qlabel_score_indicator', 'nt_topic': f'{command_prefix}/Score/running', 'command_topic': f'{command_prefix}/Score/running', 'flash':True, 'update_style': 'indicator'},
    'qlabel_game_piece_indicator': {'widget_name': 'qlabel_game_piece_indicator', 'nt_topic': f'{command_prefix}/gamepiece_present', 'command_topic': f'{command_prefix}/LedToggle/running', 'update_style': 'indicator',
                                        'style_on': "border: 7px; border-radius: 7px; background-color:rgb(0, 220, 220); color:rgb(250, 250, 250);",
                                        'style_off': "border: 7px; border-radius: 7px; background-color:rgb(127, 127, 127); color:rgb(0, 0, 0);"},

    # orphaned test camera
    'qlabel_photoncam_target_indicator': {'widget_name': 'qlabel_photoncam_target_indicator', 'nt_topic': f'{vision_prefix}/photoncam_targets_exist', 'update_style': 'indicator'},

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

# ---------------    Auto-generate vanilla command indicator widgets from the COMMAND_LIST to reduce repetition
for command in COMMAND_LIST:
    # Sanitize command name for use in widget object names (replace spaces)
    # Turn capital letters after the first into lowercase with a leading underscore to make it pythonic
    widget_key_name = re.sub(r'(?<!^)(?=[A-Z])', '_', command).replace(' ', '_').lower()

    topic = f'{command_prefix}/{command}/running'
    WIDGET_CONFIG[f'qlabel_{widget_key_name}_indicator'] = {
        'widget_name': f'qlabel_{widget_key_name}_indicator',
        'nt_topic': topic,
        'command_topic': topic,
        'update_style': 'indicator'
    }

# ------------  THIS SHOULD BE GENERATED AUTOMATICALLY NOW FROM CAMERA_BASE and CAMERA_CONFIG
# CAMERA INDICATORS - HEARTBEAT AND TARGETS AVAILABLE - THESE HAVE NO NT TOPICS BECAUSE WE DO IT IN _update_camera_indicators
# TODO - give the heartbeats NT topics
# HEARTBEATS
for key, value in CAMERA_CONFIG.items():
    if 'HEARTBEAT_INDICATOR_NAME' in CAMERA_CONFIG[key]:
        # 'qlabel_arducam_high_indicator': {'widget_name': 'qlabel_cam2_indicator', 'update_style': 'camera_indicator'},
        d = {key + '_heartbeat_indicator': {'widget_name': value['HEARTBEAT_INDICATOR_NAME'], 'update_style': 'camera_indicator'}, }
        WIDGET_CONFIG.update(d)

# TARGETS AVAILABLE
for key, value in CAMERA_CONFIG.items():
    if 'TARGET_INDICATOR_NAME' in CAMERA_CONFIG[key]:
        # 'qlabel_arducam_high_target_indicator': {'widget_name': 'qlabel_cam2_target_indicator', 'nt_topic': f'{vision_prefix}/arducam_high_targets_exist', 'update_style': 'indicator'},
        d = {key + '_target_indicator': {'widget_name': value['TARGET_INDICATOR_NAME'], 'nt_topic': f'{vision_prefix}/{key}_targets_exist', 'update_style': 'indicator'}, }
        WIDGET_CONFIG.update(d)

if __name__ == "__main__":
    import pprint
    print("\n" + "="*80)
    print(f"CAMERA CONFIG ({len(CAMERA_CONFIG)} items)")
    print("="*80)
    pprint.pprint(CAMERA_CONFIG, width=120, sort_dicts=False)
    
    print("\n" + "="*80)
    print(f"WIDGET CONFIG ({len(WIDGET_CONFIG)} items)")
    print("="*80)
    pprint.pprint(WIDGET_CONFIG, width=120, sort_dicts=False)

    print("\n" + "="*80)
    print(f"WIDGET NAMES ({len(WIDGET_CONFIG)} items)")
    print("="*80)
    pprint.pprint(list(WIDGET_CONFIG.keys()), width=120, sort_dicts=False)