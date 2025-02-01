import math
import wpilib

from rev import SparkBaseConfig
from subsystems.swerve_constants import DriveConstants

k_start_x = 0
k_start_y = 0
k_driver_controller_port = 0
k_robot_mass_kg = 56
k_robot_moi = 1/12 * k_robot_mass_kg * (DriveConstants.kWheelBase**2 + DriveConstants.kWheelBase**2) # (https://choreo.autos/usage/estimating-moi/) 
k_reset_sparks_to_default = False
k_swerve_debugging_messages = True
k_use_apriltag_odometry = True
k_swerve_only = True
k_swerve_rate_limited = True
k_field_oriented = True
k_led_count = 20  # todo: update this to actual number
k_led_pwm_port = 0  # todo: update this to actual number

class IntakeConstants:

    k_CAN_id = 12
    k_intake_config = SparkBaseConfig()
    k_intake_config.inverted(True)
    k_intake_config.closedLoop.pid(1, 0, 0)

    k_tof_algae_port = 2
    k_tof_coral_port = 2


class LowerCrankConstants:

    k_CAN_id = 7
    kP = 6
    kI = 0
    kD = 0
    kIZone = 0
    kIMaxAccum = 0
    k_gear_ratio = 5 * 5 * 3 * 4.44
    k_abs_encoder_offset = 0
    k_forward_limit = math.radians(116)
    k_reverse_limit = math.radians(40)
    k_length_meters = 20 * 0.0254
    k_mass_kg = 8
    k_lower_crank_sim_starting_angle = math.radians(60)

class ScoringSystemConstants:
    k_CAN_elevator_id = 4
    k_CAN_shoulder_id = 6

    kP = 6
    kI = 0
    kD = 0

    k_forward_limit = 100 #MUST_TEST_FOR_ACTUAL_ROBOT
    k_reverse_limit = 0
    k_forward_limit_enabled = False
    k_reverse_limit_enabled = False

    k_timeofflight = 14 #elevator time of flight CAN ID
    
    k_elevator_max_height = 2.2 #MUST_TEST_FOR_ACTUAL_ROBOT
    k_elevator_min_height = 0 #MUST_TEST_FOR_ACTUAL_ROBOT
    k_elevator_encoder_conversion_factor = 1 # [dist_unit] per revolution; MUST_TEST_FOR_ACTUAL_ROBOT

    k_pivot_height = 0.15 #distance between bottom and pivot center point. Note: the sim will reflect this value, not the total elevator height (because the delta between pivot piont & bottom + between pivot & top are both variable)
    k_tolerance = 0.01 # MUST_TEST_FOR_ACTUAL_ROBOT
    k_tolerance_degrees = 1.5

    k_positions = { #note: angles are relative to the parent ligament they're connected to. (test if negative angles are understood by sim)
        "stow": {
            "elevator_height": k_pivot_height,
            "shoulder_pivot": 0,
            "wrist_pivot": 0,
            "wrist_color_for_ligament": wpilib.Color.kBlue,
            "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
        },
        "ground": {
            "elevator_height": k_pivot_height,
            "shoulder_pivot": 270,
            "wrist_pivot": 0,
            "wrist_color_for_ligament": wpilib.Color.kBlue,
            "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
        },
        "l1": {
            "elevator_height": 0.5282,
            "shoulder_pivot": 270, #angle between the vertical and the shoulder ligament
            "wrist_pivot": 90, #angle between the horizontal and the wrist ligament
            "wrist_color_for_ligament": wpilib.Color.kRed,
            "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
        },
        "l2": {
            "elevator_height": 0.5282,
            "shoulder_pivot": 300,
            "wrist_pivot": 90,
            "wrist_color_for_ligament": wpilib.Color.kRed,
            "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
        },
        "l3": {
            "elevator_height": 1.00,
            "shoulder_pivot": 300,
            "wrist_pivot": 90,
            "wrist_color_for_ligament": wpilib.Color.kRed,
            "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
        },
        "l4": {
            "elevator_height": 2.0,
            "shoulder_pivot": 270, 
            "wrist_pivot": 90,
            "wrist_color_for_ligament": wpilib.Color.kRed,
            "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
        },
        "processor": 0,
        "barge": 0,
        "coral station": 0,
        "algae 1": 0,
        "algae 2": 0
    }

    #sim elevator
    k_window_height = 80
    k_window_width = 60
    k_window_length = 60

    k_elevator_sim_max_height = 72
    k_elevator_sim_length = 10
    k_elevator_sim_width = 20

    k_shoulder_length_sim = 23
    k_wrist_length_sim = 7

    k_coral_intake_coordinates = [(1.2, 2.2, 1), (1.2, 4, 1), (1.2, 6, 1), (1,7, 10), (1,1, 10)] #(x-coord, y-coord, number of corals at that location)
    k_coral_outtake_coordinates = [(5,5,0)]
    k_robot_radius_sim = 0.5