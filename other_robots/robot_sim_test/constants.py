import math

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

class ElevatorConstants:
    k_CAN_id = 4

    kP = 6
    kI = 0
    kD = 0

    k_forward_limit = 100 #need to test for actual robot
    k_reverse_limit = 0
    k_forward_limit_enabled = False
    k_reverse_limit_enabled = False

    k_timeofflight = 14 #elevator time of flight CAN ID
    
    k_elevator_encoder_conversion_factor = 1 # [dist_unit] per revolution

    k_elevator_max_height = 60 #inches
    k_elevator_dist_between_pivot_and_top = 29 #inches
    k_elevator_min_height = 35 - k_elevator_dist_between_pivot_and_top #inches; treating the center of the shoulder pivot as the top of the elevator

    k_tolerance = 1.5

    k_positions = { #note: angles are relative to the parent ligament they're connected to. (test if negative angles are understood by sim)
        "stow": {
            "elevator_height": k_elevator_min_height,
            "shoulder_pivot": 0,
            "wrist_pivot": 0
        },
        "ground": {
            "elevator_height": k_elevator_min_height,
            "shoulder_pivot": 270,
            "wrist_pivot": 0
        },
        "l1": {
            "elevator_height": k_elevator_min_height + 10,
            "shoulder_pivot": 270, #angle between the vertical and the shoulder ligament
            "wrist_pivot": 90 #angle between the horizontal and the wrist ligament
        },
        "l2": {
            "elevator_height": k_elevator_min_height + 10,
            "shoulder_pivot": 315,
            "wrist_pivot": 90
        },
        "l3": {
            "elevator_height": k_elevator_min_height + 22,
            "shoulder_pivot": 315,
            "wrist_pivot": 90 
        },
        "l4": {
            "elevator_height": k_elevator_min_height + 32,
            "shoulder_pivot": 270, 
            "wrist_pivot": 90
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

    k_elevator_sim_max_height = 60
    k_elevator_sim_length = 10
    k_elevator_sim_width = 20

    k_shoulder_length_sim = 23
    k_wrist_length_sim = 14

    k_coral_intake_coordinates = [(1.2, 2.2, 1), (1.2, 4, 1), (1.2, 6, 1), (1,7, 10), (1,1, 10)] #(x-coord, y-coord, number of corals at that location)
    k_coral_outtake_coordinates = [(5,5,0)]
    k_robot_radius_sim = 0.5