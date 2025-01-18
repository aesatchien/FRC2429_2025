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

    k_elevator_max_height = 50 #inches
    k_elevator_min_height = 22 #inches

    k_shoulder_length = 12 #inches
    k_elbow_length = 22 #inches
    k_wrist_length = 14 #inches

    k_tolerance = 1.5

    k_positions = { #note: angles are relative to the parent ligament they're connected to. (test if negative angles are understood by sim)
        "stow": {
            "elevator_height": k_elevator_min_height,
            "shoulder_pivot": 180,
            "elbow_pivot": 180, 
            "wrist_pivot": 0,
            "wrist_color": wpilib.Color8Bit(0, 0, 255)
            #blue, ("#0000FF")
        },
        "ground": {
            "elevator_height": 22,
            "shoulder_pivot": 135,
            "elbow_pivot": 120,
            "wrist_pivot": 0,
            "wrist_color": wpilib.Color8Bit(0, 0, 255)
        },
        "l1": {
            "elevator_height": 22,
            "shoulder_pivot": 150, #angle between the vertical and the shoulder ligament
            "elbow_pivot": 145, #angle between shoulder ligament and elbow ligament
            "wrist_pivot": 90, #angle between the horizontal and the wrist ligament
            "wrist_color": wpilib.Color8Bit(0, 0, 255)
        },
        "l2": {
            "elevator_height": 22,
            "shoulder_pivot": 180,
            "elbow_pivot": 150,
            "wrist_pivot": 90,
            "wrist_color": wpilib.Color8Bit(255, 0, 0) #red "#FF0000"
        },
        "l3": {
            "elevator_height": 38,
            "shoulder_pivot": 180,
            "elbow_pivot": 150, 
            "wrist_pivot": 90,
            "wrist_color": wpilib.Color8Bit(255, 0, 0)
        },
        "l4": {
            "elevator_height": 38,
            "shoulder_pivot": 35, 
            "elbow_pivot": 270,
            "wrist_pivot": 90,
            "wrist_color": wpilib.Color8Bit(255, 0, 0)
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

    k_elevator_sim_max_height = 50
    k_elevator_sim_length = 10
    k_elevator_sim_width = 20

    k_shoulder_length_sim = 12
    k_elbow_length_sim = 22
    k_wrist_length_sim = 14


