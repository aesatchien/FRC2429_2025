import math

from rev import SparkBaseConfig
from subsystems.swerve_constants import DriveConstants

k_start_x = 0
k_start_y = 0
k_driver_controller_port = 0
k_robot_mass_kg = 56
k_robot_moi = 1/12 * k_robot_mass_kg * (DriveConstants.kWheelBase**2 + DriveConstants.kWheelBase**2) # (https://choreo.autos/usage/estimating-moi/) 
k_reset_sparks_to_default = False
k_swerve_debugging_messages = False
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
    

