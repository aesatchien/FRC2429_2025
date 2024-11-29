import math

class GeneralConstants:
    k_driver_controller_port = 0

class LowerCrankConstants:

    k_CAN_id = 7
    kP = 1
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
    

