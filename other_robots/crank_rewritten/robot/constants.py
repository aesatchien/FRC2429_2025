import math

# this is the constants file.  All is in radians.
k_driver_controller_port = 0

# -------- LOWER CRANK ----------
k_lower_crank_CAN_id = 7
k_lower_crank_dict = {
        "kP": 1,
        "kI": 0,
        "kD": 0,
        "kIZone": 0,
        "k_gear_ratio": 5 * 5 * 3 * 4.44, # TODO: find
        "k_abs_encoder_offset": 0, # this is the value, in radians, given by the abs encoder when its offset is 0 and it is physically where you WANT it to be zero
        "k_forward_limit": math.radians(116),
        "k_reverse_limit": math.radians(40),
        "k_length_meters": 2/3, # for the sim
        "k_mass_kg": 5 # for the sim
}
