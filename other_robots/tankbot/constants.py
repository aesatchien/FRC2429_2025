"""
This file contains constants for the tankbot robot.

It includes CAN IDs, conversion factors, and configuration objects for the Spark MAX motor controllers.
The constants are organized into classes for each subsystem.
"""
import math  # use this for pi and tau, cos and sin if necessary
from rev import SparkMaxConfig  # i think we can do SparkBaseConfig - it works for Max and Flex controllers
from typing import Union, List


 # ----------------  COMMON FUNCTIONS  ------------------------------------
def set_config_defaults(configs: Union[SparkMaxConfig, List[SparkMaxConfig]]) -> None:
    """
    Applies default configuration settings to a single config object or a list of config objects.
    Args:
        configs: A single configuration object or a list of configuration objects.
    """
    # Check if the input is a list (or any sequence except a string/bytes)
    if isinstance(configs, (list, tuple)):
        config_list = configs
    else:
        config_list = [configs]  # If it's a single item, wrap it in a list for the loop
    for config in config_list:
        config.voltageCompensation(12)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        config.smartCurrentLimit(40)

# general constants
k_burn_flash = True  # whether to burn the configurations into the spark maxes
k_driver_controller_port = 0  # USB index for the driver's joystick

class TestSubsystemConstants:
    # demonstrates the simplest class for holding a group of constants
    k_my_constant = 1  # sample constant

class DriveConstants:
    k_counter_offset = 1  # we don't want the subsystems printing messages on the same tic

    # example of tuple assignment for related values, especially constants or configuration parameters,
    # that are defined close together in your code
    k_CANID_r1, k_CANID_r2 = 3, 4  # right leader and follower
    k_CANID_l1, k_CANID_l2 = 1, 2  # left leader and follower

    # drivetrain gearing constants
    k_wheel_diameter_in = 6
    k_meter_per_inch = 0.0254
    k_gear_ratio = (52/12) * (68/30)
    k_position_conversion_factor = (k_wheel_diameter_in * math.pi * k_meter_per_inch / k_gear_ratio)
    k_velocity_conversion_factor = k_position_conversion_factor / 60  # convert distance/minute to distance/s

    # make configuration objects for the rev motor controllers
    k_left_config, k_right_config  = SparkMaxConfig(), SparkMaxConfig()
    k_configs = [k_left_config, k_right_config]  # this will be convenient later

    set_config_defaults(k_configs)  # set all the default configs - ["DRY" = (don't repeat yourself)]
    # traditional for loop approach - this is probably more readable than multiple list comprehensions
    for config in k_configs:
        config.encoder.positionConversionFactor(k_position_conversion_factor)
        config.encoder.velocityConversionFactor(k_velocity_conversion_factor)
    # these individual parameters need to be separate
    k_left_config.inverted(True)
    k_right_config.inverted(False)

    # set up the followers
    k_follower_config_r2,  k_follower_config_l2= SparkMaxConfig(), SparkMaxConfig()
    set_config_defaults([k_follower_config_r2, k_follower_config_l2])

    k_follower_config_r2.follow(k_CANID_r1, invert=False)
    k_follower_config_l2.follow(k_CANID_l1, invert=False)


class ShooterConstants:
    k_flywheel_counter_offset = 2
    k_CANID_indexer = 5
    k_CANID_flywheel_left_leader, k_CANID_flywheel_right_follower = 7, 8  # left flywheel and follower
    k_CANID_turret = 9

    # FLYWHEEL
    k_flywheel_left_leader_config, k_flywheel_right_follower_config = SparkMaxConfig(), SparkMaxConfig()
    k_flywheel_configs = [k_flywheel_left_leader_config, k_flywheel_right_follower_config]
    k_test_speed = 2000
    k_fastest_speed = 4000
    k_test_rpm = 20
    k_fastest_rpm = 60

    k_flywheel_left_leader_config.inverted(False)  # have to check which way it spins for positive RPM
    # k_flywheel_right_follower.inverted(False)  # this is not necessary - it will get ignored

    # set up the followers
    k_flywheel_right_follower_config.follow(k_CANID_flywheel_left_leader, invert=True)  # always true if follower on other side

    #setting brake, voltage compensation, and current limit for the flywheel motors
    set_config_defaults(k_flywheel_configs)

    # INDEXER
    k_indexer_config = SparkMaxConfig()
    set_config_defaults(k_indexer_config)

    k_delay_between_balls = 1/5 * 50 # we want 200 milliseconds which is 1/5 of a second, setting it to 1/5 of our loop rate every second
    k_indexer_balls_per_rotation = 4
    k_indexer_config.inverted(True)
    k_indexer_gear_ratio = 5
    k_indexer_position_conversion_factor = 1/k_indexer_gear_ratio
    k_indexer_config.encoder.positionConversionFactor(k_indexer_position_conversion_factor)

    # TURRET



