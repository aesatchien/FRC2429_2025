"""
This file contains constants for the tankbot robot.

It includes CAN IDs, conversion factors, and configuration objects for the Spark MAX motor controllers.
The constants are organized into classes for each subsystem.
"""
import math  # use this for pi and tau, cos and sin if necessary
from rev import SparkMaxConfig  # i think we can do SparkBaseConfig - it works for Max and Flex controllers

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

    # set the config parameters - doing them individually is dirty, but here is an example
    _ = [config.voltageCompensation(12) for config in k_configs]
    [config.setIdleMode(SparkMaxConfig.IdleMode.kBrake) for config in k_configs]
    [config.smartCurrentLimit(40) for config in k_configs]
    #[config.encoder.positionConversionFactor(k_position_conversion_factor for config in k_configs]
    #[config.encoder.velocityConversionFactor(k_velocity_conversion_factor) for config in k_configs]

    # traditional for loop approach - this is probably more readable than multiple list comprehensions
    for config in k_configs:
        config.voltageCompensation(12)
        #sets the drivetrain to brake mode - MH
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
        config.smartCurrentLimit(40)
        config.encoder.positionConversionFactor(k_position_conversion_factor)
        config.encoder.velocityConversionFactor(k_velocity_conversion_factor)
    # these individual parameters need to be separate
    k_left_config.inverted(True)
    k_right_config.inverted(False)

    # set up the followers
    k_follower_config_r2 = SparkMaxConfig()
    k_follower_config_r2.follow(k_CANID_r1, invert=False)

    k_follower_config_l2 = SparkMaxConfig()
    k_follower_config_l2.follow(k_CANID_l1, invert=False)

    # not sure what these were for
    # k_config.absoluteEncoder.positionConversionFactor(math.tau / k_gear_ratio)
    # k_config.absoluteEncoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))
    # k_config.absoluteEncoder.zeroOffset(0.45)
    # k_abs_encoder_readout_when_at_ninety_deg_position = 0.455

class ShooterConstants:
    k_flywheel_counter_offset = 2
    k_CANID_indexer = 5
    k_CANID_flywheel_left_leader, k_CANID_flywheel_right_follower = 7, 8  # left flywheel and follower
    k_CANID_turret = 9

    # FLYWHEEL
    k_flywheel_left_leader_config, k_flywheel_right_follower_config = SparkMaxConfig(), SparkMaxConfig()
    k_flywheel_configs = [k_flywheel_left_leader_config, k_flywheel_right_follower_config]

    k_flywheel_left_leader_config.inverted(False)  # have to check which way it spins for positive RPM
    # k_flywheel_right_follower.inverted(False)  # this is not necessary - it will get ignored

    # set up the followers
    k_flywheel_right_follower_config.follow(k_CANID_flywheel_left_leader, invert=True)  # always true if follower on other side

    #setting voltage & current limit for the flywheel motors
    _ = [config.voltageCompensation(12) for config in k_flywheel_configs]
    _ = [config.setIdleMode(SparkMaxConfig.IdleMode.kBrake) for config in k_flywheel_configs]
    _ = [config.smartCurrentLimit(40) for config in k_flywheel_configs]

    # INDEXER

    # TURRET
