import math
from rev import ClosedLoopSlot, SparkClosedLoopController, SparkFlexConfig, SparkMax, SparkMaxConfig
from commands.drive_by_joystick import DriveByJoystick

k_burn_flash = True
k_driver_controller_port = 0

class TestSubsystemConstants:
    k_my_constant = 1

class DriveConstants:
    k_counter_offset = 4
    k_name = "drivetrain"

    k_CANID_r1 = 3
    k_CANID_r2 = 4
    k_CANID_l1 = 1
    k_CANID_l2 = 2

    k_gear_ratio = (52/12) * (68/30)

    k_left_config = SparkMaxConfig()
    k_right_config = SparkMaxConfig()
    k_configs = [k_left_config, k_right_config]

    #[config.encoder.positionConversionFactor(6 * math.pi * 0.0254 / k_gear_ratio) for config in k_configs]
    #[config.encoder.velocityConversionFactor(6 * math.pi * 0.0254 / (k_gear_ratio * 60)) for config in k_configs]
    [config.voltageCompensation(12) for config in k_configs]
    k_left_config.inverted(True)
    k_right_config.inverted(False)
    [config.setIdleMode(SparkMaxConfig.IdleMode.kBrake) for config in k_configs]
    [config.smartCurrentLimit(40) for config in k_configs]

    # k_config.absoluteEncoder.positionConversionFactor(math.tau / k_gear_ratio)
    # k_config.absoluteEncoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))
    #
    # k_config.absoluteEncoder.zeroOffset(0.45)
    k_abs_encoder_readout_when_at_ninety_deg_position = 0.455

    k_follower_config_r2 = SparkMaxConfig()
    k_follower_config_r2.follow(k_CANID_r1, invert=False)

    k_follower_config_l2 = SparkMaxConfig()
    k_follower_config_l2.follow(k_CANID_l1, invert=False)