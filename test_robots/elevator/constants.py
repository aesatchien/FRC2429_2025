from enum import Enum
import math
import rev
import wpilib

from rev import ClosedLoopSlot, SparkMaxConfig
from wpimath.units import inchesToMeters, lbsToKilograms
from wpimath.system.plant import DCMotor
from wpilib.simulation import SingleJointedArmSim
#from subsystems.swerve_constants import DriveConstants

k_start_x = 0
k_start_y = 0
k_driver_controller_port = 0
k_co_driver_controller_port = 1

k_led_count = 20  # todo: update this to actual number
k_led_pwm_port = 0  # todo: update this to actual number

k_positions = {
    "stow": {
        "elevator": inchesToMeters(8),
        "shoulder_pivot": math.radians(90),
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kBlue,
        "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
    },
    "ground": {
        "elevator": inchesToMeters(8),
        "shoulder_pivot": math.radians(-5),
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kBlue,
        "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)

    },
    "l1": {
        "elevator": inchesToMeters(8),
        "shoulder_pivot": math.radians(60),  # angle between the vertical and the shoulder ligament
        "wrist_pivot": math.radians(90),  # angle between the horizontal and the wrist ligament
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
    },
    "l2": {
        "elevator": inchesToMeters(14),
        "shoulder_pivot": math.radians(53),
        "wrist_pivot": math.radians(90),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "l3": {
        "elevator": inchesToMeters(40),
        "shoulder_pivot": math.radians(53),
        "wrist_pivot": math.radians(90),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "l4": {
        "elevator": inchesToMeters(57),
        "shoulder_pivot": math.radians(47),
        "wrist_pivot": math.radians(90),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "coral station": {
        "elevator": inchesToMeters(3),
        "shoulder_pivot": math.radians(55),
        "wrist_pivot": math.radians(90),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "processor": {  # TODO: find real values- this is a placeholder using stow's values
        "elevator": inchesToMeters(8),
        "shoulder_pivot": math.radians(90),
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kBlue,
        "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
    },
    "barge": {  # TODO: find real values- this is a placeholder using stow's values
        "elevator": inchesToMeters(8),
        "shoulder_pivot": math.radians(90),
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kBlue,
        "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
    },
    "algae low": {  # TODO: find real values- this is a placeholder using stow's values
        "elevator": inchesToMeters(8),
        "shoulder_pivot": math.radians(90),
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kBlue,
        "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
    },
    "algae high": {  # TODO: find real values- this is a placeholder using stow's values
        "elevator": inchesToMeters(8),
        "shoulder_pivot": math.radians(90),
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kBlue,
        "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
    },
}


class GamePiece(Enum):
    ALGAE = 1
    CORAL = 2


class IntakeConstants:
    k_CAN_id = 12
    k_intake_config = SparkMaxConfig()
    k_intake_config.inverted(True)
    k_intake_config.closedLoop.pid(1, 0, 0)

    k_tof_algae_port = 2
    k_tof_coral_port = 2

    k_sim_length = 0.25

    k_coral_intaking_voltage = 3
    k_algae_intaking_voltage = -2

    k_seconds_to_stay_on_while_scoring = 1


class ClimberConstants:
    k_CAN_id = 2
    k_climber_motor_rotation = math.radians((math.pi / 2))
    k_climber_motor_stowed_angle = 0
    k_climber_motor_climber_reference_angle = 10
    k_climber_forward_rotation_limit = 90
    k_climber_reverse_rotation_limit = 0
    # The PID constants will be changed later on.
    kP = 0
    kI = 1
    kD = 1
    k_climber_motor_voltage = 12


class WristConstants:
    k_CAN_id = 10
    k_gear_ratio = 5 * 5 * 3 * 4.44
    k_abs_encoder_offset = 0
    k_length_meters = 20 * 0.0254
    k_center_of_mass_to_axis_of_rotation_dist_meters = inchesToMeters(6)
    k_mass_kg = 8
    k_moi = SingleJointedArmSim.estimateMOI(k_length_meters, k_mass_kg)  # TODO: get from CAD
    k_plant = DCMotor.NEO550(1)

    k_min_angle = math.radians(-90)
    k_max_angle = math.radians(180)
    k_tolerance = math.radians(2.5)
    k_sim_starting_angle = 0  # sim mechanism2d takes degrees

    k_config = SparkMaxConfig()

    k_config.encoder.positionConversionFactor(math.tau / k_gear_ratio)
    k_config.encoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))

    k_config.absoluteEncoder.positionConversionFactor(math.tau / k_gear_ratio)
    k_config.absoluteEncoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))

    k_config.absoluteEncoder.zeroOffset(0.45)

    k_config.closedLoop.pid(p=6, i=0, d=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IZone(iZone=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IMaxAccum(0, slot=ClosedLoopSlot(0))

    k_config.softLimit.forwardSoftLimit(k_max_angle)
    k_config.softLimit.reverseSoftLimit(k_min_angle)

    k_config.softLimit.forwardSoftLimitEnabled(True)
    k_config.softLimit.reverseSoftLimitEnabled(True)

    k_config.smartCurrentLimit(40)


class ShoulderConstants:
    # angle is 0 when the shoulder is flat pointing forwards.
    # it increases as the shoulder goes clockwise from the perspective of the right side of the robot
    # i.e. when it starts flat and goes up, the angle increases

    k_CAN_id = 6
    k_follower_CAN_id = 7

    k_gear_ratio = 100
    k_abs_encoder_offset = 0
    k_length_meters = 0.5
    k_mass_kg = 10
    k_moi = SingleJointedArmSim.estimateMOI(k_length_meters, k_mass_kg)  # TODO: get from CAD
    k_plant = DCMotor.neoVortex(2)

    k_min_angle = math.radians(-45)
    k_max_angle = math.radians(225)
    k_tolerance = math.radians(2.5)
    k_sim_starting_angle = 0

    k_config = SparkMaxConfig()

    k_config.encoder.positionConversionFactor(math.tau / k_gear_ratio)
    k_config.encoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))

    k_config.absoluteEncoder.positionConversionFactor(math.tau / k_gear_ratio)
    k_config.absoluteEncoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))

    k_config.absoluteEncoder.zeroOffset(0.45)

    k_config.closedLoop.pid(p=6, i=0, d=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IZone(iZone=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IMaxAccum(0, slot=ClosedLoopSlot(0))
    # k_config.closedLoop.maxMotion.maxAcceleration(1)
    # k_config.closedLoop.maxMotion.maxVelocity(1000)

    k_config.softLimit.forwardSoftLimit(k_max_angle)
    k_config.softLimit.reverseSoftLimit(k_min_angle)

    k_config.softLimit.forwardSoftLimitEnabled(True)
    k_config.softLimit.reverseSoftLimitEnabled(True)

    k_follower_config = SparkMaxConfig()
    k_follower_config.follow(k_CAN_id)


class ElevatorConstants:
    # all in meters
    # although the tof uses mm, wpilib uses m, and we're using radians according to the wpilib standard
    # therefore, according to the wpilib standard, we will use m
    # 24 tooth sprocket, no. 25 chain

    k_CAN_id = 4
    k_follower_CAN_id = 5

    k_gear_ratio = 15  # 9, 12, or 15 gear ratio said victor 1/30/25
    # we need it seperate for the sim
    k_effective_pulley_diameter = inchesToMeters(
        1.91)  # (https://www.andymark.com/products/25-24-tooth-0-375-in-hex-sprocket) although we're using rev, rev doesn't give a pitch diameter
    k_meters_per_revolution = math.pi * 2 * k_effective_pulley_diameter / k_gear_ratio  # 2 because our elevator goes twice as fast as the chain because continuous rigging
    k_mass_kg = lbsToKilograms(25)
    k_plant = DCMotor.NEO(2)

    k_min_height = inchesToMeters(8)
    k_max_height = inchesToMeters(64)
    k_tolerance = 2 / 100  # 2 cm

    k_sim_starting_height = 2

    k_config = SparkMaxConfig()

    k_config.encoder.positionConversionFactor(k_meters_per_revolution)
    k_config.encoder.velocityConversionFactor(k_meters_per_revolution / 60)

    # k_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.)
    k_config.closedLoop.pid(p=6, i=0, d=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IZone(iZone=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IMaxAccum(0, slot=ClosedLoopSlot(0))

    k_config.softLimit.forwardSoftLimit(k_max_height)
    k_config.softLimit.reverseSoftLimit(k_min_height)

    k_config.softLimit.forwardSoftLimitEnabled(True)
    k_config.softLimit.reverseSoftLimitEnabled(True)

    k_config.smartCurrentLimit(60)

    k_follower_config = SparkMaxConfig()
    k_follower_config.follow(k_CAN_id)

    k_timeofflight = 14  # elevator time of flight CAN ID

    k_sim_positions_degrees = {
        # note: angles are relative to the parent ligament they're connected to. (test if negative angles are understood by sim)
        "stow": {
            "elevator_height": k_min_height,
            "shoulder_pivot": 0,
            "wrist_pivot": 0,
            "wrist_color_for_ligament": wpilib.Color.kBlue,
            "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
        },
        "ground": {
            "elevator_height": k_min_height,
            "shoulder_pivot": 270,
            "wrist_pivot": 0,
            "wrist_color_for_ligament": wpilib.Color.kBlue,
            "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)

        },
        "l1": {
            "elevator_height": k_min_height + 10,
            "shoulder_pivot": 270,  # angle between the vertical and the shoulder ligament
            "wrist_pivot": 90,  # angle between the horizontal and the wrist ligament
            "wrist_color_for_ligament": wpilib.Color.kRed,
            "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
        },
        "l2": {
            "elevator_height": k_min_height + 10,
            "shoulder_pivot": 315,
            "wrist_pivot": 90,
            "wrist_color_for_ligament": wpilib.Color.kRed,
            "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
        },
        "l3": {
            "elevator_height": k_min_height + 22,
            "shoulder_pivot": 315,
            "wrist_pivot": 90,
            "wrist_color_for_ligament": wpilib.Color.kRed,
            "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
        },
        "l4": {
            "elevator_height": k_min_height + 32,
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

    # sim elevator
    k_window_height = 80
    k_window_width = 60
    k_window_length = 60

    k_elevator_sim_max_height = 60
    k_elevator_sim_length = 10
    k_elevator_sim_width = 20

    k_shoulder_length_sim = 23
    k_wrist_length_sim = 7

    k_coral_intake_coordinates = [(1.2, 2.2, 1), (1.2, 4, 1), (1.2, 6, 1), (1, 7, 10),
                                  (1, 1, 10)]  # (x-coord, y-coord, number of corals at that location)
    k_coral_outtake_coordinates = [(5, 5, 0)]
    k_robot_radius_sim = 0.5


class VisionConstants:
    k_pi_names = ["top_pi"]
