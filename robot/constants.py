import math
import wpilib

from rev import ClosedLoopSlot, SparkMaxConfig
from wpimath.units import inchesToMeters
from wpimath.system.plant import DCMotor
from wpilib.simulation import SingleJointedArmSim
from subsystems.swerve_constants import DriveConstants

k_start_x = 0
k_start_y = 0
k_driver_controller_port = 0
k_co_driver_controller_port = 1
k_robot_mass_kg = 56
k_robot_moi = 1/12 * k_robot_mass_kg * (DriveConstants.kWheelBase**2 + DriveConstants.kWheelBase**2) # (https://choreo.autos/usage/estimating-moi/) 
k_reset_sparks_to_default = False
k_swerve_debugging_messages = True
k_use_apriltag_odometry = True
k_swerve_only = False
k_swerve_rate_limited = True
k_field_oriented = True
k_led_count = 20  # todo: update this to actual number
k_led_pwm_port = 0  # todo: update this to actual number

class IntakeConstants:

    k_CAN_id = 12
    k_intake_config = SparkMaxConfig()
    k_intake_config.inverted(True)
    k_intake_config.closedLoop.pid(1, 0, 0)

    k_tof_algae_port = 2
    k_tof_coral_port = 2

    k_sim_length = 0.25


class WristConstants:

    k_CAN_id = 10
    k_gear_ratio = 5 * 5 * 3 * 4.44
    k_abs_encoder_offset = 0
    k_length_meters = 20 * 0.0254
    k_center_of_mass_to_axis_of_rotation_dist_meters = inchesToMeters(6)
    k_mass_kg = 8
    k_moi = SingleJointedArmSim.estimateMOI(k_length_meters, k_mass_kg) # TODO: get from CAD
    k_plant = DCMotor.NEO550(1)

    k_min_angle = math.radians(0)
    k_max_angle = math.radians(180)
    k_tolerance = math.radians(2.5)
    k_sim_starting_angle = 0 # sim mechanism2d takes degrees

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
    k_moi = SingleJointedArmSim.estimateMOI(k_length_meters, k_mass_kg) # TODO: get from CAD
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

    k_gear_ratio = 12 # 9, 12, or 15 gear ratio said victor 1/30/25
                      # we need it seperate for the sim
    k_effective_pulley_diameter = inchesToMeters(2)
    k_meters_per_revolution = k_effective_pulley_diameter / k_gear_ratio
    k_mass_kg = 15
    k_plant = DCMotor.NEO(2)

    k_min_height = 0
    k_max_height = 1
    k_tolerance = 2 / 100 # 2 cm

    k_sim_starting_height = 2

    k_config = SparkMaxConfig()

    k_config.encoder.positionConversionFactor(k_meters_per_revolution)
    k_config.encoder.velocityConversionFactor(k_meters_per_revolution / 60)

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

    k_timeofflight = 14 #elevator time of flight CAN ID
    
    k_positions = { #note: angles are relative to the parent ligament they're connected to. (test if negative angles are understood by sim)
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
            "shoulder_pivot": 270, #angle between the vertical and the shoulder ligament
            "wrist_pivot": 90, #angle between the horizontal and the wrist ligament
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

    #sim elevator
    k_window_height = 80
    k_window_width = 60
    k_window_length = 60

    k_elevator_sim_max_height = 60
    k_elevator_sim_length = 10
    k_elevator_sim_width = 20

    k_shoulder_length_sim = 23
    k_wrist_length_sim = 7

    k_coral_intake_coordinates = [(1.2, 2.2, 1), (1.2, 4, 1), (1.2, 6, 1), (1,7, 10), (1,1, 10)] #(x-coord, y-coord, number of corals at that location)
    k_coral_outtake_coordinates = [(5,5,0)]
    k_robot_radius_sim = 0.5
