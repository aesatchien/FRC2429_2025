from enum import Enum
import math
import rev
import wpilib

from rev import ClosedLoopSlot, SparkClosedLoopController, SparkFlexConfig, SparkMax, SparkMaxConfig
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import inchesToMeters, lbsToKilograms
from wpimath.system.plant import DCMotor
from wpilib.simulation import SingleJointedArmSim
from subsystems.swerve_constants import DriveConstants

# starting position for odometry
k_start_x = 0
k_start_y = 0
# joysticks nad other input
k_driver_controller_port = 0
k_co_driver_controller_port = 1
k_use_bbox = True  # set to true for actual use
k_bbox_1_port = 2
k_bbox_2_port = 3

# robot characteristics - should be near driveconstants, not here
k_robot_mass_kg = 56
k_robot_moi = 1/12 * k_robot_mass_kg * (DriveConstants.kWheelBase**2 + DriveConstants.kWheelBase**2) # (https://choreo.autos/usage/estimating-moi/) 

k_reset_sparks_to_default = True
k_swerve_debugging_messages = True
# multiple attempts at tags this year - TODO - use l/r/ or up/down tilted cameras again, gives better data
k_use_apriltag_odometry = False
k_use_photontags = True
k_use_CJH_tags = False
k_swerve_only = False
k_swerve_rate_limited = True
k_field_oriented = True


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
        "elevator": 0.45, # this is the l2 angles. TODO: find actual angles
        "shoulder_pivot": math.radians(130),
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "l2": {
        "elevator": 0.45,
        "shoulder_pivot": math.radians(130),
        "wrist_pivot": math.radians(90),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "l3": {
        "elevator": 0.9,
        "shoulder_pivot": math.radians(132),
        "wrist_pivot": math.radians(90),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "l4": {
        "elevator": 1.45,
        "shoulder_pivot": math.radians(120),
        "wrist_pivot": math.radians(90),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "l2_wrist_clearing": { # intermediate position in which we spin the wrist before scoring
        "elevator": inchesToMeters(23),
        "shoulder_pivot": math.radians(130), 
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "l3_wrist_clearing": { # intermediate position in which we spin the wrist before scoring
        "elevator": 1.1,
        "shoulder_pivot": math.radians(90), 
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "l4_wrist_clearing": { # intermediate position in which we spin the wrist before scoring
        "elevator": 1.2,
        "shoulder_pivot": math.radians(90), 
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "coral station": {
        "elevator": 0.42, # HACK :was .40 at clark
        "shoulder_pivot": math.radians(57), # HACK: was 61 at clark
        "wrist_pivot": math.radians(0), # hopefully the negative makes it turn the safer way
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "processor": { # TODO: find real values- this is a placeholder using stow's values
        "elevator": inchesToMeters(8),
        "shoulder_pivot": math.radians(90),
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kBlue,
        "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
    },
    "barge": { # TODO: find real values- this is a placeholder using stow's values
        "elevator": inchesToMeters(8),
        "shoulder_pivot": math.radians(90),
        "wrist_pivot": math.radians(0),
        "wrist_color_for_ligament": wpilib.Color.kBlue,
        "wrist_color_for_setColor": wpilib.Color8Bit(0, 0, 255)
    },
    "algae low": { # TODO: find real values- this is a placeholder using stow's values
        "elevator": 0.45,
        "shoulder_pivot": math.radians(130),
        "wrist_pivot": math.radians(90),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
    "algae high": { # TODO: find real values- this is a placeholder using stow's values
        "elevator": 0.9,
        "shoulder_pivot": math.radians(132),
        "wrist_pivot": math.radians(90),
        "wrist_color_for_ligament": wpilib.Color.kRed,
        "wrist_color_for_setColor": wpilib.Color8Bit(255, 0, 0)
    },
}

print("\nWARNING! NOT USING COMP SETPOINTS!!" * 40)

k_blue_reef_center = Translation2d(inchesToMeters(176.75), inchesToMeters(158.3))
k_field_center = Translation2d(inchesToMeters(690.876 / 2), inchesToMeters(158.3))
k_red_reef_center = k_blue_reef_center.rotateAround(k_field_center, Rotation2d(math.radians(180)))
k_reef_center_to_face_translation = Translation2d(inchesToMeters(65.5 / 2), 0)

# made from this:
# k_blue_reef_center = Translation2d(inchesToMeters(176.75), inchesToMeters(158.3))
# k_field_center = Translation2d(inchesToMeters(690.876 / 2), inchesToMeters(158.3))
# k_red_reef_center = k_blue_reef_center.rotateAround(k_field_center, Rotation2d(math.radians(180)))
# k_reef_center_to_face_translation = Translation2d(-inchesToMeters(65.5 / 2 + 14), 0)
#
# k_useful_robot_poses = {}
# for idx, letter in enumerate(["a", "b", "c", "d", "e", "f", "g", "h", "i", "j", "k", "l"]):
#
#     center_to_this_face = k_reef_center_to_face_translation.rotateBy(Rotation2d(math.radians((idx // 2) * 60)))
#     pose = Pose2d(center_to_this_face + k_blue_reef_center, Rotation2d(math.radians((idx // 2) * 60) ))
#     k_useful_robot_poses.update({letter : pose})

# we can define in the above format or the below format depending on how we want to tune
 
k_useful_robot_poses_blue = {
    'a': Pose2d(Translation2d(x=3.302000, y=4.020820), Rotation2d(0.000000)),
    'b': Pose2d(Translation2d(x=3.302000, y=4.020820), Rotation2d(0.000000)),
    'c': Pose2d(Translation2d(x=3.895725, y=2.992458), Rotation2d(1.047198)),
    'd': Pose2d(Translation2d(x=3.895725, y=2.992458), Rotation2d(1.047198)),
    'e': Pose2d(Translation2d(x=5.083175, y=2.992458), Rotation2d(2.094395)),
    'f': Pose2d(Translation2d(x=5.083175, y=2.992458), Rotation2d(2.094395)),
    'g': Pose2d(Translation2d(x=5.676900, y=4.020820), Rotation2d(3.141593)),
    'h': Pose2d(Translation2d(x=5.676900, y=4.020820), Rotation2d(3.141593)),
    'i': Pose2d(Translation2d(x=5.083175, y=5.049182), Rotation2d(4.188790)),
    'j': Pose2d(Translation2d(x=5.083175, y=5.049182), Rotation2d(4.188790)),
    'k': Pose2d(Translation2d(x=3.895725, y=5.049182), Rotation2d(5.235988)),
    'l': Pose2d(Translation2d(x=3.895725, y=5.049182), Rotation2d(5.235988)),
    'left_coral_station': Pose2d(Translation2d(x=1.060193, y=7.108810), Rotation2d(-0.942478)),
    'right_coral_station': Pose2d(Translation2d(x=1.060193, y=0.932830), Rotation2d(0.942478)), # from Pose2d(Translation2d(x=1.060193, y=-7.108810 + 2 * k_field_center.y), Rotation2d(-0.942478)),
}

k_useful_robot_poses_red = {key: pose.rotateAround(k_field_center, Rotation2d(math.radians(180))) for key, pose in k_useful_robot_poses_blue.items()}


class GamePiece(Enum):
    ALGAE = 1
    CORAL = 2


class IntakeConstants:

    k_counter_offset = 1
    k_nt_debugging = False  # print extra values to NT for debugging
    k_CAN_id = 12
    k_intake_config = SparkMaxConfig()
    k_intake_config.inverted(False) # this is how our code works LHACK 3/3/25
    k_intake_config.closedLoop.pid(1, 0, 0)
    k_intake_config.smartCurrentLimit(5)
    k_intake_config.voltageCompensation(12)

    k_tof_coral_port = 13
    k_max_tof_distance_where_we_have_coral = 70  # millimeters  engages at 60 and bottoms out at 26

    k_sim_length = 0.25

    k_coral_intaking_voltage = -6
    k_algae_intaking_voltage = -6

    k_coral_scoring_voltage = 10

    k_seconds_to_stay_on_while_scoring = 0.5


class ClimberConstants:

    k_counter_offset = 2
    k_nt_debugging = False  # print extra values to NT for debugging
    k_CAN_id = 2
    k_follower_CAN_id = 3
    k_gear_ratio = 167
    # 25:1
    # 26:64
    # 30:80
    k_climber_motor_ready = math.radians(30)
    k_climber_motor_stowed_angle = 0
    k_climber_motor_climb_angle = math.radians(60)
    k_climber_forward_rotation_limit = math.radians(90)
    k_climber_reverse_rotation_limit = 0
    k_length_meters = 0.1209
    k_moi = 0.0173 #kg m^2
    k_plant = DCMotor.NEO(2)
    # The PID constants will be changed later on.
    kP = 0.75
    kI = 0
    kD = 0
    k_climber_motor_voltage = 12
    k_tolerance = math.radians(5)

    k_config = SparkMaxConfig()
    k_config.inverted(True) # makes more sense LHACK 3/3/25
    k_config.setIdleMode(SparkFlexConfig.IdleMode.kBrake)
    # k_config.smartCurrentLimit(40)
    k_config.voltageCompensation(12)

    k_config.encoder.positionConversionFactor(math.tau / k_gear_ratio)
    k_config.encoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))

    k_config.closedLoop.pid(p=kP, i=kI, d=kD, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IZone(iZone=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IMaxAccum(0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.outputRange(-1, 1)
    # k_config.closedLoop.maxMotion.maxAcceleration(1)
    # k_config.closedLoop.maxMotion.maxVelocity(1000)
        
    k_config.softLimit.forwardSoftLimit(k_climber_forward_rotation_limit)
    k_config.softLimit.reverseSoftLimit(k_climber_reverse_rotation_limit)

    k_config.softLimit.forwardSoftLimitEnabled(False)
    k_config.softLimit.reverseSoftLimitEnabled(False)

    k_follower_config = SparkMaxConfig()
    k_follower_config.follow(k_CAN_id, True)


class WristConstants:

    k_counter_offset = 3
    k_nt_debugging = False  # print extra values to NT for debugging
    k_CAN_id = 10
    k_gear_ratio = 16 * 64 / 26
    k_abs_encoder_offset = 0
    k_sim_length_meters = 0.4
    k_center_of_mass_to_axis_of_rotation_dist_meters = 0.164
    k_mass_kg = lbsToKilograms(5.57)
    k_moi = 0.0474
    k_plant = DCMotor.NEO550(1)

    k_min_angle = math.radians(-180)
    k_max_angle = math.radians(95)
    k_tolerance = math.radians(2.5)
    k_starting_angle = 0 # sim mechanism2d takes degrees

    k_config = SparkMaxConfig()
    k_config.voltageCompensation(12)
    k_config.inverted(True)
    k_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

    k_config.encoder.positionConversionFactor(math.tau / k_gear_ratio)
    k_config.encoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))

    k_config.absoluteEncoder.positionConversionFactor(math.tau / k_gear_ratio)
    k_config.absoluteEncoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))
    
    k_config.absoluteEncoder.zeroOffset(0.45)

    k_config.closedLoop.pid(p=0.8, i=0, d=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.pid(p=0.4, i=0, d=0, slot=ClosedLoopSlot(1))
    k_config.closedLoop.IZone(iZone=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IMaxAccum(0, slot=ClosedLoopSlot(0))
        
    k_config.softLimit.forwardSoftLimit(k_max_angle)
    k_config.softLimit.reverseSoftLimit(k_min_angle)

    k_config.softLimit.forwardSoftLimitEnabled(True)
    k_config.softLimit.reverseSoftLimitEnabled(True)

    k_config.smartCurrentLimit(40)

    k_min_arm_angle_where_spinning_dangerous = math.radians(70)
    k_max_arm_angle_where_spinning_dangerous = math.radians(110)

    k_max_elevator_height_where_spinning_dangerous = inchesToMeters(35)

    k_stowed_min_angle = math.radians(-15)
    k_stowed_max_angle = math.radians(15)


class ShoulderConstants:

    # angle is 0 when the shoulder is flat pointing forwards. 
    # it increases as the shoulder goes clockwise from the perspective of the right side of the robot
    # i.e. when it starts flat and goes up, the angle increases

    k_counter_offset = 4
    k_nt_debugging = False  # print extra values to NT for debugging
    k_name = "profiled_pivot"

    k_CAN_id = 6
    k_follower_CAN_id = 7

    k_gear_ratio = 100
    k_abs_encoder_offset = 0
    k_length_meters = 0.6785
    k_mass_kg = lbsToKilograms(17.6)
    k_moi = SingleJointedArmSim.estimateMOI(k_length_meters, k_mass_kg) # TODO: get from CAD
    k_moi = 0.5914
    k_plant = DCMotor.neoVortex(2)

    k_max_velocity_rad_per_second = 1.5
    k_max_acceleration_rad_per_sec_squared = 2.5
    k_kS_volts = 0 # constant to always add, uses the sign of velocity
    k_kG_volts = 1.4/2.0  # 12kg at .2m COM, cuts in half with two motors, goes up with mass and distance, down with efficiency
    k_kV_volt_second_per_radian = 1.69  # stays the same with one or two motors, based on the NEO itself and gear ratio
    k_kA_volt_second_squared_per_meter = 0.04 / 2.0 # cuts in half with 2 motors

    k_min_angle = math.radians(-45)
    k_max_angle = math.radians(225)
    k_tolerance = math.radians(2.5)
    k_starting_angle = math.radians(90) # until we have an abs encoder this is where we expect it to start

    k_config = SparkFlexConfig()
    k_config.voltageCompensation(12)
    k_config.inverted(False)
    k_config.setIdleMode(SparkFlexConfig.IdleMode.kBrake)
    k_config.smartCurrentLimit(40)

    k_config.encoder.positionConversionFactor(math.tau / k_gear_ratio)
    k_config.encoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))

    k_config.absoluteEncoder.positionConversionFactor(math.tau / k_gear_ratio)
    k_config.absoluteEncoder.velocityConversionFactor(math.tau / (k_gear_ratio * 60))
    
    k_config.absoluteEncoder.zeroOffset(0.45)

    k_config.closedLoop.pid(p=0.85, i=0, d=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IZone(iZone=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IMaxAccum(0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.outputRange(-1, 1)
    # k_config.closedLoop.maxMotion.maxAcceleration(1)
    # k_config.closedLoop.maxMotion.maxVelocity(1000)
        
    k_config.softLimit.forwardSoftLimit(k_max_angle)
    k_config.softLimit.reverseSoftLimit(k_min_angle)

    k_config.softLimit.forwardSoftLimitEnabled(True)
    k_config.softLimit.reverseSoftLimitEnabled(True)

    k_follower_config = SparkMaxConfig()
    k_follower_config.follow(k_CAN_id, invert=False)

    k_min_angle_to_elevate_in_front_of_reef = 80



class ElevatorConstants:
    # all in meters
    # although the tof uses mm, wpilib uses m, and we're using radians according to the wpilib standard
    # therefore, according to the wpilib standard, we will use m
    # 24 tooth sprocket, no. 25 chain

    k_counter_offset = 5
    k_nt_debugging = False  # print extra values to NT for debugging
    k_name = "elevator"

    k_CAN_id = 4
    k_follower_CAN_id = 5

    k_max_velocity_meter_per_second = 2
    k_max_acceleration_meter_per_sec_squared = 5
    k_kS_volts = 0 # constant to always add, uses the sign of velocity
    k_kG_volts = 0.88 / 2.0  # 12kg at .2m COM, cuts in half with two motors, goes up with mass and distance, down with efficiency
    k_kV_volt_second_per_radian = 12.05  # stays the same with one or two motors, based on the NEO itself and gear ratio
    k_kA_volt_second_squared_per_meter = 0.10 / 2.0 # cuts in half with 2 motors

    k_gear_ratio = 15 # 9, 12, or 15 gear ratio said victor 1/30/25
                      # we need it seperate for the sim
    k_effective_pulley_diameter = inchesToMeters(1.91) # (https://www.andymark.com/products/25-24-tooth-0-375-in-hex-sprocket) although we're using rev, rev doesn't give a pitch diameter
    k_meters_per_revolution = math.pi * 2 * k_effective_pulley_diameter / k_gear_ratio # 2 because our elevator goes twice as fast as the chain because continuous rigging
    k_mass_kg = lbsToKilograms(19)
    k_plant = DCMotor.NEO(2)

    k_min_height = inchesToMeters(8)
    k_max_height = inchesToMeters(64)
    k_tolerance = 2 / 100 # 2 cm

    k_sim_starting_height = 2

    k_config = SparkMaxConfig()
    k_config.voltageCompensation(12)
    k_config.inverted(True)

    k_config.encoder.positionConversionFactor(k_meters_per_revolution)
    k_config.encoder.velocityConversionFactor(k_meters_per_revolution / 60)

    # k_config.closedLoop.setFeedbackSensor(rev.ClosedLoopConfig.)
    k_config.closedLoop.pid(p=1.4, i=0, d=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IZone(iZone=0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.IMaxAccum(0, slot=ClosedLoopSlot(0))
    k_config.closedLoop.outputRange(-1, 1)
        
    k_config.softLimit.forwardSoftLimit(k_max_height)
    k_config.softLimit.reverseSoftLimit(k_min_height)

    k_config.softLimit.forwardSoftLimitEnabled(True)
    k_config.softLimit.reverseSoftLimitEnabled(True)

    k_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    k_config.smartCurrentLimit(40)

    k_follower_config = SparkMaxConfig()
    k_follower_config.follow(k_CAN_id, invert=True)
    k_follower_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)

    k_timeofflight = 14 #elevator time of flight CAN ID
    
    # sim elevator
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


class VisionConstants:

    k_counter_offset = 6
    k_nt_debugging = False  # print extra values to NT for debugging
    k_pi_names = ["top_pi"]


class LedConstants:

    k_counter_offset = 7
    k_nt_debugging = False  # print extra values to NT for debugging
    k_led_count = 40  # correct as of 2025 0305
    k_led_count_ignore = 4  # flat ones not for the height indicator
    k_led_pwm_port = 0  # correct as of 2025 0305

class RobotStateConstants:

    k_counter_offset = 8
    k_nt_debugging = False  # print extra values to NT for debugging

class DrivetrainConstants:

    k_counter_offset = 9
    k_nt_debugging = False  # print extra values to NT for debugging
    # these are for the apriltags.  For the most part, you want to trust the gyro, not the tags for angle
    # based on https://www.chiefdelphi.com/t/swerve-drive-pose-estimator-and-add-vision-measurement-using-limelight-is-very-jittery/453306/13
    k_pose_stdevs_large = (2, 2, 50)  # use when you don't trust the april tags - stdev x, stdev y, stdev theta
    k_pose_stdevs_small = (0.1, 0.1, 10)  # use when you do trust the tags
