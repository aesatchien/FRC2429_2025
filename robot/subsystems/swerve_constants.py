import math
from pathplannerlib.auto import PathConstraints
from pathplannerlib.controller import PPHolonomicDriveController
import wpilib
from wpimath import units
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from rev import SparkMax, SparkFlex, SparkFlexConfig, SparkMaxConfig
from pathplannerlib.config import DCMotor, PIDConstants

class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    k_drive_controller_type = SparkFlex
    kMaxSpeedMetersPerSecond = 4.75  # Sanjith started at 3.7, 4.25 was Haochen competition, 4.8 is full out on NEOs
    kMaxAngularSpeed = 0.75 * math.tau # 0.5 * math.tau  # radians per second was 0.5 tau through AVR - too slow
    # TODO: actually figure out what the total max speed should be - vector sum?
    kMaxTotalSpeed = 1.1 * math.sqrt(2) * kMaxSpeedMetersPerSecond  # sum of angular and rotational, should probably do hypotenuse
    # set the acceleration limits used in driving using the SlewRateLimiter tool
    kMagnitudeSlewRate = 5  # hundred percent per second (1 = 100%)
    kRotationalSlewRate = 5  # hundred percent per second (1 = 100%)
    k_inner_deadband = 0.10  # use deadbands for joystick transformations and keepangle calculations
    k_outer_deadband = 0.95  # above this you just set it to 1 - makes going diagonal easier
    # k_minimum_rotation = kMaxAngularSpeed * k_inner_deadband

    k_swerve_state_messages = True # these currently send the pose data to the sim - keep them on

    # Chassis configuration - not sure it even matters if we're square because wpilib accounts for it
    # MK4i modules have the centers of the wheels 2.5" from the edge, so this is robot length (or width) minus 5
    kTrackWidth = units.inchesToMeters(23.0)  # Distance between centers of right and left wheels on robot
    kWheelBase = units.inchesToMeters(23.0)   # Distance between front and back wheels on robot

    # kinematics gets passed [self.frontLeft, self.frontRight, self.rearLeft, self.rearRight]
    # Front left is X+Y+, Front right is + -, Rear left is - +, Rear right is - - (otherwise odometery is wrong)
    # this should be left as the convention, so match the above.  Then take care of turning issues with the
    # INVERSION OF THE TURN OR DRIVE MOTORS, GYRO and ABSOLUTE ENCODERS
    swerve_orientation = [(1, 1), (1, -1), (-1, 1), (-1, -1)]  # MAKE SURE ANGLE ENCODERS ARE CCW +
    kModulePositions = [
        Translation2d(swerve_orientation[0][0]*kWheelBase / 2, swerve_orientation[0][1]*kTrackWidth / 2),
        Translation2d(swerve_orientation[1][0]*kWheelBase / 2, swerve_orientation[1][1]*kTrackWidth / 2),
        Translation2d(swerve_orientation[2][0]*kWheelBase / 2, swerve_orientation[2][1]*kTrackWidth / 2),
        Translation2d(swerve_orientation[3][0]*kWheelBase / 2, swerve_orientation[3][1]*kTrackWidth / 2),
    ]

    # set up kinematics object for swerve subsystem
    kDriveKinematics = SwerveDrive4Kinematics(*kModulePositions)

    # which motors need to be inverted - depends on if on top or bottom
    k_drive_motors_inverted = True  # False for 2023 - motors below
    k_turn_motors_inverted = True  # True for 2023 - motors below
    # incorrect gyro inversion will make the pose odometry have the wrong sign on rotation
    kGyroReversed = True  # False for 2023 (was upside down), True for 2024?
    # used in the swerve modules themselves to reverse the direction of the analog encoder
    # note turn motors and analog encoders must agree - or you go haywire
    k_reverse_analog_encoders = False  # False for 2024 and probably always.

    # max absolute encoder value on each wheel - 20230322 CJH
    k_analog_encoder_abs_max = 0.989  # determined by filtering and watching as it flips from 1 to 0
    # we pass this next one to the analog potentiometer object t0 determine the full range
    k_analog_encoder_scale_factor = 1 / k_analog_encoder_abs_max  # 1.011  #so have to scale back up to be b/w 0 and 1

    # absolute encoder values when wheels facing forward  - 20230322 CJH
    # NOW IN RADIANS to feed right to the AnalogPotentiometer on the module
    k_lf_zero_offset = k_analog_encoder_scale_factor * math.tau * (0.988) # (0.532)  #  rad
    k_rf_zero_offset = k_analog_encoder_scale_factor * math.tau * (0.573) # (0.937)  #  rad   billet gear out on rf
    k_lb_zero_offset = k_analog_encoder_scale_factor * math.tau * (0.937) # (0.573)  #  rad
    k_rb_zero_offset = k_analog_encoder_scale_factor * math.tau * (0.532) # (0.988)  #  rad  billet gear out on rb
    k_analog_encoder_offsets = {'lf':0.829, 'rf':0.783, 'lb':0.304, 'rb':0.986}  # use in sim

    # SPARK MAX CAN IDs  - checked for correctness 2025 0317
    kFrontLeftDrivingCanId = 27
    kRearLeftDrivingCanId = 25
    kFrontRightDrivingCanId = 23
    kRearRightDrivingCanId = 21

    kFrontLeftTurningCanId = 26
    kRearLeftTurningCanId = 24
    kFrontRightTurningCanId = 22
    kRearRightTurningCanId = 20

    kFrontLeftAbsEncoderPort = 3
    kFrontRightAbsEncoderPort = 2
    kBackLeftAbsEncoderPort = 1
    kBackRightAbsEncoderPort = 0

    # kFrontLeftDrivingCanId = 21
    # kRearLeftDrivingCanId = 23
    # kFrontRightDrivingCanId = 25
    # kRearRightDrivingCanId = 27
    #
    # kFrontLeftTurningCanId = 20
    # kRearLeftTurningCanId = 22
    # kFrontRightTurningCanId = 24
    # kRearRightTurningCanId = 26

    # kFrontLeftAbsEncoderPort = 0
    # kFrontRightAbsEncoderPort = 1
    # kBackLeftAbsEncoderPort = 2
    # kBackRightAbsEncoderPort = 3

class NeoMotorConstants:
    kFreeSpeedRpm = 6784  # neo is 5676, vortex is 6784

class ModuleConstants:

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 4 * 0.0254  #  0.1016  =  four inches
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = 6.75 #8.14 #6.75  # From MK4i website, L2  #  From (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

    k_wheel_cof = 1 # (https://www.vexrobotics.com/colsonperforma.html)
    k_drive_motor = DCMotor.neoVortex(1).withReduction(kDrivingMotorReduction)
    k_max_current_amps = 60 # (https://www.chiefdelphi.com/t/current-limiting-on-swerve/ first post)


    kDrivingEncoderPositionFactor = (kWheelDiameterMeters * math.pi) / kDrivingMotorReduction  # meters
    kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0  # meters per second

    k_turning_motor_gear_ratio = 150/7  #  not needed when we switch to absolute encoder of 150/7
    kTurningEncoderPositionFactor = math.tau / k_turning_motor_gear_ratio # radian
    kTurningEncoderVelocityFactor = kTurningEncoderPositionFactor / 60.0  # radians per second

    kDrivingP = 0
    kDrivingI = 0
    kDrivingD = 0
    kDrivingFF = 1 / kDriveWheelFreeSpeedRps  # CJH tested 3/19/2023, works ok  - 0.2235
    # print(f'kdrivingFF: {kDrivingFF}')
    kDrivingMinOutput = -0.96 # why is this here?
    kDrivingMaxOutput = 0.96
    k_smartmotion_max_velocity = 3  # m/s
    k_smartmotion_max_accel = 2  # m/s/s

    # todo: put common constants across controllers into constants.py

    k_driving_config = SparkFlexConfig()
    k_driving_config.closedLoop.pidf(p=0, i=0, d=0, ff=1/kDriveWheelFreeSpeedRps)
    k_driving_config.closedLoop.minOutput(-0.96)
    k_driving_config.closedLoop.maxOutput(0.96)
    k_driving_config.closedLoop.IZone(0.001)
    k_driving_config.closedLoop.maxMotion.maxVelocity(3)
    k_driving_config.closedLoop.maxMotion.maxAcceleration(2)
    k_driving_config.setIdleMode(idleMode=SparkFlexConfig.IdleMode.kBrake)
    k_driving_config.smartCurrentLimit(stallLimit=0)
    k_driving_config.voltageCompensation(12)
    k_driving_config.encoder.positionConversionFactor((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) # meters
    k_driving_config.encoder.velocityConversionFactor((kWheelDiameterMeters * math.pi) / ( kDrivingMotorReduction * 60)) # meters per second
    # k_driving_config.closedLoop.pidf(0, 0, 0, 0.01)

    # note: we don't use any spark pid or ff for turning
    k_turning_config = SparkMaxConfig()
    k_turning_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    k_turning_config.smartCurrentLimit(stallLimit=0)
    k_turning_config.voltageCompensation(12)

    # nor do we use this encoder-- we configure it "just to watch it if we need to for velocities, etc."

    k_turning_config.encoder.positionConversionFactor(math.tau/k_turning_motor_gear_ratio) # radian
    k_turning_config.encoder.velocityConversionFactor(math.tau/(k_turning_motor_gear_ratio * 60)) # radians per second

    kTurningP = 0.3 #  CJH tested this 3/19/2023  and 0.25 was good
    kTurningI = 0.0
    kTurningD = 0.0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    # used for configuring and burning the flash - I think we only need slot 0 - we only do velocity control?
    k_PID_dict_vel = {'kP': kDrivingP, 'kI': kDrivingI, 'kD': kDrivingD, 'kIz': 0.001, 'kFF': kDrivingFF, 'kArbFF':0, 'kMaxOutput': kDrivingMaxOutput,
                'kMinOutput': kDrivingMinOutput, 'SM_MaxVel':k_smartmotion_max_velocity, 'SM_MaxAccel':k_smartmotion_max_accel}

    kDrivingMotorIdleMode = DriveConstants.k_drive_controller_type.IdleMode.kBrake
    kTurningMotorIdleMode = SparkMax.IdleMode.kCoast  # for now it's easier to move by hand when testing

    # 2024 0414 CJH - 80A allows the drive motors to pull WAY too much and we brown out (AVR)
    kDrivingMotorCurrentLimit = 60  # amp - set to 50 for worlds to make sure no brownouts - maybe 60 will still be safe
    kTurningMotorCurrentLimit = 40  # amp

class AutoConstants:

    k_pathplanner_translation_pid_constants = PIDConstants(kP=6, kI=0, kD=0)
    if wpilib.RobotBase.isReal():
        # CJH - IT SHOULD BE OBVIOUS THAT THE DRIVETRAIN IS NOT SET UP CORRECTLY IF YOU HAVE TO DO THIS
        k_pathplanner_rotation_pid_constants = PIDConstants(kP=4, kI=0, kD=0) # HACK: kp should not be negative lmao

    else:
        k_pathplanner_rotation_pid_constants = PIDConstants(kP=4, kI=0, kD=0)

    k_pathplanner_holonomic_controller = PPHolonomicDriveController(
            translation_constants=k_pathplanner_translation_pid_constants,
            rotation_constants=k_pathplanner_rotation_pid_constants,
    )

    k_pathfinding_constraints = PathConstraints(
            maxVelocityMps=1.5,
            maxAccelerationMpsSq=4,
            maxAngularVelocityRps=2*math.pi,  # radians per second
            maxAngularAccelerationRpsSq=4*math.pi,  # radians per second squared
            nominalVoltage=12,

    )

    k_rotation_tolerance = Rotation2d(math.radians(0))
    k_translation_tolerance_meters = 5 / 100

