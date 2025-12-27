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

    k_robot_id = 'practice'  # used to switch between the two configs - different controllers, IDs, and abs encoder offsets
    if k_robot_id not in ['practice', 'comp']:
        raise ValueError(f'robot_id "{k_robot_id}" must be one of [comp, practice]')

    k_drive_controller_type = SparkMax if k_robot_id == 'practice' else SparkFlex

    # ---- Driving Parameters - Note that these are not the maximum possible speeds, rather the allowed maximum speeds
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

    # ---- reporting  parameters
    k_swerve_state_messages = True # these currently send the pose data to the sim - keep them on

    # ------------- KINEMATICS -------------
    # Chassis configuration - not sure if it even matters if we're square because wpilib accounts for it
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

    #  ----------------   THIS IS TRIAL AND ERROR - USE BLOCKS UNDER CHASSIS   ---------------
    # which motors need to be inverted - depends on if mounted on top (True) or bottom (False)
    # THIS IS THE FIRST CHECK - DRIVE WITH DPAD (robot relative) AND MAKE SURE DIRECTION IS CORRECT
    k_drive_motors_inverted = False  # drive forward and reverse correct?  If not, invert this.
    # THIS IS THE SECOND CHECK - HOW DO YOU TEST IT?
    k_turn_motors_inverted = True  # True for 2023 - motors below
    # incorrect gyro inversion will make the pose odometry have the wrong sign on rotation
    # IF DRIVE MOTORS ARE CORRECT AND TURN MOTORS ARE CORRECT, THEN CCW IS POSITIVE OR YOU REVERSE GYRO?
    kGyroReversed = True  # False for 2023 (was upside down), True for 2024/2025
    # used in the swerve modules themselves to reverse the direction of the analog encoder
    # note turn motors and analog encoders must agree - or you go haywire
    k_reverse_analog_encoders = False  # False for 2024 and probably always.

    # max absolute encoder value on each wheel they seem to stop at 0.99 for some reason - 20230322 CJH
    k_analog_encoder_abs_max = 1.0 # 0.990  # can we just leave it as 1?

    # we pass this next one to the analog potentiometer object to determine the full range
    # IN RADIANS to feed right to the AnalogPotentiometer on the module
    k_analog_encoder_scale_factor = math.tau * 1 / k_analog_encoder_abs_max  # 1.011 * 2pi  # so have to scale back up to be b/w 0 and 1
    sf = k_analog_encoder_scale_factor

    # SPARK controller  settings and CAN IDs  - checked for correctness 2025 0317
    # turning offset needs to be in radians, so it uses the 2pi scaling factor
    # I meant to have billet out on the right side, but it looks like i had that opposite for reefbot
    comp_bot_dict = {'LF':{'driving_can': 27, 'turning_can': 26, 'port': 3, 'turning_offset': sf * 0.484},
                    'LB':{'driving_can': 25, 'turning_can': 24, 'port': 1, 'turning_offset': sf * 0.437},
                    'RF':{'driving_can': 23, 'turning_can': 22, 'port': 2, 'turning_offset': sf *  0.068},
                    'RB':{'driving_can': 21, 'turning_can': 20, 'port': 0, 'turning_offset': sf *  0.030}}
    practice_bot_dict = {'LF':{'driving_can': 21, 'turning_can': 20, 'port': 3, 'turning_offset': sf *  0.841},
                    'LB':{'driving_can': 23, 'turning_can': 22, 'port': 1, 'turning_offset': sf *  0.718},
                    'RF':{'driving_can': 25, 'turning_can': 24, 'port': 2, 'turning_offset': sf *  0.745},  # billet out
                    'RB':{'driving_can': 27, 'turning_can': 26, 'port': 0, 'turning_offset': sf *  0.869}}  # billet out

    swerve_dict = practice_bot_dict if k_robot_id == 'practice' else  comp_bot_dict # set this to one or the other
    # print(f'swerve_dict: {swerve_dict}')

    # need the absolute encoder values when wheels facing forward  - 20230322 CJH
    analog_encoder_test_mode = False  #  set this to test the wheel alignment
    if analog_encoder_test_mode:
        print(f'YOU ARE IN ENCODER ALIGNMENT TEST MODE -- DO NOT DRIVE!!!')
        # read the raw numbers from the encoders so we can write them all down for a given robot
        k_analog_encoder_scale_factor = 1.0  # override so we get the raw reading between 0 and 1
        for key in ['LF', 'RF', 'LB', 'RB'] :
            swerve_dict[key]['turning_offset'] = 0
    else:
        # these aren't used anymore!
        pass
        # practicebot 20251224 CJH:  LF: 0.841  LB: 0.718  RF:  0.745  RB: 0.865


class NeoMotorConstants:
    kFreeSpeedRpm = 5676 if DriveConstants.k_robot_id == 'practice' else 6784   # neo is 5676, vortex is 6784

class ModuleConstants:

    # Calculations required for driving motor conversion factors and feed forward
    kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60
    kWheelDiameterMeters = 4 * 0.0254  #  0.1016  =  four inches
    kWheelCircumferenceMeters = kWheelDiameterMeters * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    kDrivingMotorReduction = 6.75 #8.14 #6.75  # From MK4i website, L2  #  From (45.0 * 22) / (kDrivingMotorPinionTeeth * 15)
    kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction

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

    # 2024 0414 CJH - 80A allows the drive motors to pull WAY too much and we brown out (AVR)
    kDrivingMotorCurrentLimit = 60  # amp - set to 50 for worlds to make sure no brownouts - maybe 60 will still be safe
    kTurningMotorCurrentLimit = 40  # amp

    k_driving_config = SparkFlexConfig() if DriveConstants.k_robot_id == 'comp' else SparkMaxConfig()
    k_driving_config.inverted(DriveConstants.k_drive_motors_inverted)
    k_driving_config.closedLoop.pidf(p=0, i=0, d=0, ff=1/kDriveWheelFreeSpeedRps)
    k_driving_config.closedLoop.minOutput(-0.96)
    k_driving_config.closedLoop.maxOutput(0.96)
    k_driving_config.closedLoop.IZone(0.001)
    k_driving_config.closedLoop.maxMotion.maxVelocity(3)
    k_driving_config.closedLoop.maxMotion.maxAcceleration(2)
    k_driving_config.setIdleMode(idleMode=SparkFlexConfig.IdleMode.kBrake)
    k_driving_config.smartCurrentLimit(stallLimit=kDrivingMotorCurrentLimit, freeLimit=kDrivingMotorCurrentLimit, limitRpm=5700)
    k_driving_config.voltageCompensation(12)
    k_driving_config.encoder.positionConversionFactor((kWheelDiameterMeters * math.pi) / kDrivingMotorReduction) # meters
    k_driving_config.encoder.velocityConversionFactor((kWheelDiameterMeters * math.pi) / ( kDrivingMotorReduction * 60)) # meters per second
    # k_driving_config.closedLoop.pidf(0, 0, 0, 0.01)

    # note: we don't use any spark pid or ff for turning
    k_turning_config = SparkFlexConfig() if DriveConstants.k_robot_id == 'comp' else SparkMaxConfig()
    k_turning_config.inverted(DriveConstants.k_turn_motors_inverted)
    k_turning_config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)
    k_turning_config.smartCurrentLimit(stallLimit=kTurningMotorCurrentLimit, freeLimit=kTurningMotorCurrentLimit, limitRpm=5700)
    k_turning_config.voltageCompensation(12)

    # nor do we use this encoder-- we configure it "just to watch it if we need to for velocities, etc."

    k_turning_config.encoder.positionConversionFactor(math.tau/k_turning_motor_gear_ratio) # radian
    k_turning_config.encoder.velocityConversionFactor(math.tau/(k_turning_motor_gear_ratio * 60)) # radians per second

    kTurningP = 0.3 #  CJH tested this 3/19/2023  and 0.25 was good.  Used in the wpilib PID controller, not rev
    kTurningI = 0.0
    kTurningD = 0.0
    kTurningFF = 0
    kTurningMinOutput = -1
    kTurningMaxOutput = 1

    # does no good ...
    #print(f'driving cfg: {k_driving_config.flatten()}')
    #print(f'turing cfg: {k_turning_config.flatten()}')


class AutoConstants:

    k_pathplanner_translation_pid_constants = PIDConstants(kP=6, kI=0, kD=0)
    k_pathplanner_rotation_pid_constants = PIDConstants(kP=4, kI=0, kD=0)  # no longer negative when swerve correct

    k_pathplanner_holonomic_controller = PPHolonomicDriveController(
            translation_constants=k_pathplanner_translation_pid_constants,
            rotation_constants=k_pathplanner_rotation_pid_constants,
    )

    # is this used anywhere?
    k_pathfinding_constraints = PathConstraints(
            maxVelocityMps=3,
            maxAccelerationMpsSq=6,  # this was at 6 for all comps - CJH lowered it to 4 for old batteries 20251006
            maxAngularVelocityRps=2*math.pi,  # radians per second
            maxAngularAccelerationRpsSq=4*math.pi,  # radians per second squared
            nominalVoltage=12,

    )

    # used as end conditions in auto to pose / pid to point
    k_rotation_tolerance = Rotation2d(math.radians(2))
    k_translation_tolerance_meters = 2 / 100

