import math
import typing

import navx
import ntcore
from commands2 import Subsystem

from wpilib import SmartDashboard, DataLogManager, DriverStation, PowerDistribution, Timer, RobotBase
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Pose3d, Rotation3d, Translation3d
from wpimath.kinematics import (ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.controller import PIDController

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, PathPlannerPath
from pathplannerlib.config import ModuleConfig, RobotConfig

import constants
from .swervemodule_2429 import SwerveModule
from .swerve_constants import DriveConstants as dc, AutoConstants as ac, ModuleConstants as mc
from helpers.utilities import compare_motors

class Swerve (Subsystem):
    def __init__(self, questnav) -> None:
        super().__init__()
        self.counter = 0
        self.questnav = questnav  #  pass in the questnav subsystem so we can query it in periodic
        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)  # check power and other issues

        # ----------  initialize swerve modules  ----------
        self.swerve_modules = []
        sd = dc.swerve_dict
        module_config = ([  # has to be LF, RF, LB, RB in that order
            [sd['LF']['driving_can'], sd['LF']['turning_can'], sd['LF']['port'], sd['LF']['turning_offset'], 'lf'],
            [sd['RF']['driving_can'], sd['RF']['turning_can'], sd['RF']['port'], sd['RF']['turning_offset'], 'rf'],
            [sd['LB']['driving_can'], sd['LB']['turning_can'], sd['LB']['port'], sd['LB']['turning_offset'], 'lb'],
            [sd['RB']['driving_can'], sd['RB']['turning_can'], sd['RB']['port'], sd['RB']['turning_offset'], 'rb'],
        ])
        for drive_id, turn_id, enc_port, offset, label in module_config:
            self.swerve_modules.append(SwerveModule(
                drivingCANId=drive_id, turningCANId=turn_id, encoder_analog_port=enc_port,
                turning_encoder_offset=offset, label=label))
        self.frontLeft, self.frontRight, self.rearLeft, self.rearRight = self.swerve_modules

        # let's make sure we're getting the right properties in the swerves
        compare_motors(self.frontLeft.drivingSpark, self.frontLeft.turningSpark, name_a='LF DRIVE', name_b='LF TURN')

        # ---------- set up gyro   ----------
        self.gyro = navx.AHRS.create_spi()
        if self.gyro.isCalibrating():
            # schedule a command to reset the navx
            print('unable to reset navx: Calibration in progress')
        else:
            pass

        self.gyro.zeroYaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
        self.gyro_calibrated = False

        # ---------- timer and variables for checking if we should be using pid on rotation ----------
        self.keep_angle = 0.0  # the heading we try to maintain when not rotating
        self.keep_angle_timer = Timer()
        self.keep_angle_timer.start()
        self.keep_angle_timer.reset()
        self.keep_angle_pid = PIDController(0.015, 0, 0)  # todo: put these in constants.  allow 1% stick per degree
        self.keep_angle_pid.enableContinuousInput(-180, 180)  # using the gyro's yaw is b/w -180 and 180
        self.last_rotation_time = 0
        self.time_since_rotation = 0
        self.last_drive_time = 0
        self.time_since_drive = 0

        # ---------- rate limiters  ----------  # TODO - centralize all tis
        self.fwd_magLimiter = SlewRateLimiter(0.9 * dc.kMagnitudeSlewRate)
        self.strafe_magLimiter = SlewRateLimiter(dc.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(dc.kRotationalSlewRate)

        # ---------- see if the asymmetry in the controllers is an issue for AJ  - 20250311 CJH ----------
        # update this in calibrate_joystick, and use in drive_by_joystick
        self.thrust_calibration_offset = 0
        self.strafe_calibration_offset = 0

        # ---------- pose estimator  ----------
        self.pose_estimator = SwerveDrive4PoseEstimator(dc.kDriveKinematics,
                                 Rotation2d.fromDegrees(self.get_gyro_angle()),                                                        self.get_module_positions(),
                                    initialPose=Pose2d(constants.k_start_x, constants.k_start_y,
                                    Rotation2d.fromDegrees(self.get_gyro_angle())))

        # ---------- Vision / NT  ----------
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.use_CJH_apriltags = constants.k_use_CJH_tags  # down below we decide which one to use in the periodic method
        
        self.camera_names = ['ArducamBack', 'ArducamHigh', 'GeniusLow', 'LogitechReef']
        self.pose_subscribers = [self.inst.getDoubleArrayTopic(f"/Cameras/{cam}/poses/tag1").subscribe([0] * 8) for cam in self.camera_names]
        self.count_subscribers = [self.inst.getDoubleTopic(f"/Cameras/{cam}/tags/targets").subscribe(0) for cam in self.camera_names]

        # -------------  Pathplanner section --------------
        robot_config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(
                pose_supplier=self.get_pose,
                reset_pose=self.resetOdometry,
                robot_relative_speeds_supplier=self.get_relative_speeds,
                output=self.drive_robot_relative,
                controller=ac.k_pathplanner_holonomic_controller,
                robot_config=robot_config,
                should_flip_path=self.flip_path,
                drive_subsystem=self
        )

        self.automated_path = None

        # ------------- Advantagescope section -------------
        if constants.k_enable_logging:
            DataLogManager.start()  # start wpilib datalog for AdvantageScope

    # ----------  pose and odometry function definitions ----------
    def get_pose(self) -> Pose2d:
        # return the pose of the robot  TODO: update the dashboard here?
        return self.pose_estimator.getEstimatedPosition()

    def resetOdometry(self, pose: Pose2d) -> None:
        self.pose_estimator.resetPosition(Rotation2d.fromDegrees(self.get_gyro_angle()), self.get_module_positions(), pose)

    def drive(self, xSpeed: float, ySpeed: float, rot: float, fieldRelative: bool, rate_limited: bool, keep_angle:bool=True) -> None:
        """Method to drive the robot using joystick info.
        :param xSpeed:        Speed of the robot in the x direction (forward).
        :param ySpeed:        Speed of the robot in the y direction (sideways).
        :param rot:           Angular rate of the robot.
        :param fieldRelative: Whether the provided x and y speeds are relative to the field.
        :param rateLimit:     Whether to enable rate limiting for smoother control.
        """

        if rate_limited:
            xSpeedCommanded = self.fwd_magLimiter.calculate(xSpeed)
            ySpeedCommanded = self.strafe_magLimiter.calculate(ySpeed)
            rotation_commanded = self.rotLimiter.calculate(rot)
        else:
            xSpeedCommanded = xSpeed
            ySpeedCommanded = ySpeed
            rotation_commanded = rot

        if keep_angle:
            rotation_commanded = self.perform_keep_angle(xSpeed, ySpeed, rot)  # call the 1706 keep angle routine to maintain rotation

        # Convert the commanded speeds into the correct units for the drivetrain
        xSpeedDelivered = xSpeedCommanded * dc.kMaxSpeedMetersPerSecond
        ySpeedDelivered = ySpeedCommanded * dc.kMaxSpeedMetersPerSecond
        rotDelivered = rotation_commanded * dc.kMaxAngularSpeed

        # create the swerve state array depending on if we are field relative or not
        swerveModuleStates = dc.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(self.get_angle()),)
            if fieldRelative else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered))

        # normalize wheel speeds so we do not exceed our speed limit
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, dc.kMaxTotalSpeed)
        for state, module in zip(swerveModuleStates, self.swerve_modules):
            module.setDesiredState(state)


    # ----------  keepangle function definitions ----------
    def reset_keep_angle(self):
        self.last_rotation_time = self.keep_angle_timer.get()  # reset the rotation time
        self.last_drive_time = self.keep_angle_timer.get()  # reset the drive time

        new_angle = self.get_angle()
        print(f'  resetting keep angle from {self.keep_angle:.1f} to {new_angle:.1f}', flush=True)
        self.keep_angle = new_angle

    def perform_keep_angle(self, xSpeed, ySpeed, rot):  # update rotation if we are drifting when trying to drive straight
        output = rot  # by default we will return rot unless it needs to be changed
        if math.fabs(rot) > dc.k_inner_deadband:  # we are actually intending to rotate
            self.last_rotation_time = self.keep_angle_timer.get()
        if math.fabs(xSpeed) > dc.k_inner_deadband or math.fabs(ySpeed) > dc.k_inner_deadband:
            self.last_drive_time = self.keep_angle_timer.get()

        self.time_since_rotation = self.keep_angle_timer.get() - self.last_rotation_time
        self.time_since_drive = self.keep_angle_timer.get() - self.last_drive_time

        if self.time_since_rotation < 0.5:  # (update keep_angle until 0.5s after rotate command stops to allow rotate to finish)
            self.keep_angle = self.get_angle()
        elif math.fabs(rot) < dc.k_inner_deadband and self.time_since_drive < 0.25:  # stop keep_angle .25s after you stop driving
            output = self.keep_angle_pid.calculate(self.get_angle(), self.keep_angle)  # 2024 real, can we just use YAW always?
            output = output if math.fabs(output) < 0.2 else 0.2 * math.copysign(1, output)  # clamp at 0.2

        return output

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        angles = [45, -45, -45, 45]

        for angle, swerve_module in zip(angles, self.swerve_modules):
            swerve_module.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(angle)))

    def set_straight(self):
        """Sets the wheels straight so we can push the robot."""
        angles = [0, 0, 0, 0]
        for angle, swerve_module in zip(angles, self.swerve_modules):
            swerve_module.setDesiredState(SwerveModuleState(0, Rotation2d.fromDegrees(angle)))

    def setModuleStates(self, desiredStates: typing.Tuple[SwerveModuleState]) -> None:
        desiredStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, dc.kMaxTotalSpeed)
        for idx, m in enumerate(self.swerve_modules):
            m.setDesiredState(desiredStates[idx])

    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        [m.resetEncoders() for m in self.swerve_modules]

    def get_module_positions(self):
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [m.getPosition() for m in self.swerve_modules]

    def get_module_states(self):
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [m.getState() for m in self.swerve_modules]

    #  -------------  gyro functions  ----------

    def get_raw_angle(self):  # never reversed value for using PIDs on the heading
        return self.gyro.getAngle()

    def get_gyro_angle(self):  # if necessary reverse the heading for swerve math
        # note this does add in the current offset
        return -self.gyro.getAngle() if dc.kGyroReversed else self.gyro.getAngle()

    def get_angle(self):  # if necessary reverse the heading for swerve math
        # used to be get_gyro_angle but LHACK changed it 12/24/24 so we don't have to manually reset gyro anymore
        return self.get_pose().rotation().degrees()

    def get_yaw(self):  # helpful for determining nearest heading parallel to the wall
        # but you should probably never use this - just use get_angle to be consistent
        return -self.gyro.getYaw() if dc.kGyroReversed else self.gyro.getYaw()  #2024 possible update

    def get_pitch(self):  # need to calibrate the navx, apparently
        pitch_offset = 0
        return self.gyro.getPitch() - pitch_offset

    def get_roll(self):  # need to calibrate the navx, apparently
        roll_offset = 0
        return self.gyro.getRoll() - roll_offset

    def reset_gyro(self, adjustment=None):
        self.gyro.reset()
        if adjustment is not None:
            # ADD adjustment - e.g trying to update the gyro from a pose
            self.gyro.setAngleAdjustment(adjustment)
        else:
            # make sure there is no adjustment
            self.gyro.setAngleAdjustment(0)
        self.reset_keep_angle()

    #  -------------  simulation helpers  ----------
    def get_desired_swerve_module_states(self) -> list[SwerveModuleState]:
        """
        what it says on the wrapper; it's for physics.py because I don't like relying on an NT entry
        to communicate between them (it's less clear what the NT entry is there for, I think) LHACK 1/12/25
        """
        return [module.getDesiredState() for module in self.swerve_modules]


    #  -------------  METHODS PATHPLANNER NEEDS  ----------
    def get_relative_speeds(self):
        return dc.kDriveKinematics.toChassisSpeeds(self.get_module_states())

    def drive_robot_relative(self, chassis_speeds: ChassisSpeeds, feedforwards):
        # required for the pathplanner lib's pathfollowing based on chassis speeds
        # idk if we need the feedforwards
        swerveModuleStates = dc.kDriveKinematics.toSwerveModuleStates(chassis_speeds)
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, dc.kMaxTotalSpeed)
        for state, module in zip(swerveModuleStates, self.swerve_modules):
            module.setDesiredState(state)

    def flip_path(self):  # pathplanner needs a function to see if it should mirror a path
        if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
            return False
        else:
            return True
    # -------------- END PATHPLANNER STUFF  --------------

    # -------------- periodic and periodic helpers --------------
    def periodic(self) -> None:
        self.counter += 1
        ts = Timer.getFPGATimestamp()
        current_pose = self.get_pose()  # Optimization: Cache pose to avoid recalculating it below

        self._update_vision_measurements(current_pose, ts)
        self._update_odometry(ts)
        
        if self.counter % 10 == 0:
            self._update_dashboard(current_pose, ts)

    def _update_vision_measurements(self, current_pose, ts):
        """
        # QuestNav Logic
        if self.questnav.use_quest and self.questnav.quest_has_synched and self.counter % 5 == 0:
            quest_accepted = self.questnav.is_pose_accepted()
            quest_pose = self.questnav.get_pose().transformBy(self.questnav.quest_to_robot)  # TODO make part of Quest
            delta_pos = Translation2d.distance(current_pose.translation(), quest_pose.translation())
            if delta_pos < 5 and quest_accepted:  # if the quest is way off, we don't want to update from it
                self.pose_estimator.addVisionMeasurement(quest_pose, ts, constants.DrivetrainConstants.k_pose_stdevs_disabled)
        """
        
        # AprilTag Logic
        if self.use_CJH_apriltags:
            for count_subscriber, pose_subscriber in zip(self.count_subscribers, self.pose_subscribers):
                if count_subscriber.get() > 0:  # use this camera's tag
                    tag_data = pose_subscriber.get()  # 8 items - timestamp, id, tx ty tx rx ry rz
                    tx, ty, tz = tag_data[2], tag_data[3], tag_data[4]
                    rx, ry, rz = tag_data[5], tag_data[6], tag_data[7]
                    tag_pose = Pose3d(Translation3d(tx, ty, tz), Rotation3d(rx, ry, rz)).toPose2d()

                    use_tag = constants.k_use_CJH_tags  # can disable this in constants
                    use_tag = False if self.gyro.getRate() > 90 else use_tag  # no more than n degrees per second turning if using a tag

                    if use_tag:
                        sdevs = constants.DrivetrainConstants.k_pose_stdevs_large if DriverStation.isEnabled() else constants.DrivetrainConstants.k_pose_stdevs_disabled
                        self.pose_estimator.addVisionMeasurement(tag_pose, tag_data[0], sdevs)

    def _update_odometry(self, ts):
        if RobotBase.isReal():
            self.pose_estimator.updateWithTime(ts, Rotation2d.fromDegrees(self.get_gyro_angle()), self.get_module_positions(),)

    def _update_dashboard(self, pose, ts):
        SmartDashboard.putNumber('_timestamp', ts)  # TODO - see if NT is already doing this
        SmartDashboard.putNumberArray('drive_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])
        SmartDashboard.putNumberArray('drive_pose_AdvScope', [pose.x, pose.y, pose.rotation().radians()])
        SmartDashboard.putNumber('drive_x', pose.X())
        SmartDashboard.putNumber('drive_y', pose.Y())
        SmartDashboard.putNumber('drive_theta', pose.rotation().degrees())

        SmartDashboard.putNumber('_navx', self.get_angle())
        SmartDashboard.putNumber('_navx_yaw', self.get_yaw())
        SmartDashboard.putNumber('_navx_angle', self.get_gyro_angle())
        SmartDashboard.putNumber('keep_angle', self.keep_angle)

        # post yaw, pitch, roll so we can see what is going on with the climb
        ypr = [self.gyro.getYaw(), self.get_pitch(), self.gyro.getRoll(), self.gyro.getRotation2d().degrees()]
        SmartDashboard.putNumberArray('_navx_YPR', ypr)

        # Monitor Power
        SmartDashboard.putNumber('_pdh_voltage', self.pdh.getVoltage())
        SmartDashboard.putNumber('_pdh_current', self.pdh.getTotalCurrent())

        if constants.k_swerve_debugging_messages:
            angles = [m.turningEncoder.getPosition() for m in self.swerve_modules]
            absolutes = [m.get_turn_encoder() for m in self.swerve_modules]
            for idx, absolute in enumerate(absolutes):
                SmartDashboard.putNumber(f"absolute {idx}", absolute)
            SmartDashboard.putNumberArray(f'_angles', angles)
