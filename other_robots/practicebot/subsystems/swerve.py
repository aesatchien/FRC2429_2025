import math
import typing

import navx
import ntcore

from commands2 import Subsystem, InstantCommand

from wpilib import SmartDashboard, DataLogManager, DriverStation, Field2d, PowerDistribution, Timer, RobotBase
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Pose3d, Rotation3d, Translation3d, Transform2d, Transform3d
from wpimath.kinematics import (ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.controller import PIDController


from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, PathPlannerPath
from pathplannerlib.config import ModuleConfig, RobotConfig

# from photonlibpy import PhotonCamera, PhotonPoseEstimator, PoseStrategy  #  2025 is first time for us

import robotpy_apriltag as ra

import constants
from .swervemodule_2429 import SwerveModule
from .swerve_constants import DriveConstants as dc, AutoConstants as ac, ModuleConstants as mc


class Swerve (Subsystem):
    def __init__(self, questnav) -> None:
        super().__init__()

        self.counter = 0

        self.questnav = questnav  #  pass in the questnav subsystem so we can query it in periodic

        self.pdh = PowerDistribution(1, PowerDistribution.ModuleType.kRev)  # check power and other issues

        # Create SwerveModules
        self.frontLeft = SwerveModule(
            drivingCANId=dc.kFrontLeftDrivingCanId, turningCANId=dc.kFrontLeftTurningCanId,
            encoder_analog_port=dc.kFrontLeftAbsEncoderPort, turning_encoder_offset=dc.k_lf_zero_offset,
            driving_inverted=dc.k_drive_motors_inverted, turning_inverted=dc.k_turn_motors_inverted, label= 'lf' )
        self.frontRight = SwerveModule(
            dc.kFrontRightDrivingCanId, dc.kFrontRightTurningCanId, dc.kFrontRightAbsEncoderPort, dc.k_rf_zero_offset,
            dc.k_drive_motors_inverted, dc.k_turn_motors_inverted, label='rf')
        self.rearLeft = SwerveModule(
            dc.kRearLeftDrivingCanId, dc.kRearLeftTurningCanId, dc.kBackLeftAbsEncoderPort, dc.k_lb_zero_offset,
            dc.k_drive_motors_inverted, dc.k_turn_motors_inverted, label='lb')
        self.rearRight = SwerveModule(
            dc.kRearRightDrivingCanId, dc.kRearRightTurningCanId, dc.kBackRightAbsEncoderPort, dc.k_rb_zero_offset,
            dc.k_drive_motors_inverted, dc.k_turn_motors_inverted, label='rb')

        # let's make this pythonic so we can do things quickly and with readability
        self.swerve_modules = [self.frontLeft, self.frontRight, self.rearLeft, self.rearRight]

        # The gyro sensor
        self.gyro = navx.AHRS.create_spi()
        if self.gyro.isCalibrating():
            # schedule a command to reset the navx
            print('unable to reset navx: Calibration in progress')
        else:
            pass

        #  stupid gyro never resets on boot
        self.gyro.zeroYaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
        self.gyro_calibrated = False

        # timer and variables for checking if we should be using pid on rotation
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

        self.fwd_magLimiter = SlewRateLimiter(0.9 * dc.kMagnitudeSlewRate)
        self.strafe_magLimiter = SlewRateLimiter(dc.kMagnitudeSlewRate)
        self.rotLimiter = SlewRateLimiter(dc.kRotationalSlewRate)

        # see if the asymmetry in the controllers is an issue for AJ  - 20250311 CJH
        # update this in calibrate_joystick, and use in drive_by_joystick
        self.thrust_calibration_offset = 0
        self.strafe_calibration_offset = 0

        # Odometry class for tracking robot pose
        # when we boot should we always be at zero angle?
        # self.odometry = SwerveDrive4Odometry(
        #     dc.kDriveKinematics, Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions(),
        #     initialPose=Pose2d(constants.k_start_x, constants.k_start_y, Rotation2d.fromDegrees(self.get_angle())))

        # 2024 - orphan the old odometry, now use the vision enabled version of odometry instead
        self.pose_estimator = SwerveDrive4PoseEstimator(dc.kDriveKinematics,
                                                        Rotation2d.fromDegrees(self.get_gyro_angle()),
                                                        self.get_module_positions(),
            initialPose=Pose2d(constants.k_start_x, constants.k_start_y, Rotation2d.fromDegrees(self.get_gyro_angle())))

        # get poses from NT
        self.inst = ntcore.NetworkTableInstance.getDefault()


        # -----------   CJH simple apriltags  ------------
        # get poses from NT
        self.use_CJH_apriltags = constants.k_use_CJH_tags  # down below we decide which one to use in the periodic method
        # lhack turned off 15:48 2/28/25 to test pathplanner wo tags first
        self.inst = ntcore.NetworkTableInstance.getDefault()
        # TODO - move the camera names to constants
        camera_names = ['ArducamBack', 'ArducamHigh', 'GeniusLow', 'LogitechReef']
        self.pose_subscribers = [self.inst.getDoubleArrayTopic(f"/Cameras/{cam}/poses/tag1").subscribe([0] * 8) for cam in camera_names]
        self.count_subscribers = [self.inst.getDoubleTopic(f"/Cameras/{cam}/tags/targets").subscribe(0) for cam in camera_names]

        self.desired_tags = constants.VisionConstants.k_valid_tags

        # TODO - give me a list of six filters for the apriltags - smooth if we are not moving, else use reset each measurement
        # def tag_filter(window):
        #     return [wpimath.filter.LinearFilter.movingAverage(window) for _ in range(6) ]
        # window = 5
        # self.tag_motion_filters = [tag_filter(window) for _ in self.pose_subscribers]

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
        DataLogManager.start()  # start wpilib datalog for AdvantageScope

        # Optimization: Load the field layout once on boot, not every time we ask for a tag
        self.field_layout = ra.AprilTagFieldLayout.loadField(ra.AprilTagField.k2025ReefscapeWelded)

        # end of init

    def get_pose(self) -> Pose2d:
        # return the pose of the robot  TODO: update the dashboard here?
        return self.pose_estimator.getEstimatedPosition()

    # def get_pose_no_tag(self) -> Pose2d:
    #     return self.odometry.getPose()

    def resetOdometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.
        :param pose: The pose to which to set the odometry.
        """
        # self.odometry.resetPosition(
        #     Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions(), pose)
        self.pose_estimator.resetPosition(
            Rotation2d.fromDegrees(self.get_gyro_angle()), self.get_module_positions(), pose)

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

        # probably can stop doing this now
        if dc.k_swerve_state_messages and self.counter % 50 == 0:
            SmartDashboard.putNumberArray('_xyr', [xSpeedDelivered, ySpeedDelivered, rotDelivered])
            SmartDashboard.putNumber('_swerve commanded rotation', rotDelivered)

        # create the swerve state array depending on if we are field relative or not
        swerveModuleStates = dc.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(self.get_angle()),)
            if fieldRelative else ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered))

        # normalize wheel speeds so we do not exceed our speed limit
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, dc.kMaxTotalSpeed)
        for state, module in zip(swerveModuleStates, self.swerve_modules):
            module.setDesiredState(state)


    #  -------------  THINGS PATHPLANNER NEEDS  - added for pathplanner 20230218 CJH
    def get_relative_speeds(self):
        # added for pathplanner 20230218 CJH
        return dc.kDriveKinematics.toChassisSpeeds(self.get_module_states())

    def drive_robot_relative(self, chassis_speeds: ChassisSpeeds, feedforwards):
        """c
        feedforwards isn't used at all so pass it whatever
        """
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

    # -------------- END PATHPLANNER STUFF

    def reset_keep_angle(self):
        """
        perhaps deprecated because we want to use resetOdometry to reset the gyro
        """
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
            self.keep_angle = self.get_angle()  # todo: double check SIGN (and units are in degrees)
        elif math.fabs(rot) < dc.k_inner_deadband and self.time_since_drive < 0.25:  # stop keep_angle .25s after you stop driving
            # output = self.keep_angle_pid.calculate(-self.get_angle(), self.keep_angle)  # 2023
            # TODO: figure out if we want YAW or ANGLE, and WHY NOT BE CONSISTENT WITH YAW AND ANGLE?
            output = self.keep_angle_pid.calculate(self.get_angle(), self.keep_angle)  # 2024 real, can we just use YAW always?
            output = output if math.fabs(output) < 0.2 else 0.2 * math.copysign(1, output)  # clamp at 0.2

        if self.counter % 20 == 0:
            SmartDashboard.putNumber('keep_angle_output', output)

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
        """Sets the swerve ModuleStates.
        :param desiredStates: The desired SwerveModule states.
        """
        desiredStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, dc.kMaxTotalSpeed)
        for idx, m in enumerate(self.swerve_modules):
            m.setDesiredState(desiredStates[idx])


    def resetEncoders(self) -> None:
        """Resets the drive encoders to currently read a position of 0."""
        [m.resetEncoders() for m in self.swerve_modules]

    def zeroHeading(self) -> None:
        """Zeroes the heading of the robot."""
        self.gyro.reset()

    def getTurnRate(self) -> float:
        """Returns the turn rate of the robot.
        :returns: The turn rate of the robot, in degrees per second
        """
        return self.gyro.getRate() * (-1.0 if dc.kGyroReversed else 1.0)

    def get_module_positions(self):
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [m.getPosition() for m in self.swerve_modules]

    def get_module_states(self):
        """ CJH-added helper function to clean up some calls above"""
        # note lots of the calls want tuples, so _could_ convert if we really want to
        return [m.getState() for m in self.swerve_modules]

    def get_raw_angle(self):  # never reversed value for using PIDs on the heading
        return self.gyro.getAngle()

    def get_gyro_angle(self):  # if necessary reverse the heading for swerve math
        # note this does add in the current offset
        # print(f"get_gyro_angle is returning {-self.gyro.getAngle() if dc.kGyroReversed else self.gyro.getAngle()}")
        return -self.gyro.getAngle() if dc.kGyroReversed else self.gyro.getAngle()

    def get_angle(self):  # if necessary reverse the heading for swerve math
        # used to be get_gyro_angle but LHACK changed it 12/24/24 so we don't have to manually reset gyro anymore
        return self.get_pose().rotation().degrees()

    def get_yaw(self):  # helpful for determining nearest heading parallel to the wall
        # but you should probably never use this - just use get_angle to be consistent
        # because yaw does NOT return the offset that get_Angle does
        # return self.gyro.getYaw()
        return -self.gyro.getYaw() if dc.kGyroReversed else self.gyro.getYaw()  #2024 possible update

    def get_pitch(self):  # need to calibrate the navx, apparently
        pitch_offset = 0
        return self.gyro.getPitch() - pitch_offset

    def get_roll(self):  # need to calibrate the navx, apparently
        roll_offset = 0
        return self.gyro.getRoll() - roll_offset

    def reset_gyro(self, adjustment=None):  # use this from now on whenever we reset the gyro
        """
        perhaps deprecated because we want to use resetOdometry to reset the gyro 1/12/25 LHACK
        """
        self.gyro.reset()
        if adjustment is not None:
            # ADD adjustment - e.g trying to update the gyro from a pose
            self.gyro.setAngleAdjustment(adjustment)
        else:
            # make sure there is no adjustment
            self.gyro.setAngleAdjustment(0)
        self.reset_keep_angle()
    

    # figure out the nearest stage - or any tag, I suppose if we pass in a list
    def get_nearest_tag(self, destination='stage'):
        # get a field so we can query the tags
        current_pose = self.get_pose()

        if destination == 'reef':
            # get all distances to the stage tags
            tags = [6,7,8,9,10,11,17,18,19,20,21,22]  # the ones we can see from driver's station - does not matter if red or blue
            x_offset, y_offset = -0.10, 0.10  # subtracting translations below makes +x INTO the tage, +y LEFT of tag
            robot_offset = Pose2d(Translation2d(x_offset, y_offset), Rotation2d(0))
            face_tag = True  # do we want to face the tag?
        else:
            raise ValueError('  location for get_nearest tag must be in ["stage", "amp"] etc')

        poses = [self.field_layout.getTagPose(tag).toPose2d() for tag in tags]
        distances = [current_pose.translation().distance(pose.translation()) for pose in poses]

        # sort the distances
        combined = list(zip(tags, distances))
        combined.sort(key=lambda x: x[1])  # sort on the distances
        sorted_tags, sorted_distances = zip(*combined)
        nearest_pose = self.field_layout.getTagPose(sorted_tags[0])  # get the pose of the nearest stage tag

        # transform the tag pose to our specific needs
        tag_pose = nearest_pose.toPose2d()  # work with a 2D pose
        tag_rotation = tag_pose.rotation()  # we are either going to match this or face opposite
        robot_offset_corrected = robot_offset.rotateBy(tag_rotation)  # rotate our offset so we align with the tag
        updated_translation = tag_pose.translation() - robot_offset_corrected.translation()  # careful with these signs
        updated_rotation = tag_rotation + Rotation2d(math.pi) if face_tag else tag_rotation  # choose if we flip
        updated_pose = Pose2d(translation=updated_translation, rotation=updated_rotation)  # drive to here

        print(f'  nearest {destination} is tag {sorted_tags[0]} at {nearest_pose.translation()}')
        return sorted_tags[0]  # changed this in 2025 instead of updated_pose

    def get_desired_swerve_module_states(self) -> list[SwerveModuleState]:
        """
        what it says on the wrapper; it's for physics.py because I don't like relying on an NT entry
        to communicate between them (it's less clear what the NT entry is there for, I think) LHACK 1/12/25
        """
        return [module.getDesiredState() for module in self.swerve_modules]

    def periodic(self) -> None:

        self.counter += 1

        # send our current time to the dashboard
        ts = Timer.getFPGATimestamp()
        current_pose = self.get_pose()  # Optimization: Cache pose to avoid recalculating it below

        SmartDashboard.putNumber('_timestamp', ts)  # this one we actually do every time TODO - see if this is done by wpilib and use it instead

        # moved quest stuff out, except for this - may want a command glue instead of passing it
        if self.questnav.use_quest and self.questnav.quest_has_synched and self.counter % 5 == 0:
            quest_accepted = self.questnav.is_pose_accepted()
            quest_pose = self.questnav.get_pose().transformBy(self.questnav.quest_to_robot)  # TODO make part of Quest
            delta_pos = Translation2d.distance(current_pose.translation(), quest_pose.translation())
            if delta_pos < 5 and quest_accepted:  # if the quest is way off, we don't want to update from it
                self.pose_estimator.addVisionMeasurement(quest_pose, ts, constants.DrivetrainConstants.k_pose_stdevs_disabled)

        if self.use_CJH_apriltags:  # loop through all of our subscribers above
            for count_subscriber, pose_subscriber in zip(self.count_subscribers, self.pose_subscribers):
                # print(f"count subscriber says it has {count_subscriber.get()} tags")
                if count_subscriber.get() > 0:  # use this camera's tag
                    # update pose from apriltags
                    tag_data = pose_subscriber.get()  # 8 items - timestamp, id, tx ty tx rx ry rz
                    id = tag_data[0]
                    tx, ty, tz = tag_data[2], tag_data[3], tag_data[4]
                    rx, ry, rz = tag_data[5], tag_data[6], tag_data[7]
                    tag_pose = Pose3d(Translation3d(tx, ty, tz), Rotation3d(rx, ry, rz)).toPose2d()

                    use_tag = constants.k_use_CJH_tags  # can disable this in constants
                    # do not allow large jumps when enabled
                    delta_pos = Translation2d.distance(current_pose.translation(), tag_pose.translation())
                    # 20251018 commented out the 1m sanity check in case the questnav dies - this way we can get back
                    #use_tag = False if (delta_pos > 1 and DriverStation.isEnabled()) else use_tag  # no big movements in odometry from tags
                    use_tag = False if self.gyro.getRate() > 90 else use_tag  # no more than n degrees per second turning if using a tag
                    # use_tag = False if id not in self.desired_tags else use_tag

                    # TODO - figure out ambiguity (maybe pass to NT from the pi)
                    # do i have a fatal lag issue?  am i better without the time estimate?
                    # based on https://www.chiefdelphi.com/t/swerve-drive-pose-estimator-and-add-vision-measurement-using-limelight-is-very-jittery/453306/13
                    # I gave a fairly high x and y, and a very high theta
                    if use_tag:
                        # print(f'adding vision measurement at {wpilib.getTime()}')
                        sdevs = constants.DrivetrainConstants.k_pose_stdevs_large if DriverStation.isEnabled() else constants.DrivetrainConstants.k_pose_stdevs_disabled
                        self.pose_estimator.addVisionMeasurement(tag_pose, tag_data[0], sdevs)


        # Update the odometry in the periodic block -
        if RobotBase.isReal():
            # self.odometry.update(Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions(),)
            self.pose_estimator.updateWithTime(ts, Rotation2d.fromDegrees(self.get_gyro_angle()), self.get_module_positions(),)

        # in sim, we update from physics.py
        # TODO: if we want to be cool and have spare time, we could use SparkBaseSim with FlywheelSim to do
        # actual physics simulation on the swerve modules instead of assuming perfect behavior

        if self.counter % 10 == 0:
            pose = current_pose  # self.odometry.getPose()
            if True:  # RobotBase.isReal():  # update the NT with odometry for the dashboard - sim will do its own
                SmartDashboard.putNumberArray('drive_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])
                SmartDashboard.putNumberArray('drive_pose_AdvScope', [pose.x, pose.y,pose.rotation().radians()])  # for AdvantageScope
                SmartDashboard.putNumber('drive_x', pose.X())
                SmartDashboard.putNumber('drive_y', pose.Y())
                SmartDashboard.putNumber('drive_theta', pose.rotation().degrees())

            SmartDashboard.putNumber('_navx', self.get_angle())
            SmartDashboard.putNumber('_navx_yaw', self.get_yaw())
            SmartDashboard.putNumber('_navx_angle', self.get_gyro_angle())

            SmartDashboard.putNumber('keep_angle', self.keep_angle)
                # SmartDashboard.putNumber('keep_angle_output', output)

            # post yaw, pitch, roll so we can see what is going on with the climb
            ypr = [self.gyro.getYaw(), self.get_pitch(), self.gyro.getRoll(), self.gyro.getRotation2d().degrees()]
            SmartDashboard.putNumberArray('_navx_YPR', ypr)

            # monitor power as well
            if True: # RobotBase.isReal():
                # there's some kind of voltage simulation but idk if this covers it
                voltage = self.pdh.getVoltage()
                total_current = self.pdh.getTotalCurrent()
            else:
                # make up a current based on how fast we're going
                total_current = 2 + 10 * sum([math.fabs(module.drivingEncoder.getVelocity()) for module in self.swerve_modules])
                voltage = 12.5 - 0.02 * total_current

            SmartDashboard.putNumber('_pdh_voltage', voltage)
            SmartDashboard.putNumber('_pdh_current', total_current)

            if constants.k_swerve_debugging_messages:  # this is just a bit much unless debugging the swerve
                angles = [m.turningEncoder.getPosition() for m in self.swerve_modules]
                absolutes = [m.get_turn_encoder() for m in self.swerve_modules]
                for idx, absolute in enumerate(absolutes):
                    SmartDashboard.putNumber(f"absolute {idx}", absolute)

                SmartDashboard.putNumberArray(f'_angles', angles)
                # SmartDashboard.putNumberArray(f'_analog_radians', absolutes)



