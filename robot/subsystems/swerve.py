import math
import typing

import commands2
from pathplannerlib.controller import PPHolonomicDriveController
import wpilib

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation3d, Pose3d, Rotation3d
from wpimath.kinematics import (ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics)
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.controller import PIDController
import navx
import rev
from pathplannerlib.path import PathPlannerTrajectory
import ntcore
import robotpy_apriltag as ra
import wpimath.geometry as geo


import constants
from .swervemodule_2429 import SwerveModule
from .swerve_constants import DriveConstants as dc, AutoConstants as ac, ModuleConstants as mc

from pathplannerlib.auto import AutoBuilder, PathPlannerAuto, PathPlannerPath

from pathplannerlib.config import ModuleConfig, RobotConfig

class Swerve (Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.counter = 0

        self.pdh = wpilib.PowerDistribution(1, wpilib.PowerDistribution.ModuleType.kRev)  # check power and other issues

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
        #self.gyro = wpilib.ADIS16470_IMU()
        self.gyro = navx.AHRS.create_spi()
        self.navx = self.gyro
        if self.navx.isCalibrating():
            # schedule a command to reset the navx
            print('unable to reset navx: Calibration in progress')
        else:
            pass

        #  stupid gyro never resets on boot
        self.navx.zeroYaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
        self.gyro_calibrated = False

        # timer and variables for checking if we should be using pid on rotation
        self.keep_angle = 0.0  # the heading we try to maintain when not rotating
        self.keep_angle_timer = wpilib.Timer()
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
        self.apriltag_rear_pose_subscriber = self.inst.getDoubleArrayTopic("/Cameras/Tagcam/poses/tag1").subscribe([0] * 8)
        self.apriltag_rear_count_subscriber = self.inst.getDoubleTopic("/Cameras/Tagcam/tags/targets").subscribe(0)
        self.apriltag_front_pose_subscriber = self.inst.getDoubleArrayTopic("/Cameras/TagcamFront/poses/tag1").subscribe([0]*8)
        self.apriltag_front_count_subscriber = self.inst.getDoubleTopic("/Cameras/TagcamFront/tags/targets").subscribe(0)

        # I'm fairly confident we don't need to use the DriveFeedForwards 12/22/24 LHACK
        # module_config = ModuleConfig(
        #             wheelRadiusMeters=mc.kWheelDiameterMeters/2,
        #             maxDriveVelocityMPS=mc.kDriveWheelFreeSpeedRps * mc.kWheelCircumferenceMeters * 0.85, # the robot advances one wheel circumference per rotation. 
        #             wheelCOF=mc.k_wheel_cof,
        #             driveMotor=mc.k_drive_motor,
        #             driveCurrentLimit=mc.k_max_current_amps,
        #             numMotors=1
        # )
        #
        # robot_config = RobotConfig(
        #         massKG=constants.k_robot_mass_kg,
        #         MOI=constants.k_robot_moi,
        #         moduleConfig=module_config,
        #         moduleOffsets=dc.kModulePositions,
        #         # trackwidthMeters=dc.kTrackWidth
        # )

        robot_config = RobotConfig.fromGUISettings()

        holonomic_controller = PPHolonomicDriveController(
                translation_constants=ac.k_pathplanner_translation_pid_constants,
                rotation_constants=ac.k_pathplanner_rotation_pid_constants,
        )


        AutoBuilder.configure(
                pose_supplier=self.get_pose,
                reset_pose=self.resetOdometry,
                robot_relative_speeds_supplier=self.get_relative_speeds,
                output=self.drive_robot_relative,
                controller=holonomic_controller,
                robot_config=robot_config,
                should_flip_path=self.flip_path,
                drive_subsystem=self
        )

        self.automated_path = None

        self.field2d_for_atag_testing = wpilib.Field2d()
        wpilib.SmartDashboard.putData("fieldd", self.field2d_for_atag_testing)

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
        if dc.k_swerve_state_messages:
            wpilib.SmartDashboard.putNumberArray('_xyr', [xSpeedDelivered, ySpeedDelivered, rotDelivered])

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
        # required for the pathplanner lib's pathfollowing based on chassis speeds
        # idk if we need the feedforwards
        swerveModuleStates = dc.kDriveKinematics.toSwerveModuleStates(chassis_speeds)
        swerveModuleStates = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerveModuleStates, dc.kMaxTotalSpeed)
        for state, module in zip(swerveModuleStates, self.swerve_modules):
            module.setDesiredState(state)

    def flip_path(self):  # pathplanner needs a function to see if it should mirror a path
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kBlue:
            return False
        else:
            return True

    # def follow_pathplanner_trajectory_command(self, trajectory:PathPlannerTrajectory, is_first_path:bool):
    #     #from pathplannerlib.path import PathPlannerPath
    #     #from pathplannerlib.commands import FollowPathWithEvents, FollowPathHolonomic
    #     #from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants

    #     # copy of pathplannerlib's method for returning a swervecommand, with an optional odometry reset
    #     # using the first pose of the trajectory
    #     if is_first_path:
    #         reset_cmd = commands2.InstantCommand(self.resetOdometry(trajectory.getInitialTargetHolonomicPose()))
    #     else:
    #         reset_cmd = commands2.InstantCommand()

    #     # useful stuff controller.PPHolonomicDriveController, controller.PIDController, auto.FollowPathHolonomic
    #     swerve_controller_cmd = None

    #     cmd = commands2.SequentialCommandGroup(reset_cmd, swerve_controller_cmd)

    #     return cmd

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

        wpilib.SmartDashboard.putNumber('keep_angle_output', output)

        return output

    def setX(self) -> None:
        """Sets the wheels into an X formation to prevent movement."""
        angles = [45, -45, -45, 45]
        # angles = [0, 0, 0, 0]

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
        # return self.get_gyro_angle()
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
        field = ra.AprilTagField.k2024Crescendo
        layout = ra.loadAprilTagLayoutField(field)
        current_pose = self.get_pose()

        if destination == 'stage':
            # get all distances to the stage tags
            tags = [11, 12, 15, 16]  # the ones we can see from driver's station - does not matter if red or blue
            x_offset, y_offset = -0.10, 0.10  # subtracting translations below makes +x INTO the tage, +y LEFT of tag
            robot_offset = geo.Pose2d(geo.Translation2d(x_offset, y_offset), geo.Rotation2d(0))
            face_tag = True  # do we want to face the tag?
        elif destination == 'amp':
            tags = [5, 6]
            x_offset, y_offset = -0.37, 0  # subtracting translations below makes +1 INTO the tage, +y LEFT of tag
            robot_offset = geo.Pose2d(geo.Translation2d(x_offset, y_offset), geo.Rotation2d(0))
            face_tag = True
        elif destination == 'speaker':
            tags = [7, 4]  # right one facing blue, left one facing red
            x_offset, y_offset = -1.5, 0  # subtracting translations below makes +1 INTO the tage
            robot_offset = geo.Pose2d(geo.Translation2d(x_offset, y_offset), geo.Rotation2d(0))
            face_tag = False
        else:
            raise ValueError('  location for get_nearest tag must be in ["stage", "amp"] etc')

        poses = [layout.getTagPose(tag).toPose2d() for tag in tags]
        distances = [current_pose.translation().distance(pose.translation()) for pose in poses]

        # sort the distances
        combined = list(zip(tags, distances))
        combined.sort(key=lambda x: x[1])  # sort on the distances
        sorted_tags, sorted_distances = zip(*combined)
        nearest_pose = layout.getTagPose(sorted_tags[0])  # get the pose of the nearest stage tag

        # transform the tag pose to our specific needs
        tag_pose = nearest_pose.toPose2d()  # work with a 2D pose
        tag_rotation = tag_pose.rotation()  # we are either going to match this or face opposite
        robot_offset_corrected = robot_offset.rotateBy(tag_rotation)  # rotate our offset so we align with the tag
        updated_translation = tag_pose.translation() - robot_offset_corrected.translation()  # careful with these signs
        updated_rotation = tag_rotation + geo.Rotation2d(math.pi) if face_tag else tag_rotation  # choose if we flip
        updated_pose = geo.Pose2d(translation=updated_translation, rotation=updated_rotation)  # drive to here

        print(f'  nearest {destination} is tag {sorted_tags[0]} at {nearest_pose.translation()}')
        return updated_pose

    def get_desired_swerve_module_states(self) -> list[SwerveModuleState]:
        """
        what it says on the wrapper; it's for physics.py because I don't like relying on an NT entry
        to communicate between them (it's less clear what the NT entry is there for, I think) LHACK 1/12/25
        """
        return [module.getDesiredState() for module in self.swerve_modules]


    def periodic(self) -> None:

        self.counter += 1

        self.field2d_for_atag_testing.setRobotPose(
                x=self.inst.getEntry("SmartDashboard/pose 0 idx x").getFloat(0),
                y=self.inst.getEntry("SmartDashboard/pose 0 idx y").getFloat(0),
                rotation=Rotation2d(0)
        )

        # send the current time to the dashboard
        wpilib.SmartDashboard.putNumber('_timestamp', wpilib.Timer.getFPGATimestamp())
        # update pose based on apriltags
        if constants.k_use_apriltag_odometry:

            if self.apriltag_front_count_subscriber.get() > 0:  # use front camera
                # update pose from apriltags
                tag_data = self.apriltag_front_pose_subscriber.get()  # 8 items - timestamp, id, tx ty tx rx ry rz
                tx, ty, tz = tag_data[2], tag_data[3], tag_data[4]
                rx, ry, rz = tag_data[5], tag_data[6], tag_data[7]
                tag_pose = Pose3d(Translation3d(tx, ty, tz), Rotation3d(rx, ry, rz)).toPose2d()
                self.pose_estimator.addVisionMeasurement(tag_pose, tag_data[0])

            if self.apriltag_rear_count_subscriber.get() > 0:  # use rear camera - should I make this an elif?
                # update pose from apriltags
                tag_data = self.apriltag_rear_pose_subscriber.get()  # 8 items - timestamp, id, tx ty tx rx ry rz
                tx, ty, tz = tag_data[2], tag_data[3], tag_data[4]
                rx, ry, rz = tag_data[5], tag_data[6], tag_data[7]
                tag_pose = Pose3d(Translation3d(tx, ty, tz), Rotation3d(rx, ry, rz)).toPose2d()
                self.pose_estimator.addVisionMeasurement(tag_pose, tag_data[0])



        # Update the odometry in the periodic block -
        if wpilib.RobotBase.isReal():
            # self.odometry.update(Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions(),)
            self.pose_estimator.updateWithTime(wpilib.Timer.getFPGATimestamp(), Rotation2d.fromDegrees(self.get_gyro_angle()), self.get_module_positions(),)

        # in sim, we update from physics.py
        # TODO: if we want to be cool and have spare time, we could use SparkBaseSim with FlywheelSim to do
        # actual physics simulation on the swerve modules instead of assuming perfect behavior

        # else:
        #     # sim is not updating the odometry right yet, not sure why since all the sparks should be set in the sim
        #     # self.odometry.update(Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions(),)
        #     self.pose_estimator.update(Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions())
        #     pose = self.get_pose()
        #     wpilib.SmartDashboard.putNumberArray('real_robot_pose', [pose.x, pose.y, pose.rotation().degrees()])

        if self.counter % 10 == 0:
            pose = self.get_pose()  # self.odometry.getPose()
            if True: # wpilib.RobotBase.isReal():  # update the NT with odometry for the dashboard - sim will do its own
                wpilib.SmartDashboard.putNumberArray('drive_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])
                wpilib.SmartDashboard.putNumber('drive_x', pose.X())
                wpilib.SmartDashboard.putNumber('drive_y', pose.Y())
                wpilib.SmartDashboard.putNumber('drive_theta', pose.rotation().degrees())

            wpilib.SmartDashboard.putNumber('_navx', self.get_angle())
            wpilib.SmartDashboard.putNumber('_navx_yaw', self.get_yaw())
            wpilib.SmartDashboard.putNumber('_navx_angle', self.get_angle())

            wpilib.SmartDashboard.putNumber('keep_angle', self.keep_angle)
                # wpilib.SmartDashboard.putNumber('keep_angle_output', output)

            # post yaw, pitch, roll so we can see what is going on with the climb
            ypr = [self.navx.getYaw(), self.get_pitch(), self.navx.getRoll(), self.navx.getRotation2d().degrees()]
            wpilib.SmartDashboard.putNumberArray('_navx_YPR', ypr)

            # monitor power as well
            if True: # wpilib.RobotBase.isReal():
                # there's some kind of voltage simulation but idk if this covers it
                voltage = self.pdh.getVoltage()
                total_current = self.pdh.getTotalCurrent()
            else:
                # make up a current based on how fast we're going
                total_current = 2 + 10 * sum([math.fabs(module.drivingEncoder.getVelocity()) for module in self.swerve_modules])
                voltage = 12.5 - 0.02 * total_current

            wpilib.SmartDashboard.putNumber('_pdh_voltage', voltage)
            wpilib.SmartDashboard.putNumber('_pdh_current', total_current)



            if constants.k_swerve_debugging_messages:  # this is just a bit much unless debugging the swerve
                angles = [m.turningEncoder.getPosition() for m in self.swerve_modules]
                absolutes = [m.get_turn_encoder() for m in self.swerve_modules]
                wpilib.SmartDashboard.putNumberArray(f'_angles', angles)
                wpilib.SmartDashboard.putNumberArray(f'_analog_radians', absolutes)

