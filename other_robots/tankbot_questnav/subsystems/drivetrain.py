import math
import wpilib
import typing

import navx
import ntcore
import commands2

from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation, Field2d
from ntcore import NetworkTableInstance
import rev
from rev import SparkMax, SparkFlex

from wpilib.drive import DifferentialDrive

import constants
from constants import DriveConstants

from helpers.questnav.questnav2 import QuestNav
from wpimath.geometry import Rotation2d, Transform2d, Pose2d
from wpimath.units import inchesToMeters


class Drivetrain(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Drivetrain')
        self.counter = 0

        # add motors and drive methods

        # motors
        motor_type = rev.SparkMax.MotorType.kBrushless
        self.drive_l1 = rev.SparkMax(1, motor_type)
        self.drive_l2 = rev.SparkMax(2, motor_type)
        self.drive_r1 = rev.SparkMax(3, motor_type)
        self.drive_r2 = rev.SparkMax(4, motor_type)

        self.motors = [self.drive_r1, self.drive_r2, self.drive_l1, self.drive_l2]
        """
        self.encoder_l = self.drive_l1.getEncoder()  # c.f. wpilib.Encoder(2, 3)
        self.encoder_r = self.drive_r1.getEncoder()

        self.encoders = [self.encoder_l, self.encoder_r]

        [encoder.setPositionCoversionFactor(constants.kDrivingEncoderPositionFactor) for encoder in self.encoders]
        [encoder.setVelocityCoversionFactor(constants.kDrivingEncoderVelocityFactor) for encoder in self.encoders]
        
        inversions = [False, False, True, True] # order matters!!! matches to self.motors above
        [drive.setInverted(inversion) for drive, inversion in zip(self.motors, inversions)]

        # this is what you would do if you didn't use the zip above
        self.drive_l1.setInverted(True)  # need to see if + pwr moves motor clockwise and therefore robot fwd
        self.drive_l2.setInverted(True)
        self.drive_r1.setInverted(False)  # need to see if + pwr moves motor clockwise and therefore robot fwd
        self.drive_r2.setInverted(False)

        self.drive_l2.follow(self.drive_l1)  # sets 2 to follow 1
        self.drive_r2.follow(self.drive_r1)
        """

        self.rev_resets = rev.SparkMax.ResetMode.kResetSafeParameters
        self.rev_persists = rev.SparkMax.PersistMode.kPersistParameters

        # this should be its own function later - we will call it whenever we change brake mode
        if constants.k_burn_flash:
            controller_revlib_error_source_r = self.drive_r1.configure(DriveConstants.k_right_config, self.rev_resets,
                                                                  self.rev_persists)
            controller_revlib_error_follower_r = self.drive_r2.configure(DriveConstants.k_follower_config_r2,
                                                                       self.rev_resets, self.rev_persists)
            controller_revlib_error_source_l = self.drive_l1.configure(DriveConstants.k_left_config, self.rev_resets,
                                                                  self.rev_persists)
            controller_revlib_error_follower_l = self.drive_l2.configure(DriveConstants.k_follower_config_l2,
                                                                       self.rev_resets, self.rev_persists)
            print(
                f"Reconfigured elevator sparkmaxes. Controller status: \n {controller_revlib_error_source_r}\n {controller_revlib_error_follower_r}\n {controller_revlib_error_source_l}\n {controller_revlib_error_follower_l}")

        self.drive = wpilib.drive.DifferentialDrive(self.drive_l1, self.drive_r1)

        # class variables
        self.counter = 0

        # networktables
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("datatable")

        #QuestNav
        self.questnav = QuestNav()
        self.quest_to_robot = Transform2d(inchesToMeters(-12.5-0.55), 0, Rotation2d().fromDegrees(0))
        # self.quest_to_robot = Transform2d(inchesToMeters(4), 0, Rotation2d().fromDegrees(0))
        self.quest_field = Field2d()
        wpilib.SmartDashboard.putData('Reset Quest Odometry', commands2.InstantCommand(lambda: self.reset_quest_odometry()))

    def tank_drive(self, leftSpeed, rightSpeed):
        self.drive.tankDrive(leftSpeed, rightSpeed)

    def arcade_drive(self, xSpeed, zRotation, square_inputs):
        self.drive.arcadeDrive(xSpeed, zRotation, square_inputs)

    def periodic(self):
        # here we do all the checking of the robot state - read inputs, calculate outputs
        self.counter += 1

        if self.counter % 50 == 0:
            # update the SmartDashboard
            wpilib.SmartDashboard.putNumber('counter', self.counter)
        
        # Import pose from QuestNav.
        self.quest_periodic()
    
    def quest_periodic(self) -> None:
        self.questnav.command_periodic()
        SmartDashboard.putBoolean("QUEST_CONNECTED", self.questnav.is_connected())
        SmartDashboard.putBoolean("QUEST_TRACKING", self.questnav.is_tracking())
        quest_pose = self.questnav.get_pose().transformBy(self.quest_to_robot)

        SmartDashboard.putString("QUEST_POSE", str(quest_pose))
        self.quest_field.setRobotPose(quest_pose)
        SmartDashboard.putData("QUEST_FIELD", self.quest_field)
        if 0 < quest_pose.x < 17.658 and 0 < quest_pose.y < 8.131 and self.questnav.is_connected():
            SmartDashboard.putBoolean("QUEST_POSE_ACCEPTED", True)
            # print("Quest Timestamp: " + str(self.questnav.get_app_timestamp()))
            # print("System Timestamp: " + str(utils.get_system_time_seconds()))
            # if abs(self.questnav.get_data_timestamp() - utils.get_current_time_seconds()) < 5:
            #     print("Timestamp in correct epoch.")
            #self.add_vision_measurement(quest_pose,
            #                            utils.fpga_to_current_time(self.questnav.get_data_timestamp()),
            #                            (0.02, 0.02, 0.035))
        else:
            SmartDashboard.putBoolean("QUEST_POSE_ACCEPTED", False)

    def reset_pose_with_quest(self, pose: Pose2d) -> None:
        #self.reset_pose(pose)
        self.questnav.set_pose(pose.transformBy(self.quest_to_robot.inverse()))

    def reset_quest_odometry(self) -> None:
        """Reset robot odometry at the Subwoofer."""
        if DriverStation.getAlliance() == DriverStation.Alliance.kRed:
            #self.reset_pose(Pose2d(14.337, 4.020, Rotation2d.fromDegrees(0)))
            #self.set_operator_perspective_forward(Rotation2d.fromDegrees(180))
            self.questnav.set_pose(Pose2d(14.337, 4.020, Rotation2d.fromDegrees(0)).transformBy(self.quest_to_robot.inverse()))
        else:
            #self.reset_pose(Pose2d(3.273, 4.020, Rotation2d.fromDegrees(180)))
            #self.set_operator_perspective_forward(Rotation2d.fromDegrees(0))
            self.questnav.set_pose(Pose2d(3.273, 4.020, Rotation2d.fromDegrees(180)).transformBy(self.quest_to_robot.inverse()))
