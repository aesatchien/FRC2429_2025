import math
import wpilib
import typing

import navx
import ntcore

from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance
import rev
from rev import SparkMax, SparkFlex

from wpilib.drive import DifferentialDrive

import constants
from constants import DriveConstants


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
            controller_revlib_error_source = self.motor.configure(DriveConstants.k_config_r1, self.rev_resets,
                                                                  self.rev_persists)
            controller_revlib_error_follower = self.follower.configure(DriveConstants.k_follower_config,
                                                                       self.rev_resets, self.rev_persists)
            print(
                f"Reconfigured elevator sparkmaxes. Controller status: \n {controller_revlib_error_source}\n {controller_revlib_error_follower}")

        self.drive = wpilib.drive.DifferentialDrive(self.drive_l1, self.drive_r1)

        # class variables
        self.counter = 0

        # networktables
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("datatable")

    def tank_drive(self, leftSpeed, rightSpeed):
        self.drive.tankDrive(leftSpeed, rightSpeed)

    def arcade_drive(self, xSpeed, zRotation):
        self.drive.arcadeDrive(xSpeed, zRotation, True)

    def periodic(self):
        # here we do all the checking of the robot state - read inputs, calculate outputs
        self.counter += 1

        if self.counter % 50 == 0:
            # update the SmartDashboard
            wpilib.SmartDashboard.putNumber('counter', self.counter)