import wpilib
import navx
import ntcore
from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance
import rev
from rev import SparkBase  # trying to save some typing
from wpilib.drive import DifferentialDrive

import constants
from constants import DriveConstants


class Drivetrain(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Drivetrain')
        self.counter = DriveConstants.k_counter_offset  # note this should be an offset in constants

        # --------------- add motors and drive methods ----------------

        # motors - this should be cleaner - seems like we are retyping code
        motor_type = rev.SparkMax.MotorType.kBrushless
        self.drive_l1 = rev.SparkMax(DriveConstants.k_CANID_l1, motor_type)
        self.drive_l2 = rev.SparkMax(DriveConstants.k_CANID_l2, motor_type)
        self.drive_r1 = rev.SparkMax(DriveConstants.k_CANID_r1, motor_type)
        self.drive_r2 = rev.SparkMax(DriveConstants.k_CANID_r2, motor_type)

        # convenient list of motors if we need to query or set all of them
        self.motors = [self.drive_r1, self.drive_r2, self.drive_l1, self.drive_l2]

        # default parameters for the sparkmaxes reset and persist modes -
        self.rev_resets = SparkBase.ResetMode.kResetSafeParameters if constants.k_burn_flash \
            else SparkBase.ResetMode.kNoResetSafeParameters
        self.rev_persists = SparkBase.PersistMode.kPersistParameters if constants.k_burn_flash \
            else SparkBase.ResetMode.kNoResetSafeParameters

        # put the configs in a list matching the motors list
        configs = [DriveConstants.k_right_config, DriveConstants.k_follower_config_r2,
                   DriveConstants.k_left_config, DriveConstants.k_follower_config_l2]

        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, configs)]
        print(
            f"Initialized drivetrain sparkmaxes. Controller status: \n {rev_errors}")

        # set up a high-level drive object so we can use its drive methods
        self.drive = wpilib.drive.DifferentialDrive(self.drive_l1, self.drive_r1)

        # networktables
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable("datatable")  # for example

    # give access to wpilib's tankdrive and arcadedrive methods
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
