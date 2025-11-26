import wpilib
import navx
import ntcore
from commands2 import Subsystem
from wpilib import SmartDashboard, DriverStation
from ntcore import NetworkTableInstance
import rev
from rev import SparkBase, SparkMaxConfig  # trying to save some typing
from wpilib.drive import DifferentialDrive

import constants
from constants import DriveConstants as dc


class Drivetrain(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.setName('Drivetrain')
        self.counter = dc.k_counter_offset  # note this should be an offset in constants

        # --------------- add motors and drive methods ----------------

        # motors - this should be cleaner - seems like we are retyping code
        motor_type = rev.SparkMax.MotorType.kBrushless
        self.drive_l1 = rev.SparkMax(dc.k_CANID_l1, motor_type)
        self.drive_l2 = rev.SparkMax(dc.k_CANID_l2, motor_type)
        self.drive_r1 = rev.SparkMax(dc.k_CANID_r1, motor_type)
        self.drive_r2 = rev.SparkMax(dc.k_CANID_r2, motor_type)

        # convenient list of motors if we need to query or set all of them
        self.motors = [self.drive_r1, self.drive_r2, self.drive_l1, self.drive_l2]

        # default parameters for the sparkmaxes reset and persist modes -
        self.rev_resets = SparkBase.ResetMode.kResetSafeParameters
        self.rev_persists = SparkBase.PersistMode.kPersistParameters if constants.k_burn_flash \
            else SparkBase.PersistMode.kNoPersistParameters

        # put the configs in a list matching the motors list
        self.configs = [dc.k_right_config, dc.k_follower_config_r2,
                   dc.k_left_config, dc.k_follower_config_l2]

        # this should be its own function later - we will call it whenever we change brake mode
        rev_errors = [motor.configure(config, self.rev_resets, self.rev_persists)
                      for motor, config in zip(self.motors, self.configs)]
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

    #mode in 'brake' by default
    def set_brake_mode(self, mode='brake'):
        # Non-persistent - just change  things temporarily - these settings leave the config untouched
        no_resets = SparkBase.ResetMode.kNoResetSafeParameters
        no_persists = SparkBase.PersistMode.kNoPersistParameters

        # make a temporary config just to set break or coast
        if mode == 'coast':
            idle_mode = SparkMaxConfig.IdleMode.kCoast
        else:
            idle_mode = SparkMaxConfig.IdleMode.kBrake
        tmp_config = SparkMaxConfig().setIdleMode(idle_mode)

        # send config to motors
        rev_errors = [motor.configure(tmp_config, no_resets, no_persists)
                      for motor in self.motors]
        # report our results - but not the best way since there is no timestamp here
        print(f'Setting drivetrain idle mode to {mode}: {rev_errors}')

    def periodic(self):
        # here we do all the checking of the robot state - read inputs, calculate outputs
        self.counter += 1

        if self.counter % 50 == 0:
            # update the SmartDashboard
            wpilib.SmartDashboard.putNumber('counter', self.counter)
