import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters
import math

import constants
from constants import ElevatorConstants


class Elevator(commands2.TrapezoidProfileSubsystem):

    def __init__(self) -> None:
        super().__init__(
            constraints=wpimath.trajectory.TrapezoidProfile.Constraints(
                ElevatorConstants.k_max_velocity_meter_per_second,
                ElevatorConstants.k_max_acceleration_meter_per_sec_squared
            ),
            initial_position=ElevatorConstants.k_min_height,
            period=0.02,
        )

        # used to calculate the voltage needed for a given velocity
        # note that the velocity of this half the actual velocity of our carriage because it is cascade
        # i'm not sure where we need to trick it - probably send it the cascade statge velocity / 2
        self.feedforward = wpimath.controller.ElevatorFeedforward(
            kS=ElevatorConstants.k_kS_volts,
            kG=ElevatorConstants.k_kG_volts,
            kV=ElevatorConstants.k_kV_volt_second_per_radian,
            kA=ElevatorConstants.k_kA_volt_second_squared_per_meter,
            dt=0.02)

# ------------   2429 Additions to the template's __init__  ------------
        self.setName(ElevatorConstants.k_name)
        self.counter = ElevatorConstants.k_counter_offset
        self.is_moving = False  # may want to keep track of if we are in motion
        self.tolerance = 0.03  # meters - then we will be "at goal"
        self.goal = ElevatorConstants.k_min_height
        self.at_goal = True

        # initialize the motors and keep a list of them for configuration later
        self.motor = rev.SparkMax(ElevatorConstants.k_CAN_id, rev.SparkMax.MotorType.kBrushless)
        self.follower = rev.SparkMax(ElevatorConstants.k_follower_CAN_id, rev.SparkMax.MotorType.kBrushless)
        self.sparks = [self.motor, self.follower]

        self.rev_resets = rev.SparkMax.ResetMode.kResetSafeParameters
        self.rev_persists = rev.SparkMax.PersistMode.kPersistParameters

        # this should be its own function later - we will call it whenever we change brake mode
        if constants.k_burn_flash:
            controller_revlib_error_source = self.motor.configure(ElevatorConstants.k_config, self.rev_resets, self.rev_persists)
            controller_revlib_error_follower = self.follower.configure(ElevatorConstants.k_follower_config, self.rev_resets, self.rev_persists)
            print(f"Reconfigured elevator sparkmaxes. Controller status: \n {controller_revlib_error_source}\n {controller_revlib_error_follower}")

        # configure our PID controller
        self.controller = self.motor.getClosedLoopController()
        # does this still work?
        # self.controller.setP(self.config['k_kP'])  # P is pretty much all we need in the controller!
        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(self.goal)

        self.enable()

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        # print("SETPOINT POSITION: " + str(math.degrees(setpoint.position)))
        feedforward = self.feedforward.calculate(setpoint.velocity/2)  # the 2 corrects for the 2x carriage speed

        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0, arbFeedforward=feedforward)
        # self.goal = setpoint.position  # don't want this - unless we want to plot the trapezoid

    def set_brake_mode(self, mode='brake'):
        if mode == 'brake':
            ElevatorConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            ElevatorConstants.k_follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        else:
            ElevatorConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
            ElevatorConstants.k_follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        # do not make the changes permanent
        rev_resets = rev.SparkMax.ResetMode.kNoResetSafeParameters
        rev_persists = rev.SparkMax.PersistMode.kNoPersistParameters
        self.motor.configure(ElevatorConstants.k_config, rev_resets, rev_persists)
        self.follower.configure(ElevatorConstants.k_follower_config, rev_resets, rev_persists)

    def get_height(self):
        return self.encoder.getPosition()

    def set_goal(self, goal):
        # make our own sanity-check on the subsystem's setGoal function
        goal = goal if goal < ElevatorConstants.k_max_height else ElevatorConstants.k_max_height
        goal = goal if goal > ElevatorConstants.k_min_height else ElevatorConstants.k_min_height
        self.goal = goal
        # print(f'setting goal to {self.goal}')
        self.setGoal(self.goal)
        self.at_goal = False

    def move_meters(self, delta_meters: float, silent=False) -> None:  # way to bump up and down for testing
        current_position = self.get_height()
        goal = current_position + delta_meters
        self.set_goal(goal)  # check and set
        if not silent:
            message = f'setting {self.getName()} from {current_position:.2f} to {self.goal:.2f}'
            print(message)

    def get_at_goal(self):
        return self.at_goal

    def periodic(self) -> None:
        # What if we didn't call the below for a few cycles after we set the position?
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 10 == 0:
            self.position = self.encoder.getPosition()
            self.at_goal = math.fabs(self.position - self.goal) < self.tolerance  # maybe we want to call this an error
            self.error = self.position - self.goal

            if ElevatorConstants.k_nt_debugging:  # add additional info to NT for debugging
                wpilib.SmartDashboard.putBoolean(f'{self.getName()}_at_goal', self.at_goal)
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_error', self.error)
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_goal', self.goal)
                # wpilib.SmartDashboard.putNumber(f'{self.getName()}_curr_sp',) not sure how to ask for this - controller won't give it
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_output', self.motor.getAppliedOutput())
            self.is_moving = abs(self.encoder.getVelocity()) > 0.001  # m per second
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_moving', self.is_moving)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_spark_pos', self.position * 1000)  #  make it mm