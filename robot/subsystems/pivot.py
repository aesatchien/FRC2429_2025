import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters, radiansToDegrees, degreesToRadians
import math

import constants


class Pivot(commands2.TrapezoidProfileSubsystem):

    def __init__(self) -> None:
        super().__init__(
            constraints=wpimath.trajectory.TrapezoidProfileRadians.Constraints(
                constants.ShoulderConstants.k_max_velocity_rad_per_second,
                constants.ShoulderConstants.k_max_acceleration_rad_per_sec_squared
            ),
            initial_position=constants.ShoulderConstants.k_starting_angle,  # straight up
            period=0.02,
        )

        # used to calculate the voltage needed for a given velocity
        # note that the velocity of this half the actual velocity of our carriage because it is cascade
        # i'm not sure where we need to trick it - probably send it the cascade statge velocity / 2
        self.feedforward = wpimath.controller.ArmFeedforward(
            kS=constants.ShoulderConstants.k_kS_volts,
            kG=constants.ShoulderConstants.k_kG_volts,
            kV=constants.ShoulderConstants.k_kV_volt_second_per_radian,
            kA=constants.ShoulderConstants.k_kA_volt_second_squared_per_meter,
            dt=0.02)

        # initialize the motors and keep a list of them for configuration later
        self.motor = rev.SparkFlex(constants.ShoulderConstants.k_CAN_id, rev.SparkBase.MotorType.kBrushless)
        self.follower = rev.SparkFlex(constants.ShoulderConstants.k_follower_CAN_id, rev.SparkBase.MotorType.kBrushless)
        self.sparks = [self.motor, self.follower]

        self.rev_resets = rev.SparkFlex.ResetMode.kResetSafeParameters
        self.rev_persists = rev.SparkFlex.PersistMode.kPersistParameters

        # this should be its own function later - we will call it whenever we change brake mode
        self.motor.configure(constants.ShoulderConstants.k_config, self.rev_resets, self.rev_persists)
        self.follower.configure(constants.ShoulderConstants.k_follower_config, self.rev_resets, self.rev_persists)

        # configure our PID controller
        self.controller = self.motor.getClosedLoopController()
        # does this still work?
        # self.controller.setP(self.config['k_kP'])  # P is pretty much all we need in the controller!
        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(constants.ShoulderConstants.k_starting_angle)

        wpilib.SmartDashboard.putNumber("profiled_pivot velocity setpoint", 0)
        wpilib.SmartDashboard.putNumber("profiled_pivot position setpoint", 0)
        wpilib.SmartDashboard.putNumber("profiled_pivot applied output", 0)
        wpilib.SmartDashboard.putNumber("profiled_pivot follower applied output", 0)

# ------------   2429 Additions to the template's __init__  ------------
        self.setName(constants.ShoulderConstants.k_name)
        self.counter = constants.ShoulderConstants.k_counter_offset
        self.is_moving = False  # may want to keep track of if we are in motion
        self.tolerance = 0.087  # rads equal to five degrees - then we will be "at goal"
        self.goal = constants.ShoulderConstants.k_starting_angle
        self.at_goal = True

        self.enable()
        # self.disable()

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        # print("SETPOINT POSITION: " + str(math.degrees(setpoint.position)))
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)  #

        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.SparkFlex.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0, arbFeedforward=feedforward)
        wpilib.SmartDashboard.putNumber("profiled_pivot velocity setpoint", setpoint.velocity)
        wpilib.SmartDashboard.putNumber("profiled_pivot position setpoint", setpoint.position)
        wpilib.SmartDashboard.putNumber("profiled_pivot applied output", self.motor.getAppliedOutput())
        wpilib.SmartDashboard.putNumber("profiled_pivot follower applied output", self.follower.getAppliedOutput())
        # self.goal = setpoint.position  # don't want this - unless we want to plot the trapezoid

    def set_brake_mode(self, mode='brake'):
        if mode == 'brake':
            constants.ShoulderConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            constants.ShoulderConstants.k_follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        else:
            constants.ShoulderConstants.k_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
            constants.ShoulderConstants.k_follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        self.motor.configure(constants.ShoulderConstants.k_config, self.rev_resets, self.rev_persists)
        self.follower.configure(constants.ShoulderConstants.k_follower_config, self.rev_resets, self.rev_persists)

    def get_angle(self):
        return self.encoder.getPosition()

    def set_goal(self, goal, use_trapezoid=True):
        # make our own sanity-check on the subsystem's setGoal function
        goal = goal if goal < constants.ShoulderConstants.k_max_angle else constants.ShoulderConstants.k_max_angle
        goal = goal if goal > constants.ShoulderConstants.k_min_angle else constants.ShoulderConstants.k_min_angle
        self.goal = goal
        # print(f'setting goal to {self.goal}')
        if use_trapezoid:
            self.enable()
            self.setGoal(self.goal)
        else:
            self.disable()
            self.controller.setReference(goal, rev.SparkMax.ControlType.kPosition, slot=rev.ClosedLoopSlot(2))

        self.at_goal = False

    def move_degrees(self, delta_degrees: float, silent=True) -> None:  # way to bump up and down for testing
        current_angle = self.get_angle()
        goal = current_angle + degreesToRadians(delta_degrees)
        self.set_goal(goal)  # check and set
        if not silent:
            message = f'setting {self.getName()} from {current_angle:.2f} to {self.goal:.2f}'
            print(message)

    def periodic(self) -> None:
        # What if we didn't call the below for a few cycles after we set the position?
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 10 == 0:
            self.angle = self.encoder.getPosition()
            self.at_goal = math.fabs(self.angle - self.goal) < self.tolerance  # maybe we want to call this an error
            self.error = self.angle - self.goal

            if constants.ShoulderConstants.k_nt_debugging:  # extra debugging info for NT
                wpilib.SmartDashboard.putBoolean(f'{self.getName()}_at_goal', self.at_goal)
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_error', self.error)
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_goal', self.goal)
                # wpilib.SmartDashboard.putNumber(f'{self.getName()}_curr_sp',) not sure how to ask for this - controller won't give it
                wpilib.SmartDashboard.putNumber(f'{self.getName()}_output', self.motor.getAppliedOutput())
            self.is_moving = abs(self.encoder.getVelocity()) > 0.001  # m per second
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_moving', self.is_moving)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_spark_angle', radiansToDegrees(self.angle))
