import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters, radiansToDegrees, degreesToRadians
import math
from time import sleep

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

        # CJH looking to see if we can make these guys more robust - three things to try
        # Use the setPeriodicFrameTimeout() function in REVLib to configure the CAN timeout for periodic status frames.
        # self.motor.setPeriodicFrameTimeout(timeoutMs=500)  # The default timeout is 500ms.
        # Use the setCANMaxRetries() function to configure the number of retries for CAN frames.
        # self.motor.setCANMaxRetries(numRetries=5)  # the default is 5
        # self.motor.setCANTimeout(milliseconds=) # not sure the default, but should go up to 32k

        self.rev_resets = rev.SparkFlex.ResetMode.kResetSafeParameters
        self.rev_persists = rev.SparkFlex.PersistMode.kPersistParameters

        # this should be its own function later - we will call it whenever we change brake mode
        if constants.k_burn_flash:
            controller_revlib_error_source = self.motor.configure(constants.ShoulderConstants.k_config, self.rev_resets, self.rev_persists)
            controller_revlib_error_follower = self.follower.configure(constants.ShoulderConstants.k_follower_config, self.rev_resets, self.rev_persists)
            print(f"Reconfigured pivot sparkmaxes. Controller status: \n {controller_revlib_error_source}\n {controller_revlib_error_follower}")

        # configure our PID controller
        self.controller = self.motor.getClosedLoopController()
        # does this still work?
        # self.controller.setP(self.config['k_kP'])  # P is pretty much all we need in the controller!
        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(constants.ShoulderConstants.k_starting_angle)

        if constants.ShoulderConstants.k_nt_debugging:  # extra debugging info for NT
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

        self.abs_encoder = self.motor.getAbsoluteEncoder()
        abs_raw = self.abs_encoder.getPosition()
        abs_raws = []
        for reading in range(20):
            abs_raws.append(self.abs_encoder.getPosition())
            sleep(0.02)

        abs_raws_trunc = abs_raws[10:]
        abs_raw = sum(abs_raws_trunc) / len(abs_raws_trunc)
        abs_offset = -(abs_raw - constants.ShoulderConstants.k_abs_encoder_readout_when_at_ninety_deg_position)
        abs_offset_rad = abs_offset * math.tau
        abs_offset_rad += math.pi / 2
        msg = f'Pivot raw abs enc: {abs_raws[:10]}  Final: {abs_raws_trunc}\n'
        msg += f'Pivot absolute offset: {abs_offset:.3f} ({abs_offset_rad:.3f} rad or {math.degrees(abs_offset_rad):.1f} degrees) '
        print(msg)

        self.encoder.setPosition(abs_offset_rad)

        self.enable()
        # self.disable()

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        # print("SETPOINT POSITION: " + str(math.degrees(setpoint.position)))
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)  #

        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.SparkFlex.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0, arbFeedforward=feedforward)
        if constants.ShoulderConstants.k_nt_debugging:  # extra debugging info for NT
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

        # do not make the changes permanent, only change temporary brake mode
        rev_resets = rev.SparkMax.ResetMode.kNoResetSafeParameters
        rev_persists = rev.SparkMax.PersistMode.kNoPersistParameters
        self.motor.configure(constants.ShoulderConstants.k_config, rev_resets, rev_persists)
        self.follower.configure(constants.ShoulderConstants.k_follower_config, rev_resets, rev_persists)

    def get_angle(self):
        return self.encoder.getPosition()

    def set_goal(self, goal, use_trapezoid=True):
        # make our own sanity-check on the subsystem's setGoal function
        # this is not what is hanging L1
        if goal < constants.ShoulderConstants.k_min_angle:
            self.goal = constants.ShoulderConstants.k_min_angle
            print(f'Pivot goal too low: {goal:.3f} -> set to {self.goal}')
        elif goal > constants.ShoulderConstants.k_max_angle:
            self.goal = constants.ShoulderConstants.k_max_angle
            print(f'Pivot goal too high: {goal:.3f} -> set to {self.goal}')
        else:
            self.goal = goal

        #goal = goal if goal < constants.ShoulderConstants.k_max_angle else constants.ShoulderConstants.k_max_angle
        #goal = goal if goal > constants.ShoulderConstants.k_min_angle else constants.ShoulderConstants.k_min_angle
        #self.goal = goal

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

    def reflash(self, burn: bool, reset: bool):
        rev_persists = rev.SparkMax.PersistMode.kPersistParameters if burn else rev.SparkMax.PersistMode.kNoPersistParameters
        rev_resets = rev.SparkMax.ResetMode.kResetSafeParameters if reset else rev.SparkMax.ResetMode.kNoResetSafeParameters
        self.motor.configure(constants.ShoulderConstants.k_config, rev_resets, rev_persists)
        self.follower.configure(constants.ShoulderConstants.k_follower_config, rev_resets, rev_persists)

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
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_abs_encoder_readout', self.abs_encoder.getPosition())
