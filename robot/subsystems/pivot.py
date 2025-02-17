import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters, radiansToDegrees, degreesToRadians
import math


class Pivot(commands2.TrapezoidProfileSubsystem):

    config = {
        'name': 'profiled_pivot',
        'max_angle': degreesToRadians(225),
        'min_angle': degreesToRadians(-45),
        'motor_can_id': 6, 'follower_can_id': 7,
        'gear_ratio': 100,
        'initial_theta': math.pi / 2,  # straight up
        'k_MaxVelocityRadPerSecond': 1.5,  # max possible speed of gearing is about 2 m/s
        'k_MaxAccelerationRadPerSecSquared': 2.5,
        'k_kP': 0.75,  # TODO - test this

        # Pivot stuff - from recalc
        'k_kSVolts': 0.0,  # constant to always add, uses the sign of velocity
        'k_kGVolts': 1.4/2.0,  # 12kg at .2m COM, cuts in half with two motors, goes up with mass and distance, down with efficiency
        'k_kVVoltSecondPerMeter': 1.69,  # stays the same with one or two motors, based on the NEO itself and gear ratio
        'k_kAVoltSecondSquaredPerMeter': 0.04 / 2.0,  # cuts in half with two motors
    }

    def __init__(self) -> None:
        super().__init__(
            constraints=wpimath.trajectory.TrapezoidProfileRadians.Constraints(
                self.config['k_MaxVelocityRadPerSecond'],
                self.config['k_MaxAccelerationRadPerSecSquared'],
            ),
            initial_position= self.config['initial_theta'],  # straight up
            period=0.02,
        )

        # used to calculate the voltage needed for a given velocity
        # note that the velocity of this half the actual velocity of our carriage because it is cascade
        # i'm not sure where we need to trick it - probably send it the cascade statge velocity / 2
        self.feedforward = wpimath.controller.ArmFeedforward(
            kS=self.config['k_kSVolts'],
            kG=self.config['k_kGVolts'],
            kV=self.config['k_kVVoltSecondPerMeter'],
            kA=self.config['k_kAVoltSecondSquaredPerMeter'],
            dt=0.02)

# ------------   2429 Additions to the template's __init__  ------------
        self.setName(self.config['name'])
        self.counter = 5
        self.is_moving = False  # may want to keep track of if we are in motion
        self.tolerance = 0.034  # rads equal to two degrees
        self.setpoint = self.config['initial_theta']  # current setpoint
        self.goal = self.config['initial_theta']  # current goal
        self.at_goal = True

        self.configure_motors()
        self.enable()
        # self.disable()

    def configure_motors(self):
        # too much to put in init
        #  *** LEADER CONFIG ***
        self.motor_config = rev.SparkFlexConfig()
        self.motor_config.inverted(False)  # basically defines which side is zero
        # we need it separate for the sim
        k_rads_per_revolution = 2 * math.pi / self.config['gear_ratio']  # elevator goes 2x the chain
        self.motor_config.encoder.positionConversionFactor(k_rads_per_revolution)  # meters
        self.motor_config.encoder.velocityConversionFactor(k_rads_per_revolution / 60)  # meters per second

        self.motor_config.closedLoop.pidf(p=self.config['k_kP'], i=0, d=0, ff=0, slot=rev.ClosedLoopSlot(0))
        self.motor_config.closedLoop.outputRange(-1, 1)

        self.motor_config.softLimit.forwardSoftLimit(self.config['max_angle'])
        self.motor_config.softLimit.reverseSoftLimit(self.config['min_angle'])
        self.motor_config.softLimit.forwardSoftLimitEnabled(True)
        self.motor_config.softLimit.reverseSoftLimitEnabled(True)
        self.motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.motor_config.smartCurrentLimit(40)

        #  *** FOLLOWER CONFIG ***
        self.follower_config = rev.SparkFlexConfig()
        self.follower_config.follow(self.config['motor_can_id'], invert=False)
        self.follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        # initialize the motors and keep a list of them for configuration later
        self.motor = rev.SparkFlex(self.config['motor_can_id'], rev.SparkBase.MotorType.kBrushless)
        self.follower = rev.SparkFlex(self.config['follower_can_id'], rev.SparkBase.MotorType.kBrushless)
        self.sparks = [self.motor, self.follower]

        self.rev_resets = rev.SparkFlex.ResetMode.kResetSafeParameters
        self.rev_persists = rev.SparkFlex.PersistMode.kPersistParameters

        # this should be its own function later - we will call it whenever we change brake mode
        self.motor.configure(self.motor_config, self.rev_resets, self.rev_persists)
        self.follower.configure(self.follower_config, self.rev_resets, self.rev_persists)

        # configure our PID controller
        self.controller = self.motor.getClosedLoopController()
        # does this still work?
        # self.controller.setP(self.config['k_kP'])  # P is pretty much all we need in the controller!
        self.encoder = self.motor.getEncoder()
        self.encoder.setPosition(self.setpoint)

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        # print("SETPOINT POSITION: " + str(math.degrees(setpoint.position)))
        feedforward = self.feedforward.calculate(setpoint.position, setpoint.velocity)  #

        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.SparkFlex.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0, arbFeedforward=feedforward)
        # self.goal = setpoint.position  # don't want this - unless we want to plot the trapezoid

    def set_brake_mode(self, mode='brake'):
        if mode == 'brake':
            self.motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            self.follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        else:
            self.motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
            self.follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        self.motor.configure(self.motor_config, self.rev_resets, self.rev_persists)
        self.follower.configure(self.follower_config, self.rev_resets, self.rev_persists)

    def get_angle(self):
        return self.encoder.getPosition()

    def set_goal(self, goal):
        # make our own sanity-check on the subsystem's setGoal function
        goal = goal if goal < self.config['max_angle'] else self.config['max_angle']
        goal = goal if goal > self.config['min_angle'] else self.config['min_angle']
        self.goal = goal
        # print(f'setting goal to {self.goal}')
        self.setGoal(self.goal)
        self.at_goal = False

    def move_degrees(self, delta_degrees: float, silent=False) -> None:  # way to bump up and down for testing
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
        if self.counter % 5 == 0:
            self.angle = self.encoder.getPosition()
            self.at_goal = math.fabs(self.angle - self.goal) < self.tolerance  # maybe we want to call this an error
            self.error = self.angle - self.goal

            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_at_goal', self.at_goal)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_error', self.error)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_goal', self.goal)
            # wpilib.SmartDashboard.putNumber(f'{self.getName()}_curr_sp',) not sure how to ask for this - controller won't give it
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_spark_angle', self.angle)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_output', self.motor.getAppliedOutput())
            self.is_moving = abs(self.encoder.getVelocity()) > 0.001  # m per second
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_moving', self.is_moving)
