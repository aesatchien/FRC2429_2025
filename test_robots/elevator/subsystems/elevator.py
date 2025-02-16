import commands2
import wpimath.controller
import wpimath.trajectory
import rev
import wpilib
from wpimath.units import inchesToMeters
import math


class Elevator(commands2.TrapezoidProfileSubsystem):

    config = {
        'name': 'profiled_elevator',
        'max_height': inchesToMeters(64), 'min_height': inchesToMeters(8),
        'motor_can_id': 4, 'follower_can_id': 5,
        'gear_ratio': 15,

        'k_MaxVelocityMeterPerSecond': 1.0,  #
        'k_MaxAccelerationMeterPerSecSquared': 2.0,
        'k_kP': 0.00,  #TODO - update this

        # Elevator stuff - from recalc
        'k_kSVolts': 0.0,  # constant to always add - seems redundant with kG?
        'k_kGVolts': 0.88/2.0,  # 16kg, cuts in half with two motors, goes up with mass and distance, down with efficiency
        'k_kVVoltSecondPerMeter': 12.05,  # stays the same with one or two motors, based on the NEO itself and gear ratio
        'k_kAVoltSecondSquaredPerMeter': 0.10 / 2.0,  # cuts in half with two motors
    }

    def __init__(self) -> None:
        super().__init__(
            constraints=wpimath.trajectory.TrapezoidProfile.Constraints(
                self.config['k_MaxVelocityMeterPerSecond'],
                self.config['k_MaxAccelerationMeterPerSecSquared'],
            ),
            initial_position=self.config['min_height'],  # bottom of elevator
            period=0.02,
        )

        # used to calculate the voltage needed for a given velocity
        # note that the velocity of this half the actual velocity of our carriage because it is cascade
        # i'm not sure where we need to trick it - probably send it the cascade statge velocity / 2
        self.feedforward = wpimath.controller.ElevatorFeedforward(
            kS=self.config['k_kSVolts'],
            kG=self.config['k_kGVolts'],
            kV=self.config['k_kVVoltSecondPerMeter'],
            kA=self.config['k_kAVoltSecondSquaredPerMeter'],
            dt=0.02)

# ------------   2429 Additions to the template's __init__  ------------
        self.setName(self.config['name'])
        self.counter = 4
        self.is_moving = False  # may want to keep track of if we are in motion
        self.tolerance = 0.01  # meters
        self.setpoint = self.config['min_height']  # current setpoint
        self.goal = self.config['min_height']  # current goal
        self.at_goal = True
        self.enable()

        self.configure_motors()

    def configure_motors(self):
        # too much to put in init
        #  *** LEADER CONFIG ***
        self.motor_config = rev.SparkMaxConfig()
        self.motor_config.inverted(True)
        # we need it separate for the sim
        k_effective_pulley_diameter = inchesToMeters(1.91)
        k_meters_per_revolution = math.pi * 2 * k_effective_pulley_diameter / self.config['gear_ratio']  # elevator goes 2x the chain
        self.motor_config.encoder.positionConversionFactor(k_meters_per_revolution)  # meters
        self.motor_config.encoder.velocityConversionFactor(k_meters_per_revolution / 60)  # meters per second

        self.motor_config.closedLoop.pidf(p=self.config['k_kP'], i=0, d=0, ff=0, slot=rev.ClosedLoopSlot(0))
        self.motor_config.closedLoop.outputRange(-1, 1)

        self.motor_config.softLimit.forwardSoftLimit(self.config['max_height'])
        self.motor_config.softLimit.reverseSoftLimit(self.config['min_height'])
        self.motor_config.softLimit.forwardSoftLimitEnabled(True)
        self.motor_config.softLimit.reverseSoftLimitEnabled(True)
        self.motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        self.motor_config.smartCurrentLimit(40)

        #  *** FOLLOWER CONFIG ***
        self.follower_config = rev.SparkMaxConfig()
        self.follower_config.follow(self.config['motor_can_id'], invert=True)
        self.follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)

        # initialize the motors and keep a list of them for configuration later
        self.motor = rev.SparkMax(self.config['motor_can_id'], rev.SparkMax.MotorType.kBrushless)
        self.follower = rev.SparkMax(self.config['follower_can_id'], rev.SparkMax.MotorType.kBrushless)
        self.sparks = [self.motor, self.follower]

        self.rev_resets = rev.SparkMax.ResetMode.kResetSafeParameters
        self.rev_persists = rev.SparkMax.PersistMode.kPersistParameters

        # this should be its own function later - we will call it whenever we change brake mode
        self.motor.configure(self.motor_config, self.rev_resets, self.rev_persists)
        self.follower.configure(self.follower_config, self.rev_resets, self.rev_persists)

        # configure our PID controller
        self.controller = self.motor.getClosedLoopController()
        # does this still work?
        # self.controller.setP(self.config['k_kP'])  # P is pretty much all we need in the controller!
        self.encoder = self.motor.getEncoder()

    def useState(self, setpoint: wpimath.trajectory.TrapezoidProfile.State) -> None:
        # Calculate the feedforward from the setpoint
        # print("SETPOINT POSITION: " + str(math.degrees(setpoint.position)))
        feedforward = self.feedforward.calculate(setpoint.velocity/2)  # the 2 corrects for the 2x carriage speed

        # Add the feedforward to the PID output to get the motor output
        # TODO - check if the feedforward is correct in units for the sparkmax - documentation says 32, not 12
        self.controller.setReference(setpoint.position, rev.SparkMax.ControlType.kPosition, rev.ClosedLoopSlot.kSlot0, arbFeedforward=feedforward)
        # self.goal = setpoint.position  # don't want this - unless we want to plot the trapezoid

        if wpilib.RobotBase.isSimulation():
            self.encoder.setPosition(setpoint.position)

    def set_brake_mode(self, mode='brake'):
        if mode == 'brake':
            self.motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            self.follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
        else:
            self.motor_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)
            self.follower_config.setIdleMode(rev.SparkBaseConfig.IdleMode.kCoast)

        self.motor.configure(self.motor_config, self.rev_resets, self.rev_persists)
        self.follower.configure(self.follower_config, self.rev_resets, self.rev_persists)

    def get_height(self):
        return self.encoder.getPosition()

    def set_goal(self, goal):
        # make our own sanity-check on the subsystem's setGoal function
        goal = goal if goal < self.config['max_height'] else self.config['max_height']
        goal = goal if goal > self.config['min_height'] else self.config['min_height']
        self.goal = goal
        # print(f'setting goal to {self.goal}')
        self.setGoal(self.goal)

    def move_meters(self, delta_meters: float, silent=False) -> None:  # way to bump up and down for testing
        current_position = self.get_height()
        goal = current_position + delta_meters
        self.set_goal(goal)  # check and set
        if not silent:
            message = f'setting {self.getName()} from {current_position:.2f} to {self.goal:.2f}'
            print(message)

    def periodic(self) -> None:
        # What if we didn't call the below for a few cycles after we set the position?
        super().periodic()  # this does the automatic motion profiling in the background
        self.counter += 1
        if self.counter % 2 == 0:
            self.position = self.encoder.getPosition()
            self.at_goal = math.fabs(self.position - self.goal) < self.tolerance  # maybe we want to call this an error
            self.error = self.position - self.goal

            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_at_goal', self.at_goal)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_error', self.error)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_goal', self.goal)
            # wpilib.SmartDashboard.putNumber(f'{self.getName()}_curr_sp',) not sure how to ask for this - controller won't give it
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_spark_pos', self.position)
            wpilib.SmartDashboard.putNumber(f'{self.getName()}_output', self.motor.getAppliedOutput())
            self.is_moving = abs(self.encoder.getVelocity()) > 0.001  # m per second
            wpilib.SmartDashboard.putBoolean(f'{self.getName()}_is_moving', self.is_moving)
