import math
import rev
import wpilib
import wpimath
from commands2 import Subsystem
from wpilib import SmartDashboard
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig, SparkMaxSim, SparkMax
from wpimath.units import inchesToMeters, radiansToDegrees, degreesToRadians
import constants

class Climber(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Climber')
        self.counter = 3
        self.sparkmax = rev.SparkMax(constants.ClimberConstants.k_CAN_id, rev.SparkMax.MotorType.kBrushless)
        self.sparkmax.configure(config=constants.ClimberConstants.k_config,
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)
        #configure PID controller
        self.controller = self.sparkmax.getClosedLoopController()
        self.config.softLimit.forwardSoftLimit(constants.ClimberConstants.k_climber_rotation_limit)
        self.config.softLimit.reverseSoftLimit(constants.ClimberConstants.k_climber_reverse_rotation_limit)
        self.climber_abs_encoder.setInverted(True)
        # chassis orientation
        self.swerve = Swerve()
        self.z_rotation = self.swerve.get_angle()
        self.x_rotation = self.swerve.get_roll()
        self.z_displacement = self.swerve.get_pose()
        self.z_minimum = 4
        self.y_rotation = self.swerve.get_yaw()
        # PID controller (Constants will be edited later)
        self.pid_controller = wpimath.controller.PIDController(Kp=6, Ki=0, Kd=0)
        self.output = self.pid_controller.calculate(self.z_displacement, self.z_minimum)
    # 3 Climber Positions:
    # Stowed, Ready, Climb
    # Use IMU to measure the tilt of the robot to ensure it is off the ground during climbing.
    def rotate_motor_arms(self):
        if self.is_ready() or self.climb():
            self.sparkmax.setReference(0, SparkMax.ControlType)
    def get_angle(self):
        return self.encoder.getPosition()
    def is_stowed(self):
        return math.fabs(self.get_angle() - constants.ClimberConstants.k_climber_motor_stowed_angle) < self.tolerance
    def is_ready(self):
        return self.climber_position == constants.ClimberConstants.k_climber_motor_rotation

    def climb(self):
        return (self.climber_position == constants.ClimberConstants.k_climber_motor_rotation
                + constants.ClimberConstants.k_climber_motor_climber_reference_angle)
    def is_hanging(self):
        return (self.output <= 0) and abs(self.x_rotation) > 0 and abs(self.y_rotation) > 0


