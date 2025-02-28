import rev
import constants
from commands2 import Subsystem
import wpilib
from rev import SparkMax, SoftLimitConfig, SparkMaxConfig, SoftLimitConfig
from robot.subsystems.swerve import Swerve
import wpimath.controller



class Climber(Subsystem):
    # Description of the climber:
    # 1.
    def __init__(self, container):
        self.setName("Climber")
        self.sparkmax = SparkMax(constants.ClimberConstants.k_CAN_id, rev.SparkMax.MotorType.kBrushless)
        self.config = SparkMaxConfig()
        self.sparkmax.configure(config=constants.ClimberConstants.k_CAN_id,
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)
        self.climber_abs_encoder = rev.AbsoluteEncoder()
        self.climber_position = self.climber_abs_encoder.getPosition()
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
        else:
            self.sparkmax.setReference(constants.ClimberConstants.k_climber_motor_voltage, SparkMax.ControlType)

    def is_stowed(self):
        return self.climber_position == constants.ClimberConstants.k_climber_motor_stowed_angle

    def is_ready(self):
        return self.climber_position == constants.ClimberConstants.k_climber_motor_rotation

    def climb(self):
        return (self.climber_position == constants.ClimberConstants.k_climber_motor_rotation
                + constants.ClimberConstants.k_climber_motor_climber_reference_angle)
    def is_hanging(self):
        return (self.output <= 0) and abs(self.x_rotation) > 0 and abs(self.y_rotation) > 0
















