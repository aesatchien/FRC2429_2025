import rev
import constants
from commands2 import Subsystem
import wpilib
from rev import SparkMax, SoftLimitConfig, SparkMaxConfig


class Climber(Subsystem):
    # Description of the climber:
    # 1.
    def __init__(self):
        self.setName("Climber")
        self.sparkmax = SparkMax(constants.ClimberConstants.k_CAN_id, rev.SparkMax.MotorType.kBrushless)
        self.config = SparkMaxConfig()
        self.sparkmax.configure(config=constants.ClimberConstants.k_CAN_id,
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)
        self.climber_abs_encoder = rev.AbsoluteEncoder()
        self.climber_position = self.climber_abs_encoder.getPosition()
        self.controller = self.sparkmax.getClosedLoopController()


    # 3 Climber Positions:
    # Stowed, Ready, Climb
    def rotate_motor_arms(self):
        if self.is_stowed():
            self.sparkmax.setReference(constants.ClimberConstants.k_climber_motor_voltage, SparkMax.ControlType)

    def is_stowed(self):
        return self.climber_position == constants.ClimberConstants.k_climber_motor_rotation

    def 

    #def
     #   pass










