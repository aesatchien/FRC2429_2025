from commands2.subsystem import Subsystem
import wpilib
from rev import ClosedLoopSlot, SparkMax
from constants import ShoulderConstants, ShoulderConstants

class Shoulder(Subsystem):

    def __init__(self):

        self.sparkmax = SparkMax(ShoulderConstants.k_CAN_id, SparkMax.MotorType.kBrushless)

        self.sparkmax.configure(config=ShoulderConstants.k_config, 
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)

        self.follower_sparkmax = SparkMax(ShoulderConstants.k_follower_CAN_id, SparkMax.MotorType.kBrushless)

        self.follower_sparkmax.configure(config=ShoulderConstants.k_follower_config,
                                         resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                         persistMode=SparkMax.PersistMode.kPersistParameters)

        self.encoder = self.sparkmax.getEncoder()
        self.abs_encoder = self.sparkmax.getAbsoluteEncoder()
        self.abs_encoder

        if wpilib.RobotBase.isReal():
            self.encoder.setPosition(self.abs_encoder.getPosition()) # may have to set offset here if the zeroOffset kParamInvalid error isn't fixed
                                                                     # if so put the offset here into shoulderconstants

        else:
            self.encoder.setPosition(ShoulderConstants.k_sim_starting_angle)

        self.controller = self.sparkmax.getClosedLoopController()

        self.setpoint = self.encoder.getPosition()

        self.counter = 0

    def set_position(self, radians: float) -> None:
        self.setpoint = radians
        self.controller.setReference(value=self.setpoint, ctrl=SparkMax.ControlType.kPosition, slot=ClosedLoopSlot(0))

    def set_encoder_position(self, radians: float):
        self.encoder.setPosition(radians)

    def get_angle(self) -> float:
        return self.encoder.getPosition()
        # return self.abs_encoder.getPosition()

    def get_at_setpoint(self) -> bool:
        return abs(self.encoder.getPosition() - self.setpoint) < ShoulderConstants.k_tolerance

    def periodic(self) -> None:

        self.counter += 1

        return super().periodic()
