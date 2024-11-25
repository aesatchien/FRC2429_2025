from commands2.subsystem import Subsystem
import wpilib
from sparksim import CANSparkMax as SimCANSparkMax
from rev import CANSparkMax
import constants
import math


class LowerCrank(Subsystem):

    def __init__(self, container):
        self.container = container

        if wpilib.RobotBase.isReal():
            self.sparkmax = CANSparkMax(constants.k_lower_crank_CAN_id, CANSparkMax.MotorType.kBrushless)
        else:
            self.sparkmax = SimCANSparkMax(constants.k_lower_crank_CAN_id, CANSparkMax.MotorType.kBrushless)

        self.sparkmax.restoreFactoryDefaults()

        self.encoder = self.sparkmax.getEncoder()

        # radians
        self.encoder.setPositionConversionFactor(math.tau / constants.k_lower_crank_dict["k_gear_ratio"])
        self.encoder.setVelocityConversionFactor(math.tau / constants.k_lower_crank_dict["k_gear_ratio"])

        self.abs_encoder = self.sparkmax.getAbsoluteEncoder()
        self.abs_encoder.setPositionConversionFactor(math.tau / constants.k_lower_crank_dict["k_gear_ratio"])
        self.abs_encoder.setVelocityConversionFactor(math.tau / constants.k_lower_crank_dict["k_gear_ratio"])
        self.abs_encoder.setZeroOffset(constants.k_lower_crank_dict["k_abs_encoder_offset"])

        # since everything's in radians, the encoders should agree
        self.encoder.setPosition(self.abs_encoder.getPosition())

        self.pid_controller = self.sparkmax.getPIDController()
        self.pid_controller.setP(gain=constants.k_lower_crank_dict['kP'], slotID=0)
        self.pid_controller.setI(gain=constants.k_lower_crank_dict['kI'], slotID=0)
        self.pid_controller.setD(gain=constants.k_lower_crank_dict['kD'], slotID=0)
        self.pid_controller.setIZone(math.radians(5), slotID=0)
        self.pid_controller.setIMaxAccum(iMaxAccum=0.1, slotID=0)

        self.sparkmax.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, constants.k_lower_crank_dict["k_forward_limit"])
        self.sparkmax.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, constants.k_lower_crank_dict["k_reverse_limit"])

        self.sparkmax.burnFlash()
        
        self.counter = 0

    def set_position(self, position: float) -> None:
        self.pid_controller.setReference(value=position, ctrl=CANSparkMax.ControlType.kPosition, pidSlot=0)

        # cory's method: if this is a sim, just set the encoder to the setpoint
        # todo: in the sim, set up the whole singlejointedarmsim, then use that to calculate the value which we set the encoder to.
        if False: # wpilib.RobotBase.isSimulation():
            self.encoder.setPosition(position)

    def get_angle(self) -> float:
        return self.abs_encoder.getPosition()

    def set_encoder_position(self, radians: float):
        print(f"crank is settings its encoder to {radians} rad")
        self.encoder.setPosition(radians)

    def periodic(self) -> None:

        self.counter += 1
        if self.counter % 100 > 50:
            print("setting position to 90 degrees!")
            self.set_position(math.radians(90))
        else:
            print("setting position to 45 degrees!")
            self.set_position(math.radians(45))

        return super().periodic()
