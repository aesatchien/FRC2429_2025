from commands2.subsystem import Subsystem
import wpilib
from wpimath.system.plant import DCMotor
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig, SparkMaxSim 
from constants import LowerCrankConstants
import math

import constants


class LowerCrank(Subsystem):

    def __init__(self, container):

        self.container = container

        self.sparkmax = SparkMax(LowerCrankConstants.k_CAN_id, SparkMax.MotorType.kBrushless)

        self.config = SparkMaxConfig()

        self.config.closedLoop.pid(p=constants.LowerCrankConstants.kP, i=0, d=0, slot=ClosedLoopSlot(0))

        self.config.encoder.positionConversionFactor(math.tau / LowerCrankConstants.k_gear_ratio)
        self.config.encoder.velocityConversionFactor(math.tau / LowerCrankConstants.k_gear_ratio)

        self.config.absoluteEncoder.positionConversionFactor(math.tau / LowerCrankConstants.k_gear_ratio)
        self.config.absoluteEncoder.velocityConversionFactor(math.tau / LowerCrankConstants.k_gear_ratio)
        self.config.absoluteEncoder.zeroOffset(LowerCrankConstants.k_abs_encoder_offset)

        self.config.closedLoop.pid(p=LowerCrankConstants.kP, i=LowerCrankConstants.kI, d=LowerCrankConstants.kD, slot=ClosedLoopSlot(0))
        self.config.closedLoop.IZone(iZone=LowerCrankConstants.kIZone, slot=ClosedLoopSlot(0))
        self.config.closedLoop.IMaxAccum(LowerCrankConstants.kIMaxAccum, slot=ClosedLoopSlot(0))
        
        self.config.softLimit.forwardSoftLimit(LowerCrankConstants.k_forward_limit)
        self.config.softLimit.reverseSoftLimit(LowerCrankConstants.k_reverse_limit)

        self.config.softLimit.forwardSoftLimitEnabled(True)
        self.config.softLimit.reverseSoftLimitEnabled(True)

        self.sparkmax.configure(config=self.config, 
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)

        self.encoder = self.sparkmax.getEncoder()
        self.abs_encoder = self.sparkmax.getAbsoluteEncoder()
        self.encoder.setPosition(self.abs_encoder.getPosition())

        self.controller = self.sparkmax.getClosedLoopController()

        if wpilib.RobotBase.isSimulation():
            self.sparkmax_sim = SparkMaxSim(self.sparkmax, DCMotor.NEO550(1))

        self.counter = 0
        self.setpoint = self.get_angle()

    def set_position(self, position: float) -> None:
        self.setpoint = position
        self.controller.setReference(value=self.setpoint, ctrl=SparkMax.ControlType.kPosition, pidSlot=0)

    def get_angle(self) -> float:
        return self.abs_encoder.getPosition()

    def set_encoder_position(self, radians: float):
        self.encoder.setPosition(radians)

    def periodic(self) -> None:

        # self.counter += 1
        if wpilib.RobotBase.isSimulation():
            # the 
            # self.controller.setReference(value=self.setpoint, ctrl=SparkMax.ControlType.kPosition, pidSlot=0)
            pass

        return super().periodic()
