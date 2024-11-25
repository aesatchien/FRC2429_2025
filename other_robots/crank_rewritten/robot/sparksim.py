import wpilib
from rev import (
        REVLibError, 
        SparkMaxAbsoluteEncoder as RealSparkMaxAbsoluteEncoder,
        SparkMaxRelativeEncoder as RealSparkMaxRelativeEncoder,
        CANSparkMax as RealCANSparkMax
)
from wpimath.controller import PIDController

class SparkMaxRelativeEncoder():
    def __init__(self):
        # super().__init__()
        self._velocity = 0
        self._position = 0
        self._velocity_conversion_factor = 1
        self._position_conversion_factor = 1

    def getVelocity(self):
        return self._velocity

    def getPosition(self):
        return self._position

    def setPosition(self, position) -> REVLibError:
        print(f"spark max relative encoder is being set to {position}")
        self._position = position
        return REVLibError.kOk

    def setVelocity(self, velocity):
        self._velocity = velocity

    def setPositionConversionFactor(self, factor):
        self._position_conversion_factor = factor
        return REVLibError.kOk

    def setVelocityConversionFactor(self, factor):
        self._velocity_conversion_factor = factor
        return REVLibError.kOk

class SparkMaxAbsoluteEncoder():
    def __init__(self):
        # super().__init__()
        self._zero_offset = 0
        self._velocity = 0
        self._position = 0
        self._velocity_conversion_factor = 1
        self._position_conversion_factor = 1

    def getVelocity(self):
        return self._velocity * self._velocity_conversion_factor

    def getPosition(self):
        return (self._position * self._position_conversion_factor) + self._zero_offset

    def getZeroOffset(self):
        return self._zero_offset

    def setPosition(self, position) -> REVLibError:
        self._position = position
        return REVLibError.kOk

    def setVelocity(self, velocity):
        self._velocity = velocity

    def setPositionConversionFactor(self, factor):
        self._position_conversion_factor = factor
        return REVLibError.kOk

    def setVelocityConversionFactor(self, factor):
        self._velocity_conversion_factor = factor
        return REVLibError.kOk

    def setZeroOffset(self, offset: float):
        self._zero_offset = offset


class SparkMaxPIDController:
    def __init__(self, motor: "CANSparkMax") -> None:
        self._motor = motor
        self._min_output = -1
        self._max_output = 1
        self._controllers = [PIDController(0, 0, 0)] * 4 # one for each slot
        self._forwards_limit = None
        self._backwards_limit = None

    def setP(self, gain, slotID):
        self._controllers[slotID].setP(gain)

    def setI(self, gain, slotID):
        self._controllers[slotID].setI(gain)

    def setD(self, gain, slotID):
        self._controllers[slotID].setD(gain)

    def setIZone(self, izone, slotID):
        """
        not implemented, may result in unexpected behaviors
        """
        pass

    def setIMaxAccum(self, iMaxAccum, slotID):
        """
        not implemented, may result in unexpected behaviors
        """
        pass

    def setFF(self, ff):
        """
        not implemented, may result in unexpected behaviors
        """
        pass

    def setOutputRange(self, min_output, max_output):
        self._min_output = min_output
        self._max_output = max_output

    def setReference(self, value: float, ctrl: RealCANSparkMax.ControlType, pidSlot: int):
        if not self._forwards_limit == None:
            if value > self._forwards_limit:
                print("Setting simulated spark reference to forward limit since request exceeded limits")
                value = self._forwards_limit

        if not self._backwards_limit == None:
            if value < self._backwards_limit:
                print("Setting simulated spark reference to reverse limit since request exceeded limits")
                value = self._backwards_limit

        v = self._controllers[pidSlot].calculate(self._motor._encoder.getPosition(), value)
        print(f"PID controller requests output {v} based on position {self._motor._encoder.getPosition()} and setpoint {value}")
        if v > self._max_output:
            v = self._max_output
        elif v < self._min_output:
            v = self._min_output
        self._motor.set(v)

    def _setPIDSoftLimit(self, direction, limit: float):
        if direction == RealCANSparkMax.SoftLimitDirection.kForward:
            self._forwards_limit = limit
        else:
            self._backwards_limit = limit
        pass

class CANSparkMax(wpilib.Spark):
    class MotorType:
        kBrushless = 1

    class ControlType:
        kDutyCycle = 1
        kPosition = 2
        kVelocity = 3
        kVoltage = 4

    class IdleMode:
        kCoast = 0
        kBrake = 1

    def __init__(self, channel: int, type: RealCANSparkMax.MotorType) -> None:
        super().__init__(channel)
        self._encoder = SparkMaxRelativeEncoder()
        self._abs_encoder = SparkMaxAbsoluteEncoder()
        self._pidController = SparkMaxPIDController(self)

    def follow(self, motor, invert):
        pass

    def getEncoder(self):
        return self._encoder

    def getAbsoluteEncoder(self):
        return self._abs_encoder

    def getPIDController(self):
        return self._pidController

    def getAppliedOutput(self):
        """
        here i'm running into an impasse- I don't know the units of the error
        """
        pass

    def setIdleMode(self, mode):
        pass

    def burnFlash(self):
        pass

    def restoreFactoryDefaults(self):
        pass
    
    def setSoftLimit(self, direction: RealCANSparkMax.SoftLimitDirection, limit):
        self._pidController._setPIDSoftLimit(direction, limit)


