import math

import rev
import wpilib
from wpimath.system.plant import DCMotor
from commands2 import Subsystem
from wpilib import SmartDashboard
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig, SparkMaxSim, CANSparkMax, MotorType, SparkMaxAnalogSensor
import constants
from playingwithfusion import TimeOfFlight

class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Intake')
        self.counter = 3
        motor_type = rev.CANSparkMax.MotorType.kBrushless
        self.motor = rev.CANSparkMax(constants.k_intake_neo_port, motor_type)
        self.motor.setInverted(True)
        self.encoder = self.motor.getEncoder()
        self.intake_controller = self.motor.getPIDController()
        self.intake_controller.setP(0)
        self.intake_on = False
        SmartDashboard.putBoolean('intake_enabled', self.intake_on)
        self.sparkmax = SparkMax(12, DCMotor.NEO550(1))
        self.sparkmax.configure(config=self.config,
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)
        self.motor = SparkMax(12, MotorType.kBrushless)
        self.controller = self.sparkmax.getClosedLoopController()
        self.use_intake_sim = False
        if wpilib.RobotBase.isSimulation():
            self.use_intake_sim = True
            self.sparkmax_sim = SparkMaxSim(self.sparkmax, DCMotor.NEO550(1))
        self.motor.setInverted(True)
        # Intake TOF Sensor
        self.TOFSensorAlgae = TimeOfFlight(2)
        self.TOFSensorCoral = TimeOfFlight(3)
        self.TOFSensorAlgae.setRangingMode(mode=TimeOfFlight.RangingMode.kShort, sampleTime=10)
        self.TOFSensorCoral.setRangingMode(mode=TimeOfFlight.RangingMode.kShort, sampleTime=10)
        self.TOFSensorAlgae.setRangeOfInterest(2, 2, 2, 2)
        self.TOFSensorCoral.setRangeOfInterest(1, 1, 1, 1)

    def set_reference(self, value: float, control_type: SparkMax.ControlType):
        self.controller.setReference(value, control_type)


    def turn_off(self):
        self.controller.setReference(0, SparkMax.ControlType.kVoltage)
        if not self.use_intake_sim:
            self.sparkmax_sim.setMotorCurrent(0)
        pass

    # Use TOF
    def has_algae(self) -> bool:
        average_algae_distance = self.TOFSensorAlgae.getRange()
        return average_algae_distance <= 0.5
        pass

    def has_coral(self) -> bool:
        average_coral_distance = self.TOFSensorAlgae.getRange()
        return average_coral_distance <= 2
        pass