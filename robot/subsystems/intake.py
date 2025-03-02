import math

import rev
import wpilib
from wpimath.system.plant import DCMotor
from commands2 import Subsystem
from wpilib import SmartDashboard
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig, SparkMaxSim, SparkMax
#from playingwithfusion import TimeOfFlight
import constants

class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Intake')
        self.counter = 3

        self.sparkmax = rev.SparkMax(constants.IntakeConstants.k_CAN_id, rev.SparkMax.MotorType.kBrushless)

        self.encoder = self.sparkmax.getEncoder()

        self.sparkmax.configure(config=constants.IntakeConstants.k_intake_config,
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)
        self.controller = self.sparkmax.getClosedLoopController()

        if wpilib.RobotBase.isSimulation():
            self.sparkmax_sim = SparkMaxSim(self.sparkmax, DCMotor.NEO550(1))

        #self.TOFSensorCoral = TimeOfFlight(constants.IntakeConstants.k_tof_coral_port)
        #self.TOFSensorCoral.setRangingMode(mode=TimeOfFlight.RangingMode.kShort, sampleTime=10)

        wpilib.SmartDashboard.putNumber("SET intake volts", 0)

    def set_reference(self, value: float, control_type: SparkMax.ControlType = rev.SparkBase.ControlType.kVoltage):
        self.controller.setReference(value, control_type)

    def has_coral(self) -> bool:
        #average_coral_distance = self.TOFSensorCoral.getRange()
        average_coral_distance = 100
        return average_coral_distance <= constants.IntakeConstants.k_max_tof_distance_where_we_have_coral

    def periodic(self) -> None:
        # print(f"setting reserefsersf to {wpilib.SmartDashboard.getNumber('SET intake volts', 0)}")
        # self.controller.setReference(wpilib.SmartDashboard.getNumber("SET intake volts", 0), SparkMax.ControlType.kVoltage)

        return super().periodic()

