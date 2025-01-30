import math

import rev
import wpilib
from wpimath.system.plant import DCMotor
from commands2 import Subsystem
from wpilib import SmartDashboard
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig, SparkMaxSim, SparkMax
import constants
from playingwithfusion import TimeOfFlight

class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Intake')
        self.counter = 3

        self.sparkmax = rev.SparkMax(constants.IntakeConstants.k_CAN_id, rev.SparkMax.MotorType.kBrushless)

        self.encoder = self.sparkmax.getEncoder()
        self.intake_on = False

        SmartDashboard.putBoolean('intake_enabled', self.intake_on)

        self.sparkmax.configure(config=constants.IntakeConstants.k_intake_config,
                                resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                persistMode=SparkMax.PersistMode.kPersistParameters)


        self.controller = self.sparkmax.getClosedLoopController()

        if wpilib.RobotBase.isSimulation():
            self.sparkmax_sim = SparkMaxSim(self.sparkmax, DCMotor.NEO550(1))

        # Intake TOF Sensor
        self.TOFSensorAlgae = TimeOfFlight(constants.IntakeConstants.k_tof_algae_port)
        self.TOFSensorCoral = TimeOfFlight(constants.IntakeConstants.k_tof_coral_port)

        self.TOFSensorAlgae.setRangingMode(mode=TimeOfFlight.RangingMode.kShort, sampleTime=10)
        self.TOFSensorCoral.setRangingMode(mode=TimeOfFlight.RangingMode.kShort, sampleTime=10)

        self.TOFSensorAlgae.setRangeOfInterest(2, 2, 2, 2)
        self.TOFSensorCoral.setRangeOfInterest(1, 1, 1, 1)


    def turn_on(self) -> None:
        self.controller.setReference(12, SparkMax.ControlType.kVoltage)
        # if self.use_intake_sim:
        #     self.sparkmax_sim.setMotorCurrent(23)

    def turn_off(self):
        self.controller.setReference(0, SparkMax.ControlType.kVoltage)
        # if not self.use_intake_sim:
        #     self.sparkmax_sim.setMotorCurrent(0)

    # Use TOF
    def has_algae(self) -> bool:
        average_algae_distance = self.TOFSensorAlgae.getRange()
        return average_algae_distance <= 0.5

    def has_coral(self) -> bool:
        average_coral_distance = self.TOFSensorAlgae.getRange()
        return average_coral_distance <= 2

    def periodic(self) -> None:

        return super().periodic()

