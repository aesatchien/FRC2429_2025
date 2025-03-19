import math

import rev
import wpilib
from wpimath.system.plant import DCMotor
from commands2 import Subsystem
from wpilib import SmartDashboard
from rev import ClosedLoopSlot, SparkMax, SparkMaxConfig, SparkMaxSim, SparkMax
from playingwithfusion import TimeOfFlight
import constants

class Intake(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Intake')
        self.counter = constants.IntakeConstants.k_counter_offset

        self.sparkmax = rev.SparkMax(constants.IntakeConstants.k_CAN_id, rev.SparkMax.MotorType.kBrushless)

        self.encoder = self.sparkmax.getEncoder()

        if constants.k_burn_flash:
            controller_revlib_error = self.sparkmax.configure(config=constants.IntakeConstants.k_intake_config,
                                                            resetMode=SparkMax.ResetMode.kResetSafeParameters,
                                                            persistMode=SparkMax.PersistMode.kPersistParameters)
            print(f"Reconfigured intake sparkmax. Wrist controller status: {controller_revlib_error}")


        self.controller = self.sparkmax.getClosedLoopController()

        if wpilib.RobotBase.isSimulation():
            self.sparkmax_sim = SparkMaxSim(self.sparkmax, DCMotor.NEO550(1))
        # note that if we are looking at lights, you may bounce from 0 to large numbers
        self.TOFSensorCoral = TimeOfFlight(constants.IntakeConstants.k_tof_coral_port)
        self.TOFSensorCoral.setRangingMode(mode=TimeOfFlight.RangingMode.kShort, sampleTime=50)

        wpilib.SmartDashboard.putNumber("SET intake volts", 0)

    def set_reference(self, value: float, control_type: SparkMax.ControlType = rev.SparkBase.ControlType.kVoltage):
        self.controller.setReference(value, control_type)

    def get_distance(self):
        average_coral_distance = (self.TOFSensorCoral.getRange() + self.TOFSensorCoral.getRange()) / 2
        average_coral_distance = 0 if average_coral_distance > 500 else average_coral_distance  # correct for weird stuff
        return average_coral_distance

    def has_coral(self) -> bool:
        coral_distance = self.get_distance()
        # reads 0 when no signal, so it has to be between 1 and the actual number of mm
        coral_present = 4 < coral_distance <= constants.IntakeConstants.k_max_tof_distance_where_we_have_coral
        return coral_present

    def periodic(self) -> None:
        # print(f"setting reserefsersf to {wpilib.SmartDashboard.getNumber('SET intake volts', 0)}")
        # self.controller.setReference(wpilib.SmartDashboard.getNumber("SET intake volts", 0), SparkMax.ControlType.kVoltage)


        if self.counter % 10 == 0:
            if wpilib.RobotBase.isReal():
                wpilib.SmartDashboard.putBoolean('gamepiece_present', self.has_coral())
            else:
                wpilib.SmartDashboard.putBoolean('gamepiece_present', self.counter % 200 < 100)

            wpilib.SmartDashboard.putNumber('intake_tof', self.get_distance())

            if constants.IntakeConstants.k_nt_debugging:  # extra debugging info for NT
                pass

        self.counter += 1
        return super().periodic()

