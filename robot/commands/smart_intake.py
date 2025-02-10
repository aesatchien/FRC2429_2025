import commands2
from rev import SparkMax
import wpilib
from wpilib import SmartDashboard
from subsystems.led import Led
from subsystems.intake import Intake
import constants


class SmartIntake(commands2.Command):

    def __init__(self, container, intake: Intake, game_piece: constants.GamePiece, timeout=15, wait_to_finish=True, indent=0) -> None:
        super().__init__()
        self.indent = indent
        self.setName('SmartIntake')
        self.container = container
        self.intake = intake

        self.game_piece = game_piece
        self.wait_to_finish = wait_to_finish
        self.timeout = timeout
        self.timer = wpilib.Timer()

        self.led: Led = container.led

        if game_piece == constants.GamePiece.CORAL:
            self.intake_voltage = constants.IntakeConstants.k_coral_intaking_voltage
        else:
            self.intake_voltage = constants.IntakeConstants.k_algae_intaking_voltage

        self.addRequirements(self.intake)
        self.count = 0

    def initialize(self) -> None:
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{'    ' * self.indent}** Started {self.getName()} at {self.start_time} s **", flush=True)

        self.timer.restart()

        self.intake.set_reference(value=self.intake_voltage, control_type=SparkMax.ControlType.kVoltage)

        # TODO: add led indication here that intake is on

    def execute(self) -> None:
        self.count += 1

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return self.intake.has_algae() or self.intake.has_coral() or self.timer.hasElapsed(self.timeout)
        else:
            return True

    def end(self, interrupted: bool) -> None:

        if self.intake.has_coral():
            self.led.set_mode(Led.Mode.kCORAL)

        elif self.intake.has_algae():
            self.led.set_mode(Led.Mode.kALGAE)

        else:
            self.led.set_mode(Led.Mode.kNONE)
            self.led.set_indicator(Led.Indicator.kFAILUREFLASH)

            self.container.set_robot_mode(self.container.RobotMode.EMPTY)

        self.intake.set_reference(value=0, control_type=SparkMax.ControlType.kVoltage)

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"{'    ' * self.indent}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")




