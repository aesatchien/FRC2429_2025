import commands2
from rev import SparkMax
from wpilib import SmartDashboard
import constants
from subsystems.intake import Intake


class RunIntake(commands2.Command):  # change the name for your command

    def __init__(self, container, intake: Intake, value: float, control_type: SparkMax.ControlType = SparkMax.ControlType.kVoltage, stop_on_end=False, indent=0) -> None:
        super().__init__()
        self.setName('Run Intake')  # change this to something appropriate for this command
        self.container = container
        self.intake = intake
        self.control_type = control_type
        self.value = value
        self.indent = indent
        self.stop_on_end = stop_on_end
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        msg = self.indent * "    " + f"** Started {self.getName()} with value {self.value} at {self.start_time} s **"
        print(msg, flush=True)
        SmartDashboard.putString("alert", msg)

        if self.value < 0:
            constants.IntakeConstants.k_intake_config.smartCurrentLimit(5)
        else:
            constants.IntakeConstants.k_intake_config.smartCurrentLimit(10)

        self.intake.sparkmax.configure(constants.IntakeConstants.k_intake_config,
                                       SparkMax.ResetMode.kNoResetSafeParameters,
                                       SparkMax.PersistMode.kNoPersistParameters)

        self.intake.set_reference(value=self.value, control_type=self.control_type)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.stop_on_end:
            return False # we want it to stop when this command is killed by something else
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        if self.stop_on_end:
            self.intake.set_reference(value=0, control_type=SparkMax.ControlType.kVoltage)

        print_end_message = False
        if print_end_message:
            print(self.indent * "    " + f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
