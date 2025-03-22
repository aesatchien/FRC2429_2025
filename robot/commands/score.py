import commands2
from rev import SparkMax
from commands.go_to_stow import GoToStow
import constants
from commands.run_intake import RunIntake
from commands.move_wrist_by_pose import StowWristAfterPositionDelta
from commands.set_leds import SetLEDs
from commands.rumble_command import RumbleCommand
from subsystems.led import Led

class Score(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        """
        not sure this is safe LHACK 2/19/25
        """
        super().__init__()

        self.setName(f'Score')

        self.container = container

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        self.addCommands(RunIntake(container=self.container, intake=self.container.intake, 
                                 value=constants.IntakeConstants.k_coral_scoring_voltage, 
                                 control_type=SparkMax.ControlType.kVoltage, indent=indent+1))
        self.addCommands(SetLEDs(container=self.container, led=self.container.led, indicator=Led.Indicator.kSUCCESSFLASH))
        self.addCommands(
                commands2.ParallelDeadlineGroup(
                        commands2.WaitCommand(constants.IntakeConstants.k_seconds_to_stay_on_while_scoring),
                        RumbleCommand(container, 1, True, True, 0.2, indent+1)        
                )
        )
        self.addCommands(commands2.WaitCommand(constants.IntakeConstants.k_seconds_to_stay_on_while_scoring))

        self.addCommands(StowWristAfterPositionDelta(container, wait_to_finish=True, indent=indent+1))
        self.addCommands(GoToStow(container=self.container, indent=indent+1))
        self.addCommands(RunIntake(container=self.container, intake=self.container.intake, value=0,
                                    control_type=SparkMax.ControlType.kVoltage, indent=indent + 1))
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))
