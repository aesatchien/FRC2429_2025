import commands2
from rev import SparkMax
import constants
from commands.run_intake import RunIntake

class Score(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()

        self.setName(f'Score')

        self.container = container

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))
        self.addCommands(commands2.ConditionalCommand(
                onTrue=RunIntake(container=self.container, intake=self.container.intake, 
                                 value=constants.IntakeConstants.k_coral_intaking_voltage, 
                                 control_type=SparkMax.ControlType.kVoltage, indent=indent+1),
                onFalse=RunIntake(container=self.container, intake=self.container.intake,
                                  value=constants.IntakeConstants.k_algae_intaking_voltage,
                                  control_type=SparkMax.ControlType.kVoltage, indent=indent+1),
                condition=lambda: self.container.get_robot_mode() == self.container.RobotMode.HAS_ALGAE
            )
        )
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

