import commands2
from rev import SparkMax
from commands.run_intake import RunIntake
import constants
from commands.smart_intake import SmartIntake
from commands.go_to_position import GoToPosition
from commands.move_wrist import MoveWrist


class IntakeSequence(commands2.SequentialCommandGroup):
    def __init__(self, container, position: str, indent=0) -> None:
        """
        TODO: replace these gotopositions with their appropriate split commands
        also btw this doesn't work cuz we don't have the sensors on the maniuplator
        """
        super().__init__()

        if position not in ["ground", "coral station", "algae low", "algae high"]:
            raise ValueError(f"Cannot run IntakeSequence on {position}!")

        self.setName(f'{self.getName()} (with position {position})')

        self.container = container

        gamepiece_being_intaked = constants.GamePiece.CORAL if position in ["ground", "coral station"] else constants.GamePiece.ALGAE

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} to {position} **"))

        self.addCommands(GoToPosition(container=self.container, position=position, indent=indent+1))
        self.addCommands(SmartIntake(container=container, intake=container.intake, game_piece=gamepiece_being_intaked, wait_to_finish=True, indent=indent+1))
        self.addCommands(GoToPosition(container=self.container, position="stow", indent=indent+1))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} to {position} **"))

        
