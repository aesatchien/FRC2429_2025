import commands2
from commands.move_wrist import MoveWrist
from commands.move_pivot import MovePivot
from commands.move_elevator import MoveElevator
import constants

class AutoL1(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'Auto L1')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        self.addCommands(MoveElevator(container, container.elevator, 'specified', constants.k_positions["l1"]["elevator"], use_dash=False, wait_to_finish=False, indent=indent+1))
        self.addCommands(MovePivot(container, container.pivot, "specified", constants.k_positions["l1"]["shoulder_pivot"], use_dash=False, wait_to_finish=False, indent=indent+1))
        self.addCommands(MoveWrist(container, constants.k_positions["l1"]["wrist_pivot"], 1, False, False, indent=indent+1))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

