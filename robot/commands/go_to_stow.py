import commands2
from commands.reflash import Reflash
import constants
from commands.move_wrist import MoveWrist
from commands.move_pivot import MovePivot
from commands.move_elevator import MoveElevator

class GoToStow(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()

        self.setName(f'Go to stow')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))
        
        self.addCommands(MoveWrist(container, constants.k_positions["stow"]["wrist_pivot"], 5, False, True, indent+1))

        self.addCommands(commands2.ParallelCommandGroup(
                    MovePivot(container=container, pivot=container.pivot, mode='specified',
                              angle=constants.k_positions["stow"]["shoulder_pivot"], wait_to_finish=True, indent=indent+1).withTimeout(5),
                    MoveElevator(container=container, elevator=self.container.elevator, mode="specified",
                                 height=constants.k_positions["stow"]["elevator"], wait_to_finish=True, indent=indent + 1).withTimeout(5)
        ))

        self.addCommands(Reflash(container))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

