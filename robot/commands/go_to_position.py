import commands2
from commands.move_pivot import MovePivot
from commands.move_wrist import MoveWrist
import constants

from commands.move_elevator import MoveElevator

# if l1: idk
# if l2: go higher than usual so safe to spin wrist; then spin wrist; then go back down and score
# if l3 or l4: move elevator, move wrist, then move pivot

# steps:
# 1. begin driving AND moving subsystems
# 2. move wrist after elevator (or? pivot) is ready
# 3. 


class GoToPosition(commands2.SequentialCommandGroup):
    def __init__(self, container, position: str, indent=0) -> None:
        """
        @deprecated
        DON'T USE. I've replaced it with individual commands for different positions
        because this one was getting too big and having too many cases. I don't want 
        to deal with debugging all that at comp. LHACK 2/26/25

        .. deprecated:: what the hell
        """
        super().__init__()

        self.setName(f'Go to position (with position {position})')
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} to {position} **"))
        self.container = container

        self.addCommands(commands2.ParallelCommandGroup(
            MoveElevator(container=container, elevator=container.elevator, mode='specified', height=constants.k_positions[position]["elevator"], wait_to_finish=True, indent=indent+1),
            MovePivot(container=container, pivot=container.pivot, mode='specified', angle=constants.k_positions[position]["shoulder_pivot"], wait_to_finish=True, indent=indent+1),
            MoveWrist(container=container, radians=constants.k_positions[position]["wrist_pivot"], timeout=8, incremental=False, wait_to_finish=True, indent=indent+1)
        ))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} to {position} **"))


