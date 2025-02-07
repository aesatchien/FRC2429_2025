import commands2
import constants

from commands.move_elevator import MoveElevator
from commands.move_shoulder import MoveShoulder

class GoToPosition(commands2.SequentialCommandGroup):
    def __init__(self, container, position: str) -> None:
        super().__init__()

        if position not in constants.k_positions.keys():
            raise ValueError("Unrecognized position requested while making GoToPosition!")

        self.setName(f'Go to position (with position {position})')

        self.container = container

        self.addCommands(commands2.ParallelCommandGroup(
            MoveShoulder(container=self.container, shoulder=self.container.shoulder, radians=constants.k_positions[position]["shoulder_pivot"], wait_to_finish=True),
            MoveElevator(container=self.container, elevator=self.container.elevator, target=constants.k_positions[position]["elevator"], wait_to_finish=True)
        ))

