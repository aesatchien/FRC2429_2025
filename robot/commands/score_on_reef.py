import commands2
from commands.go_to_position import GoToPosition
from commands.move_wrist_by_joystick import MoveWristByJoystick
from commands.score import Score

class ScoreOnReef(commands2.SequentialCommandGroup):
    def __init__(self, container, level: int, indent=0) -> None:
        super().__init__()

        self.setName(f'Score on reef')

        self.container = container

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        self.addCommands(GoToPosition(container=container, position="wrist_clearing"))
        self.addCommands(MoveWristByJoystick(container=container, side_decider=container.co_pilot_command_controller, timeout=10, wait_to_finish=True, indent=indent+1))
        self.addCommands(GoToPosition(container=container, position=f"l{level}", indent=indent+1))
        self.addCommands(Score(container=container, indent=indent+1))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

