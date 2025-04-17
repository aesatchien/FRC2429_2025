import commands2
from commands.auto_to_pose import AutoToPose
from commands.go_to_reef_position import GoToReefPosition
from commands.score import Score
from commands.move_wrist_swap import MoveWristSwap
import constants

class L4PreloadAutoAim(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName(f'l4 preload auto aim')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        self.addCommands(AutoToPose(container, container.swerve, constants.k_useful_robot_poses_blue['g'], nearest=False, from_robot_state=False, control_type='not_pathplanner').alongWith(
            GoToReefPosition(container, 4, auto=False) # auto false because we have enough time to let it slide down
            ).alongWith(
                MoveWristSwap(container, container.wrist)
                ))

        self.addCommands(Score(container))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

