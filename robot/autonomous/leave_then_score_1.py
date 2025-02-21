import math
import commands2
from pathplannerlib.auto import AutoBuilder, PathPlannerPath
from wpimath.geometry import Pose2d
from commands.drive_by_distance_swerve import DriveByVelocitySwerve
from commands.go_to_position import GoToPosition
from commands.move_wrist import MoveWrist
from commands.run_intake import RunIntake
from commands.score import Score

class LeaveThenScore(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()

        self.setName(f'Leave then score')

        self.container = container

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("1+0 path")))
        self.addCommands(GoToPosition(container=container, position="l4", indent=indent+1))
        self.addCommands(MoveWrist(container=container, radians=math.radians(-90), timeout=3, wait_to_finish=True, indent=indent+1))
        self.addCommands(Score(container=container, indent=indent+1))
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

