import math
import commands2
from pathplannerlib.auto import AutoBuilder, PathPlannerPath
from commands.drive_by_distance_swerve import DriveByVelocitySwerve
from commands.go_to_position import GoToPosition
from commands.go_to_reef_position import GoToReefPosition
from commands.score import Score

class LeaveThenScore(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()

        self.setName(f'Leave then score')

        self.container = container

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("1+0 path")))
        self.addCommands(GoToReefPosition(container=container, level=2, wrist_setpoint=math.radians(90), indent=indent+1))
        self.addCommands(Score(container=container, indent=indent+1))
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

