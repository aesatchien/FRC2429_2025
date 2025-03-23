import math
import commands2
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.commands import FollowPathCommand
from pathplannerlib.path import PathPlannerPath

from commands.go_to_coral_station import GoToCoralStation
from commands.go_to_reef_position import GoToReefPosition
from commands.go_to_stow import GoToStow
from commands.move_wrist import MoveWrist
from commands.run_intake import RunIntake
from commands.score import Score

class OnePlusOne(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()

        self.setName(f'1+1')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        self.addCommands(
                commands2.ParallelCommandGroup(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("1+1 preload")),
                    # this moves our wrist to 0 which is fine till we have to score on l4
                    GoToReefPosition(container, 1, 0).withTimeout(2)
                    )
                )

        self.addCommands(
                commands2.ParallelCommandGroup(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("1+1 go to hp left")),
                    Score(container).andThen(GoToCoralStation(container))
                    )
                )

        # wait for HP to drop coral
        self.addCommands(
                commands2.WaitUntilCommand(
                    container.intake.has_coral
                    )
                )

        # move to HP while moving the wrist while waiting a bit more then stopping the intake
        # path handles going to l4
        self.addCommands(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("1+1 score left")).alongWith(
                    MoveWrist(container, math.radians(90), 2, incremental=False, wait_to_finish=True)
                    ).alongWith(
                        commands2.WaitCommand(0.5).andThen(
                            RunIntake(container, container.intake, 0)
                            )
                        )
                )
        
        self.addCommands(
                Score(container).withTimeout(3).andThen(GoToStow(container)).andThen(RunIntake(container, container.intake, 0))
                )

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

