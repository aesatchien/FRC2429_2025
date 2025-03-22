import math
import commands2
from commands2.printcommand import PrintCommand
from commands2.waitcommand import WaitCommand
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.commands import FollowPathCommand
from pathplannerlib.path import PathPlannerPath

from commands.go_to_coral_station import GoToCoralStation
from commands.go_to_reef_position import GoToReefPosition
from commands.go_to_stow import GoToStow
from commands.move_wrist import MoveWrist
from commands.run_intake import RunIntake
from commands.score import Score

class OnePlusTwo(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()

        self.setName(f'1+2')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        # trough
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n driveby preload')))

        # wait for piece to come in
        self.addCommands(
                commands2.WaitUntilCommand(
                    container.intake.has_coral
                    )
                )

        # --------------- C --------------

        # drive to C
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n score C')).alongWith(
            MoveWrist(container, math.radians(90), 2, wait_to_finish=True).alongWith(
                WaitCommand(0.5).andThen(RunIntake(container, container.intake, 0))
                )
            ))

        # score then to go HP while driving back to HP
        self.addCommands(
                WaitCommand(0.5).andThen(
                        Score(container).andThen(GoToCoralStation(container).withTimeout(0.7)).alongWith(
                            commands2.WaitCommand(0.5).andThen(
                                AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n C to HP'))
                                )
                        )
                    )
                )

        # wait for piece to come in
        self.addCommands(
                commands2.WaitUntilCommand(
                    container.intake.has_coral
                    )
                )
        #
        # # --------------- D --------------
        #

        # drive to D
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n score D')).alongWith(
            MoveWrist(container, math.radians(90), 2, wait_to_finish=True).alongWith(
                WaitCommand(0.5).andThen(RunIntake(container, container.intake, 0))
                )
            ))

        # score on D with 3-second timeout then stow and turn off intake
        self.addCommands(
                PrintCommand("final score")
                )
        self.addCommands(
                WaitCommand(0.5).andThen(Score(container).withTimeout(3).andThen(GoToStow(container)).andThen(RunIntake(container, container.intake, 0))
                ))


        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

