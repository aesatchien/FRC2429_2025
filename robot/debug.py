import math
import commands2
from commands2.printcommand import PrintCommand
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

        self.setName(f'1+1')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        # run driveby path (this handles l1 and gets us ready to intake)
        self.addCommands(PrintCommand("starting driveby"))
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n driveby preload')))
        self.addCommands(PrintCommand("ending driveby"))
        # wait for piece to come in
        self.addCommands(
                commands2.WaitUntilCommand(
                    container.intake.has_coral
                    ).withTimeout(6)
                )
        # drive to C
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n score C')).alongWith(
            MoveWrist(container, math.radians(90), 2, wait_to_finish=True)
            ))
        # score then to go HP while driving back to HP
        self.addCommands(
                AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n C to HP')).alongWith(
                    Score(container).andThen(
                        GoToCoralStation(container)
                        )
                    )
                )
        # wait for piece
        self.addCommands(
                commands2.WaitUntilCommand(
                    container.intake.has_coral
                    ).withTimeout(6)
                )
        # drive to D
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n score D')).alongWith(
            MoveWrist(container, math.radians(90), 2, wait_to_finish=True)
                ).withTimeout(5)
            )

        # score on D with 3-second timeout then stow and turn off intake
        self.addCommands(
                PrintCommand("final score")
                )
        self.addCommands(
                Score(container).withTimeout(3).andThen(GoToStow(container)).andThen(RunIntake(container, container.intake, 0))
                )


        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

