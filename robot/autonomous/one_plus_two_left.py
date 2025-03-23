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
import constants

class OnePlusTwoLeft(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()

        self.setName(f'1+2 LEFT')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        # run the path that takes us by the reef, drops the coral in the trough, and goes to human station
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n LEFT A driveby preload')))

        # wait for piece to come in
        self.addCommands(commands2.WaitUntilCommand(container.intake.has_coral))

        # --------------- STEP B --------------

        # go from HP to get ready to score at L whatever is in the path
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n LEFT B score')).alongWith(
            MoveWrist(container, math.radians(90), 2, wait_to_finish=True).alongWith(
                WaitCommand(0.4).andThen(RunIntake(container, container.intake, 0))
                )
            ))

        # --------------- STEP C --------------
        # score then to go HP while driving back to HP
        self.addCommands(
                WaitCommand(0.4).andThen(RunIntake(container, container.intake, constants.IntakeConstants.k_coral_scoring_voltage)).andThen(WaitCommand(0.3)).andThen(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n LEFT C to HP'))
                    )
                )

        # wait for piece to come in
        self.addCommands(commands2.WaitUntilCommand(container.intake.has_coral))
        #
        # # --------------- STEP D --------------
        # drive to D
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n LEFT D score')).alongWith(
            MoveWrist(container, math.radians(90), 2, wait_to_finish=True).alongWith(
                WaitCommand(0.4).andThen(RunIntake(container, container.intake, 0))
                )
            ))

        # score on D with 3-second timeout then stow and turn off intake
        self.addCommands(PrintCommand("final score")                )
        self.addCommands(
                WaitCommand(0.1).andThen(Score(container).withTimeout(3).andThen(GoToStow(container)).andThen(RunIntake(container, container.intake, 0))
                ))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

