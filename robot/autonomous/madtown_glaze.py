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
from commands.move_pivot import MovePivot
from commands.move_wrist import MoveWrist
from commands.run_intake import RunIntake
from commands.score import Score
from commands.move_wrist_swap import MoveWristSwap
import constants

class MadtownGlaze(commands2.SequentialCommandGroup):
    def __init__(self, container, indent=0) -> None:
        super().__init__()

        self.setName(f'madtown lalasidrl')
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        # --------------- STEP 1 --------------

        self.addCommands(
                commands2.ParallelCommandGroup(
                    AutoBuilder.followPath(PathPlannerPath.fromPathFile("1+0 path")),
                    # this moves our wrist to 0 which is fine till we have to score on l4
                    GoToReefPosition(container, 4).withTimeout(3),
                    MoveWristSwap(container, container.wrist)
                    )
                )

        # --------------- STEP 2 --------------
        # score then to go HP while driving back to HP
        self.addCommands(
                WaitCommand(5).andThen(RunIntake(container, container.intake, constants.IntakeConstants.k_coral_scoring_voltage)).andThen(WaitCommand(0.3)).andThen(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile('go around for MADTOWN'))
                    )
                )

        # wait for piece to come in
        self.addCommands(commands2.WaitUntilCommand(container.intake.has_coral))
        #
        # # --------------- STEP 3 --------------

        # drive to D
        self.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile('1+n RIGHT D score')).alongWith(
            MoveWrist(container, math.radians(90), 2, wait_to_finish=True).alongWith(
                WaitCommand(0.4).andThen(RunIntake(container, container.intake, 0))
                )
            ))

        # score on D with 3-second timeout then stow and turn off intake
        self.addCommands(PrintCommand("final score"))
        self.addCommands(
                WaitCommand(0.1).andThen(Score(container).withTimeout(0.7).andThen(MovePivot(container, container.pivot, "specified", angle=math.radians(90)))
                ))

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

