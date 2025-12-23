import math
import commands2
from wpilib import SmartDashboard
from subsystems.swerve import Swerve
from wpimath.geometry import Pose2d
from commands.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=True)
class ResetFieldCentric(commands2.Command):

    def __init__(self, container, swerve: Swerve, angle: float=0, indent=0) -> None:
        super().__init__()
        self.indent = indent
        self.setName('Reset field centric')  # change this to something appropriate for this command
        self.container = container
        self.swerve = swerve
        self.angle = angle # todo: set to 0 if blue alliance, else 180 deg?
                            # was considering doing a conditionalcommand but that's bad if we boot up before connecting to fms


    def initialize(self) -> None:
        """Called just before this Command runs the first time."""

        fixed_pose = Pose2d(x=self.swerve.get_pose().X(), y=self.swerve.get_pose().Y(), angle=self.angle)
        self.swerve.resetOdometry(fixed_pose)

    def execute(self) -> None:
        # because it fires once and the auto-log is a one-liner
        print(f"setting x to {self.swerve.get_pose().X()}; y to {self.swerve.get_pose().Y()}; theta to {self.angle}")

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        pass
