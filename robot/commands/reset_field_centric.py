import math
import commands2
from wpilib import SmartDashboard
from subsystems.swerve import Swerve
from wpimath.geometry import Pose2d

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
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"{'    ' * self.indent}** Started {self.getName()} at {self.start_time} s **", flush=True)
        print(f"setting x to {self.swerve.get_pose().X()}; y to {self.swerve.get_pose().Y()}; theta to {self.angle}")
        fixed_pose = Pose2d(x=self.swerve.get_pose().X(), y=self.swerve.get_pose().Y(), angle=self.angle)
        self.swerve.resetOdometry(fixed_pose)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{'    ' * self.indent}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
