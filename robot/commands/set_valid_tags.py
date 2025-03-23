import commands2
import wpilib
import typing
import constants

from wpilib import SmartDashboard

class SetValidTags(commands2.Command):
    def __init__(self, container, k_valid_tags:typing.List[int], indent=0) -> None:
        super().__init__()
        self.setName('Set Valid Tags')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.k_valid_tags = k_valid_tags
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
        
        self.container.swerve.setDesiredTags(self.k_valid_tags)

        SmartDashboard.putNumberArray("valid tag IDs", self.k_valid_tags)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
