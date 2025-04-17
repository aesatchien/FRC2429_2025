import math
import commands2
from wpilib import SmartDashboard

import constants
from subsystems.wrist import Wrist


class MoveWristSwap(commands2.Command):

    def __init__(self, container, wrist: Wrist, indent=0) -> None:
        super().__init__()
        self.setName('Move wrist swap')
        self.indent = indent
        self.container = container
        self.wrist = wrist
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        if self.wrist.get_angle() > math.radians(45):
            self.wrist.set_position(math.radians(constants.k_wrist_negative_90ish_angle))
        elif self.wrist.get_angle() < math.radians(45):
            self.wrist.set_position(math.radians(constants.k_wrist_positive_90ish_angle))
        else:
            print("Didn't swap wrist- it's not in a swappable position anyways!")

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
