import commands2
from rev import SparkMax
from wpilib import SmartDashboard
from subsystems.pivot import Pivot
import constants


class Reflash(commands2.Command):

    def __init__(self, container, indent=0) -> None:
        super().__init__()
        self.setName('Reflash sparkmaxes')
        self.indent = indent
        self.container = container
        self.pivot: Pivot = container.pivot
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.pivot.motor.configure(constants.ShoulderConstants.k_config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters)

        self.pivot.follower.configure(constants.ShoulderConstants.k_follower_config, SparkMax.ResetMode.kResetSafeParameters, SparkMax.PersistMode.kPersistParameters)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def runsWhenDisabled(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
