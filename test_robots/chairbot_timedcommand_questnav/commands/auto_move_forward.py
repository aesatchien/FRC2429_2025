import commands2
from commands2.button import JoystickButton
from wpilib import SmartDashboard, Joystick

from subsystems.drivetrain import Drivetrain


class AutoMoveForward(commands2.Command):  # change the name for your command

    def __init__(self, container, drive: Drivetrain, joystick: Joystick) -> None:
        super().__init__()
        self.setName('AutoMoveForward')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.joystick = joystick
        self.addRequirements(self.container.drive)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        self.drive.drive_forward(power=0.1)


    def isFinished(self) -> bool:
        return not self.joystick.getRawButton(1)

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")