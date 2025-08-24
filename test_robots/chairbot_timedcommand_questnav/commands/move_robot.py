import commands2
from wpilib import SmartDashboard

from subsystems.drivetrain import Drivetrain

class MoveRobot(commands2.Command):  # change the name for your command

    def __init__(self, container, drive: Drivetrain) -> None:
        super().__init__()
        self.setName('MoveRobot')  # change this to something appropriate for this command
        self.container = container
        self.drive = drive
        self.addRequirements(self.container.drive)  # commandsv2 version of requirements

        self.questnav = QuestNav()
        self.quest_to_robot = Transform2d(inchesToMeters(-12.5-0.55), 0, Rotation2d().fromDegrees(0))
        # self.quest_to_robot = Transform2d(inchesToMeters(4), 0, Rotation2d().fromDegrees(0))
        self.quest_field = Field2d()

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        self.drive.drive()


    def isFinished(self) -> bool:
        return False

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
        SmartDashboard.putString(f"alert",
                                 f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
