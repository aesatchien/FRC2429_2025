import commands2
from commands.log_command import log_command  # outsource explicit logging clutter to a single line
from subsystems.swerve import Swerve
from subsystems.swervemodule_2429 import SwerveModuleState
from wpimath.geometry import Rotation2d

@log_command(console=True, nt=False, print_init=True, print_end=True)  # will print start and end messages
class SwerveTest(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve:Swerve, indent=0) -> None:
        super().__init__()
        self.setName('SwerveTest')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.swerve = swerve
        self.extra_log_info = None
        self.counter = 0
        self.addRequirements(self.swerve)  # commands2 version of requirements - add the subsystems you need

    def initialize(self) -> None:
        # Called just before each time this Command runs
        # if you wish to add more information to the console logger, change self.extra_log_info
        # self.extra_log_info = "Target=7"  # (for example)
        self.counter = 0
        pass

    def execute(self) -> None:
        self.counter += 1
        interval = 400
        test_angle = 22.5  # degrees
        test_velocity = 0.25 # m/s
        angles = [test_angle, 0, 0, 0]
        velocities = [test_velocity, 0, 0, 0]
        index = 0

        # get an index for rotating
        if 0 < self.counter % interval <= 100:
            index = 0
        elif 100 < self.counter % interval <= 200:
            index = 1
        elif 200 < self.counter % interval <= 300:
            index = 2
        elif 300 < self.counter % interval <= 400:
            index = 3

        # rotate the list - negative numbers will rotate right
        # should go through self.frontLeft, self.frontRight, self.rearLeft, self.rearRight
        angles = angles[-index:] + angles[:-index]
        velocities = velocities[-index:] + velocities[:-index]

        # rotate thru the modules
        for angle, velocity, swerve_module in zip(angles, velocities, self.swerve.swerve_modules):
            swerve_module.setDesiredState(SwerveModuleState(velocity, Rotation2d.fromDegrees(angle)))

        # runs 50x per second, so be careful about messages and timing
        pass

    def isFinished(self) -> bool:
        # True: fire once and end; False: run forever until interrupted; logic has it end when code returns True
        return False

    def end(self, interrupted: bool) -> None:
        # put your safe cleanup code here - turn off motors, set LEDs, etc
        pass