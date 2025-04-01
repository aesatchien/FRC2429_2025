import commands2
from wpilib import SmartDashboard, Timer
from wpimath.geometry import Pose2d
from wpimath.filter import SlewRateLimiter

from subsystems.swerve import Swerve


class DriveByVelocitySwerve(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve: Swerve, velocity: Pose2d, timeout: float, indent=0) -> None:
        """
        all is in duty cycle, so be careful - a 1, 1, 1 pose2d floors the bot
        emergency command in case we can't get pathplanner working by the scrim
        """
        super().__init__()
        self.setName('Drive by velocity swerve')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.swerve = swerve
        self.velocity = velocity
        self.timeout = timeout
        self.timer = Timer()
        self.addRequirements(self.swerve)

        # CJH added this so AJ's DPAD is not so shaky - may need to take out for auto stuff  20250331
        stick_max_units_per_second = 2  # can't be too low or you get lag
        self.x_drive_limiter = SlewRateLimiter(stick_max_units_per_second)
        self.y_drive_limiter = SlewRateLimiter(stick_max_units_per_second)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.timer.reset()
        self.timer.start()
        self.x_drive_limiter.reset(0)
        self.y_drive_limiter.reset(0)


    def execute(self) -> None:
        x_out = self.x_drive_limiter.calculate(self.velocity.x)
        y_out = self.y_drive_limiter.calculate(self.velocity.y)
        self.swerve.drive(x_out, y_out, self.velocity.rotation().radians(), fieldRelative=False, keep_angle=False, rate_limited=True)
        pass

    def isFinished(self) -> bool:
        return self.timer.hasElapsed(self.timeout)

    def end(self, interrupted: bool) -> None:
        self.swerve.drive(0, 0, 0, True, False)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
