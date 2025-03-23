import commands2
from wpilib import SmartDashboard
from commands2.button import CommandXboxController


class CalibrateJoystick(commands2.Command):  # change the name for your command

    def __init__(self, container, controller: CommandXboxController, indent=0) -> None:
        super().__init__()
        self.setName('CalibrateJoystick')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.swerve = self.container.swerve
        self.controller = controller
        self.count = 0
        self.iterations = 25  # average the measurement over 25 cycles
        self.thrust_array = [0] * self.iterations
        self.strafe_array = [0] * self.iterations
        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def runsWhenDisabled(self) -> bool:
        return True

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
        self.count = 0

    def execute(self) -> None:
        y = self.controller.getLeftY()
        y = y if abs(y) < 0.1 else 0  # sanity check - can't calibrate if the stick is in use
        x = self.controller.getLeftX()
        x = x if abs(x) < 0.1 else 0

        self.thrust_array[self.count % self.iterations] = y
        self.strafe_array[self.count % self.iterations] = x
        self.count += 1

    def isFinished(self) -> bool:
        return self.count >= self.iterations

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True

        thrust_offset = sum(self.thrust_array) / self.iterations
        strafe_offset = sum(self.strafe_array) / self.iterations

        self.swerve.thrust_calibration_offset = thrust_offset
        self.swerve.strafe_calibration_offset = strafe_offset

        msg = f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s with offsets Y: {thrust_offset:.3f} and X: {strafe_offset:.3f} **"
        if print_end_message:
            print(msg)
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
