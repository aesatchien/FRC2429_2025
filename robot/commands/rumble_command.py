import commands2
from wpilib import SmartDashboard
from wpilib.interfaces import GenericHID


class RumbleCommand(commands2.Command):  # change the name for your command

    def __init__(self, container, rumble_amount: float, left_rumble: bool, right_rumble: bool, rumble_time=None, indent=0) -> None:
        """
        if you call this without a rumble_time it will start rumbling and not stop till you call it again
        if you call this with a rumble_time it will rumble for that long
        """
        super().__init__()
        self.setName('Rumble command')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container

        self.rumble_amount = rumble_amount

        self.rumble_time = rumble_time

        if left_rumble:
            self.rumble_type = GenericHID.RumbleType.kLeftRumble
        if right_rumble:
            self.rumble_type = GenericHID.RumbleType.kRightRumble
        if left_rumble and right_rumble:
            self.rumble_type = GenericHID.RumbleType.kBothRumble

        if (not left_rumble) and (not right_rumble):
            raise ValueError("why are you making a rumblecommand with no rumble")

        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.container.driver_command_controller.setRumble(self.rumble_type, self.rumble_amount)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.rumble_time:
            return self.container.get_enabled_time() - self.start_time > self.rumble_time
        else:
            return True

    def end(self, interrupted: bool) -> None:

        if self.rumble_time:
            self.container.driver_command_controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
