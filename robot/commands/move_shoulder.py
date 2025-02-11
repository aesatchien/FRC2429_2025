import commands2
from wpilib import SmartDashboard
from subsystems.shoulder import Shoulder


class MoveShoulder(commands2.Command):

    def __init__(self, container, shoulder: Shoulder, radians: float, wait_to_finish=False, indent=0) -> None:
        super().__init__()
        self.indent = indent
        self.setName('Move Shoulder')
        self.container = container
        self.shoulder = shoulder
        self.radians = radians
        self.wait_to_finish = wait_to_finish
        self.addRequirements(self.shoulder)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{'    ' * self.indent}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.shoulder.set_position(self.radians)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return self.shoulder.get_at_setpoint()
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = False
        if print_end_message:
            print(f"{'    ' * self.indent}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
