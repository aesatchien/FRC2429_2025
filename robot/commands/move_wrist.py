import commands2
from wpilib import SmartDashboard
from subsystems.wrist import Wrist


class MoveWrist(commands2.Command):

    def __init__(self, container, wrist: Wrist, radians: float, wait_to_finish=False) -> None:
        super().__init__()
        self.setName('Sample Name')
        self.container = container
        self.wrist = wrist
        self.radians = radians
        self.wait_to_finish = wait_to_finish

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.wrist.set_position(self.radians)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return self.wrist.get_at_setpoint()
        else:
            return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = False
        if print_end_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
