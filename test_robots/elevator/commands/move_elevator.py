import commands2
from wpilib import SmartDashboard
from subsystems.elevator import Elevator

class MoveElevator(commands2.Command):  # change the name for your command

    def __init__(self, container, elevator:Elevator, mode='absoltue', indent=0) -> None:
        super().__init__()
        self.setName('Move Elevator')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.elevator = elevator
        self.mode = mode
        # self.addRequirements(self.container.)  # commandsv2 version of requirements
        SmartDashboard.putNumber('elevator_cmd_goal', 0.21)  # initialize the key we will use to run this command
        SmartDashboard.putString('elevator_cmd_mode', 'absolute')

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)

        self.goal = SmartDashboard.getNumber('elevator_cmd_goal', 0.21)  # get the elevator sp from the dash
        self.mode = SmartDashboard.getString('elevator_cmd_mode', 'absolute')

        if self.mode == 'absolute':
            self.elevator.set_goal(self.goal)
        else:
            self.elevator.move_meters(delta_meters=self.goal)

        print(f"{self.indent * '    '}** Started {self.getName()} with mode {self.mode} and goal {self.goal:.2f} at {self.start_time} s **", flush=True)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = False
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
