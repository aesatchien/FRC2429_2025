import commands2
from wpilib import SmartDashboard
from wpimath.units import inchesToMeters

from subsystems.elevator import Elevator
from subsystems.robot_state import RobotState

class MoveElevator(commands2.Command):  # change the name for your command

    def __init__(self, container, elevator: Elevator, mode='scoring', height=inchesToMeters(8), use_dash=True, offset=0, wait_to_finish=False, indent=0) -> None:
        super().__init__()
        self.setName('Move Elevator')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.elevator = elevator
        self.mode = mode
        self.height = height
        self.use_dash = use_dash  # testing mode - read target from dashboard?
        self.offset = offset  # attempt to have an offset
        self.wait_to_finish = wait_to_finish
        self.addRequirements(self.elevator)  # commandsv2 version of requirements
        SmartDashboard.putNumber('elevator_cmd_goal', 0.21)  # initialize the key we will use to run this command
        SmartDashboard.putString('elevator_cmd_mode', 'absolute')

        # sick of IDE complaining
        self.start_time = None
        self.goal = None

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)

        # a little bit complicated because I want to test everything here
        if self.mode == 'scoring':  # what will eventually be the norm
            self.goal = self.container.robot_state.get_elevator_goal() + self.offset
            self.elevator.set_goal(self.goal)
        elif self.mode == 'specified':  # send to a specific height
            self.goal = self.height
            self.elevator.set_goal(self.goal)
        elif self.mode == 'incremental':  # call from GUI to increment up and down
            self.goal = self.height  # height is a delta in this case
            self.elevator.move_meters(delta_meters=self.goal)
        elif self.use_dash:
            self.goal = SmartDashboard.getNumber('elevator_cmd_goal', 0.21)  # get the elevator sp from the dash
            self.goal = self.goal + self.offset  # allow for an offset from our goal
            self.mode = SmartDashboard.getString('elevator_cmd_mode', 'absolute')
            if self.mode == 'absolute':
                self.elevator.set_goal(self.goal)
            elif self.mode == 'relative':
                self.elevator.move_meters(delta_meters=self.goal)
        else:
            print(f'Invalid Elevator move mode: {self.mode}')

        print(f"{self.indent * '    '}** Started {self.getName()} with mode {self.mode} and goal {self.goal:.2f} at {self.start_time} s **", flush=True)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return self.elevator.get_at_goal() # TODO - put in a timeout, and probably a minimum time to allow to start moving
        else:
            return True


    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
