import commands2
from wpilib import SmartDashboard
from constants import ShoulderConstants
from subsystems.pivot import Pivot
from wpimath.units import inchesToMeters, radiansToDegrees, degreesToRadians
from subsystems.robot_state import RobotState


class MovePivot(commands2.Command):  # change the name for your command

    def __init__(self, container, pivot: Pivot,  mode='scoring', angle=degreesToRadians(90), use_dash=True, wait_to_finish=False, indent=0) -> None:
        """
        note: relative mode doesn't support wait to finish (it will never finish)
        """
        super().__init__()
        self.setName('Move Pivot')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.pivot = pivot
        self.mode = mode
        self.angle = angle
        self.use_dash = use_dash  # testing mode - read target from dashboard?
        self.wait_to_finish = wait_to_finish
        self.addRequirements(self.pivot)  # commandsv2 version of requirements
        SmartDashboard.putNumber('pivot_cmd_goal_deg', 90)  # initialize the key we will use to run this command
        SmartDashboard.putString('pivot_cmd_mode', 'absolute')

        # sick of IDE complaining
        self.start_time = None
        self.goal = None

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)

        # a little bit complicated because I want to test everything here
        if self.mode == 'scoring':  # what will eventually be the norm
            self.goal = self.container.robot_state.get_pivot_goal()
            self.pivot.set_goal(self.goal)  # should already be in radians
        elif self.mode == 'specified':
            self.goal = self.angle
            self.pivot.set_goal(self.goal)  # should already be in radians
        elif self.use_dash:
            self.goal = SmartDashboard.getNumber('pivot_cmd_goal_deg', 90)  # get the elevator sp from the dash
            self.mode = SmartDashboard.getString('pivot_cmd_mode', 'absolute')
            if self.mode == 'absolute':
                self.pivot.set_goal(degreesToRadians(self.goal))
            elif self.mode == 'relative':
                self.pivot.move_degrees(delta_degrees=self.goal)
        else:
            print(f'Invalid Elevator move mode: {self.mode}')

        print(f"{self.indent * '    '}** Started {self.getName()} with mode {self.mode} and goal {self.goal:.2f} at {self.start_time} s **", flush=True)

    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return abs(self.pivot.get_angle() - self.goal) < ShoulderConstants.k_tolerance
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
