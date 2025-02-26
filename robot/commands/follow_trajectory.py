import commands2
from pathplannerlib.util import translation2dFromJson
from wpilib import SmartDashboard
from wpimath.units import inchesToMeters, degreesToRadians

from subsystems.elevator import Elevator
from subsystems.pivot import Pivot
from subsystems.wrist import Wrist
from subsystems.intake import Intake
import trajectory
from subsystems.robot_state import RobotState

class FollowTrajectory(commands2.Command):  # change the name for your command

    def __init__(self, container, current_trajectory=None, wait_to_finish=True, indent=0) -> None:
        super().__init__()
        self.setName('Follow Trajectory')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.elevator: Elevator = container.elevator
        self.pivot: Pivot = container.pivot
        self.wrist: Wrist = container.wrist
        self.intake: Intake = container.intake
        self.wait_to_finish = wait_to_finish
        self.addRequirements(self.elevator, self.pivot, self.wrist, self.intake)  # commandsv2 version of requirements
        # sick of IDE complaining
        self.start_time = None
        self.command_time = 0
        if current_trajectory is None:
            self.trajectory = trajectory.trajectory_L3
        else:
            self.trajectory = current_trajectory

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = self.container.get_enabled_time()

        print(f"{self.indent * '    '}** Started {self.getName()}  at {self.start_time:.1f} s **", flush=True)

    def execute(self) -> None:
        # get how long we are into the command
        self.command_time = self.container.get_enabled_time() - self.start_time
        # get the trajectory positions
        targets = self.trajectory.get_value(self.command_time)
        # move all subsystems to the new target
        self.elevator.set_goal(targets['elevator'])
        self.pivot.set_goal(degreesToRadians(targets['pivot']))
        self.wrist.set_position(degreesToRadians(targets['wrist']))
        self.intake.set_reference(targets['intake'])

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            return self.command_time >= self.trajectory.time_steps[-1]
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
