import commands2
from pathplannerlib.util import translation2dFromJson
from wpilib import SmartDashboard
from wpimath.units import inchesToMeters, degreesToRadians

from subsystems.elevator import Elevator
from subsystems.pivot import Pivot
from subsystems.wrist import Wrist
from subsystems.intake import Intake
import trajectory
from trajectory import CustomTrajectory
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
            self.trajectory: CustomTrajectory = trajectory.trajectory_L3
        else:
            self.trajectory: CustomTrajectory = current_trajectory
        self.waypoint_counter = 0
        self.has_invalid_keys = False

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = self.container.get_enabled_time()
        self.waypoint_counter = 0

        self.has_invalid_keys = self.container.robot_state.get_target().value['name'] not in list(trajectory.score_waypoint_dict.keys())

        print(f"{self.indent * '    '}** Started {self.getName()}  at {self.start_time:.1f} s **", flush=True)

    def execute(self) -> None:
        if self.has_invalid_keys:
            pass

        else:
            # get how long we are into the command
            self.command_time = self.container.get_enabled_time() - self.start_time
            # get the trajectory positions
            targets = self.trajectory.get_value(self.command_time)
            # move all subsystems to the new target
            self.elevator.set_goal(targets['elevator'])
            self.pivot.set_goal(degreesToRadians(targets['pivot']))
            self.wrist.set_position(degreesToRadians(targets['wrist']))
            self.intake.set_reference(targets['intake'])

            # report progress
            waypoint_list = list(self.trajectory.waypoints.keys())  # make this part of the class
            if self.command_time > waypoint_list[self.waypoint_counter]:
                print(f'{"  " + " " * self.indent}starting waypoint {self.waypoint_counter}: {self.trajectory.waypoints[waypoint_list[self.waypoint_counter]]} at {self.container.get_enabled_time():.1f}')
                self.waypoint_counter += 1

    def isFinished(self) -> bool:
        if self.has_invalid_keys:
            return True
        elif self.wait_to_finish:
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
