import commands2
from commands2.button.trigger import Trigger
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d

from subsystems.swerve_constants import AutoConstants as ac
from subsystems.swerve import Swerve


class PIDToPoint(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve: Swerve, target_pose: Pose2d, indent=0) -> None:
        super().__init__()
        self.setName('PID to point')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.swerve = swerve

        self.target_pose = target_pose
        self.target_state = PathPlannerTrajectoryState()
        self.target_state.pose = self.target_pose  # set the pose of the target state
        self.target_state.heading = self.target_pose.rotation()

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

    def execute(self) -> None:
        robot_pose = self.swerve.get_pose()
        target_chassis_speeds = ac.k_pathplanner_holonomic_controller.calculateRobotRelativeSpeeds(robot_pose, self.target_state)

        print(f"commanding omega {target_chassis_speeds.omega}")

        self.swerve.drive_robot_relative(target_chassis_speeds, "we don't use feedforwards")

    def isFinished(self) -> bool:
        diff = self.swerve.get_pose().relativeTo(self.target_pose)
        rotation_achieved = abs(diff.rotation().degrees()) < ac.k_rotation_tolerance.degrees()
        translation_achieved = diff.translation().norm() < ac.k_translation_tolerance_meters
        return rotation_achieved and translation_achieved

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

