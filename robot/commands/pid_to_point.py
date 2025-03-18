from math import radians
import math
import commands2
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from pathplannerlib.util import DriveFeedforwards
from wpilib import SmartDashboard
import wpilib
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d

from subsystems.swerve_constants import AutoConstants as ac
from subsystems.swerve import Swerve


class PIDToPoint(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve: Swerve, target_pose: Pose2d, indent=0) -> None:
        """
        this command handles flipping for red alliance, so only ever pass it things which apply to blue alliance
        """
        super().__init__()
        self.setName('PID to point')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.swerve = swerve

        self.target_pose = target_pose

        self.x_pid = PIDController(1, 0, 0.1)
        self.y_pid = PIDController(1, 0, 0.1)
        self.rot_pid = PIDController(1, 0, 0)
        self.rot_pid.enableContinuousInput(radians(-180), radians(180))
        self.x_pid.setSetpoint(target_pose.X())
        self.y_pid.setSetpoint(target_pose.Y())
        self.y_pid.setSetpoint(target_pose.rotation().radians())

        SmartDashboard.putNumber("x commanded", 0)
        SmartDashboard.putNumber("y commanded", 0)
        SmartDashboard.putNumber("rot commanded", 0)

        self.addRequirements(self.swerve)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} to {self.target_pose} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")
        self.x_pid.reset()
        self.y_pid.reset()
        self.rot_pid.reset()

    def execute(self) -> None:
        # we could also do this with wpilib pidcontrollers
        robot_pose = self.swerve.get_pose()

        x_setpoint = self.x_pid.calculate(robot_pose.X())
        y_setpoint = self.y_pid.calculate(robot_pose.Y())
        rot_setpoint = self.rot_pid.calculate(robot_pose.rotation().radians())

        SmartDashboard.putNumber("x setpoint", self.x_pid.getSetpoint())
        SmartDashboard.putNumber("y setpoint", self.y_pid.getSetpoint())
        SmartDashboard.putNumber("rot setpoint", math.degrees(self.rot_pid.getSetpoint()))

        SmartDashboard.putNumber("x measured", robot_pose.x)
        SmartDashboard.putNumber("y measured", robot_pose.y)
        SmartDashboard.putNumber("rot measured", robot_pose.rotation().degrees())

        SmartDashboard.putNumber("x commanded", x_setpoint)
        SmartDashboard.putNumber("y commanded", y_setpoint)
        SmartDashboard.putNumber("rot commanded", rot_setpoint)

        # positive rot commanded here gives negative results so must negate
        if wpilib.RobotBase.isReal():
            self.swerve.drive(x_setpoint, y_setpoint, -rot_setpoint, fieldRelative=True, rate_limited=False, keep_angle=True)
        else: # yes this is horrible
            self.swerve.drive(x_setpoint, y_setpoint, rot_setpoint, fieldRelative=True, rate_limited=False, keep_angle=True)

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

