from math import radians
import math
import commands2
from wpilib import SmartDashboard
from wpimath.controller import PIDController
from wpimath.geometry import Pose2d

from subsystems.swerve_constants import AutoConstants as ac
from subsystems.swerve import Swerve
from subsystems.led import Led
from helpers.decorators import log_command


@log_command(console=True, nt=False, print_init=True, print_end=True)
class PIDToPoint(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve: Swerve, target_pose: Pose2d, control_type=None, indent=0) -> None:
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
        self.rot_pid.setSetpoint(target_pose.rotation().radians())

        SmartDashboard.putNumber("x commanded", 0)
        SmartDashboard.putNumber("y commanded", 0)
        SmartDashboard.putNumber("rot commanded", 0)

        self.addRequirements(self.swerve)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""

        self.extra_log_info = f'to {self.target_pose}'

        self.x_pid.reset()
        self.y_pid.reset()
        self.rot_pid.reset()

        self.container.led.set_indicator(Led.Indicator.kPOLKA)

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

        self.swerve.drive(x_setpoint, y_setpoint, rot_setpoint, fieldRelative=True, rate_limited=False, keep_angle=True)

    def isFinished(self) -> bool:
        diff = self.swerve.get_pose().relativeTo(self.target_pose)
        rotation_achieved = abs(diff.rotation().degrees()) < ac.k_rotation_tolerance.degrees()
        translation_achieved = diff.translation().norm() < ac.k_translation_tolerance_meters
        return rotation_achieved and translation_achieved

    def end(self, interrupted: bool) -> None:
        if interrupted:
            commands2.CommandScheduler.getInstance().schedule(
                self.container.led.set_indicator_with_timeout(Led.Indicator.kFAILUREFLASH, 2))
        else:
            commands2.CommandScheduler.getInstance().schedule(
                self.container.led.set_indicator_with_timeout(Led.Indicator.kSUCCESSFLASH, 2))


