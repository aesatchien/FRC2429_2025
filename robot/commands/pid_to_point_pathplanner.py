from math import radians
import math
import commands2
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from pathplannerlib.util import DriveFeedforwards
from wpilib import SmartDashboard
import wpilib
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.geometry import Pose2d
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians

from subsystems.swerve_constants import AutoConstants as ac
from subsystems.swerve import Swerve
from subsystems.led import Led


class PIDToPointPathPlanner(commands2.Command):  # change the name for your command

    def __init__(self, container, swerve: Swerve, target_pose: Pose2d, control_type='pathplanner', trapezoid=True, indent=0) -> None:
        """
        this command handles flipping for red alliance, so only ever pass it things which apply to blue alliance
        """
        super().__init__()
        self.setName('PID to point pathplanner')  # using the pathplanner controller instead
        self.indent = indent
        self.container = container
        self.swerve = swerve
        self.control_type = control_type  # choose between the pathplanner controller or our custom one
        self.counter = 0

        self.target_pose = target_pose
        self.trapezoid = trapezoid

        if self.control_type == 'pathplanner':
            self.target_state = PathPlannerTrajectoryState()
            self.target_state.pose = self.target_pose  # set the pose of the target state
            self.target_state.heading = self.target_pose.rotation()
            self.target_state.feedforwards = DriveFeedforwards()
            self.target_state_flipped = self.target_state

        else:  # custom
            if self.trapezoid:  # use a trapezoidal profile
                xy_constraints = TrapezoidProfile.Constraints(maxVelocity=2, maxAcceleration=3)
                self.x_pid = ProfiledPIDController(1, 0, 0.1, constraints=xy_constraints)
                self.y_pid = ProfiledPIDController(1, 0, 0.1, constraints=xy_constraints)
                self.x_pid.setGoal(target_pose.X())
                self.y_pid.setGoal(target_pose.Y())
            else:
                self.x_pid = PIDController(1, 0, 0.1)
                self.y_pid = PIDController(1, 0, 0.1)
                self.x_pid.setSetpoint(target_pose.X())
                self.y_pid.setSetpoint(target_pose.Y())

            self.rot_pid = PIDController(1, 0, 0)
            self.rot_pid.enableContinuousInput(radians(-180), radians(180))
            self.rot_pid.setSetpoint(target_pose.rotation().radians())

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

        if self.control_type == 'pathplanner':
            if self.swerve.flip_path():  # this is in initialize, not __init__, in case FMS hasn't told us the right alliance on boot-up
                self.target_state_flipped = self.target_state.flip()
            else:
                self.target_state_flipped = self.target_state

        else:
            if self.trapezoid:
                robot_pose = self.swerve.get_pose()
                self.x_pid.reset(robot_pose.X())
                self.y_pid.reset(robot_pose.Y())
            else:
                self.x_pid.reset()
                self.y_pid.reset()
                self.rot_pid.reset()

        # let the robot know what we're up to
        self.container.led.set_indicator(Led.Indicator.kPOLKA)

    def execute(self) -> None:
        # we could also do this with wpilib pidcontrollers
        robot_pose = self.swerve.get_pose()

        if self.control_type == 'pathplanner':
            target_chassis_speeds = ac.k_pathplanner_holonomic_controller.calculateRobotRelativeSpeeds(
                current_pose=robot_pose, target_state=self.target_state_flipped)
            self.swerve.drive_robot_relative(target_chassis_speeds, "we don't use feedforwards")

        else:
            x_setpoint = self.x_pid.calculate(robot_pose.X())
            y_setpoint = self.y_pid.calculate(robot_pose.Y())
            rot_setpoint = self.rot_pid.calculate(robot_pose.rotation().radians())
            self.swerve.drive(x_setpoint, y_setpoint, rot_setpoint, fieldRelative=True, rate_limited=False, keep_angle=True)
            if self.counter % 10 == 0:
                pass
                #SmartDashboard.putNumber("x setpoint", self.x_pid.getSetpoint())
                #SmartDashboard.putNumber("y setpoint", self.y_pid.getSetpoint())
                #SmartDashboard.putNumber("rot setpoint", math.degrees(self.rot_pid.getSetpoint()))
                #SmartDashboard.putNumber("x measured", robot_pose.x)
                #SmartDashboard.putNumber("y measured", robot_pose.y)
                #SmartDashboard.putNumber("rot measured", robot_pose.rotation().degrees())
                #SmartDashboard.putNumber("x commanded", x_setpoint)
                #SmartDashboard.putNumber("y commanded", y_setpoint)
                #SmartDashboard.putNumber("rot commanded", rot_setpoint)



        self.counter += 1

    def isFinished(self) -> bool:
        diff = self.swerve.get_pose().relativeTo(self.target_pose)
        rotation_achieved = abs(diff.rotation().degrees()) < ac.k_rotation_tolerance.degrees()
        translation_achieved = diff.translation().norm() < ac.k_translation_tolerance_meters
        return rotation_achieved and translation_achieved

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        end_message = 'Interrupted' if interrupted else 'Ended'
        if interrupted:
            commands2.CommandScheduler.getInstance().schedule(
                self.container.led.set_indicator_with_timeout(Led.Indicator.kFAILUREFLASH, 2))
        else:
            commands2.CommandScheduler.getInstance().schedule(
                self.container.led.set_indicator_with_timeout(Led.Indicator.kSUCCESSFLASH, 2))

        print_end_message = True
        msg = f"{self.indent * '    '}** {end_message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **"
        if print_end_message:
            print(msg)
            SmartDashboard.putString(f"alert", msg)

