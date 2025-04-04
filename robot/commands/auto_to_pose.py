from math import radians
import math
import commands2
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from pathplannerlib.util import DriveFeedforwards
from wpilib import SmartDashboard
import wpilib
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.trajectory import TrapezoidProfile, TrapezoidProfileRadians
from wpimath.filter import SlewRateLimiter

from subsystems.swerve_constants import AutoConstants as ac
from subsystems.swerve import Swerve
from subsystems.led import Led


class AutoToPose(commands2.Command):  #

    def __init__(self, container, swerve: Swerve, target_pose: Pose2d, nearest=False, from_robot_state=False, control_type='pathplanner', trapezoid=False, indent=0) -> None:
        """
        this command handles flipping for red alliance, so only ever pass it things which apply to blue alliance
        """
        super().__init__()
        self.setName('AutoToPose')  # using the pathplanner controller instead
        self.indent = indent
        self.container = container
        self.swerve = swerve
        self.control_type = control_type  # choose between the pathplanner controller or our custom one
        self.counter = 0
        self.tolerance_counter = 0
        self.from_robot_state = from_robot_state
        self.nearest = nearest  # only use nearest tags as the target
        self.print_debug = True

        # CJH added a slew rate limiter 20250323 - it jolts and browns out the robot if it servos to full speed
        max_units_per_second = 2  # can't be too low or you get lag and we allow a max of < 50% below
        self.x_limiter = SlewRateLimiter(max_units_per_second)
        self.y_limiter = SlewRateLimiter(max_units_per_second)
        self.rot_limiter = SlewRateLimiter(max_units_per_second)

        self.target_pose = target_pose
        self.trapezoid = trapezoid

        # check for overshoot
        self.x_overshot = False
        self.y_overshot = False
        self.rot_overshot = False
        self.last_diff_x = 0
        self.last_diff_y = 0
        self.last_diff_radians = 0

        self.addRequirements(self.swerve)
        self.reset_controllers()

    def reset_controllers(self):

        # if we want to run this on the fly, we need to pass it a pose
        if self.nearest:
            nearest_tag = self.container.swerve.get_nearest_tag(destination='reef')
            self.container.robot_state.set_reef_goal_by_tag(nearest_tag)
            self.target_pose = self.container.robot_state.get_reef_goal_pose()
            if wpilib.DriverStation.getAlliance()  == wpilib.DriverStation.Alliance.kRed:
                self.target_pose = self.target_pose.rotateAround(point=Translation2d(17.548 / 2, 8.062 / 2), rot=Rotation2d(math.pi))

        elif self.from_robot_state:
            self.target_pose = self.container.robot_state.get_reef_goal_pose()
        else:
            pass  # already set in __init__

        if self.control_type == 'pathplanner':
            self.target_state = PathPlannerTrajectoryState()
            self.target_state.pose = self.target_pose  # set the pose of the target state
            self.target_state.heading = self.target_pose.rotation()
            self.target_state.feedforwards = DriveFeedforwards()
            self.target_state_flipped = self.target_state

        else:  # custom
            if self.trapezoid:  # use a trapezoidal profile
                xy_constraints = TrapezoidProfile.Constraints(maxVelocity=2, maxAcceleration=1.0)
                self.x_pid = ProfiledPIDController(0.25, 0, 0.05, constraints=xy_constraints)
                self.y_pid = ProfiledPIDController(0.25, 0, 0.05, constraints=xy_constraints)
                self.x_pid.setGoal(self.target_pose.X())
                self.y_pid.setGoal(self.target_pose.Y())
            else:
                # trying to get it to slow down but still make it to final position
                self.x_pid = PIDController(0.8, 0.01, 0.0)
                self.y_pid = PIDController(0.8, 0.01, 0.0)
                self.x_pid.setSetpoint(self.target_pose.X())
                self.y_pid.setSetpoint(self.target_pose.Y())

            self.rot_pid = PIDController(0.7, 0.01, 0,)  # 0.5
            self.rot_pid.enableContinuousInput(radians(-180), radians(180))
            self.rot_pid.setSetpoint(self.target_pose.rotation().radians())

            # reset overshoot
            self.x_overshot = False
            self.y_overshot = False
            self.rot_overshot = False
            self.last_diff_x = 0
            self.last_diff_y = 0
            self.last_diff_radians = 0
            self.tolerance_counter = 0
            self.rot_limiter.reset(0)
            self.x_limiter.reset(0)
            self.y_limiter.reset(0)
            self.counter = 0

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

            #SmartDashboard.putNumber("x commanded", 0)
            #SmartDashboard.putNumber("y commanded", 0)
            #SmartDashboard.putNumber("rot commanded", 0)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        self.reset_controllers()  # this is supposed to get us a new pose and reset all the parameters

        # let the robot know what we're up to
        self.container.led.set_indicator(Led.Indicator.kPOLKA)
        self.counter = 0

        self.x_overshot = False
        self.y_overshot = False
        self.rot_overshot = False

        msg = f"{self.indent * '    '}** Started {self.getName()} to {self.target_pose} at {self.start_time} s **"
        print(msg, flush=True)
        SmartDashboard.putString("alert", msg)
        if wpilib.RobotBase.isSimulation() or self.print_debug:
            msg = f'CNT  DX  XT?  Xo | DY  YT?  Yo |  DR RT?  Ro  | TC'
            print(msg)

    def execute(self) -> None:
        # we could also do this with wpilib pidcontrollers
        robot_pose = self.swerve.get_pose()

        if self.control_type == 'pathplanner':
            target_chassis_speeds = ac.k_pathplanner_holonomic_controller.calculateRobotRelativeSpeeds(
                current_pose=robot_pose, target_state=self.target_state_flipped)
            self.swerve.drive_robot_relative(target_chassis_speeds, "we don't use feedforwards")

        else:
            x_output = self.x_pid.calculate(robot_pose.X())
            y_output = self.y_pid.calculate(robot_pose.Y())
            rot_output = self.rot_pid.calculate(robot_pose.rotation().radians())

            # TODO optimize the last mile and have it gracefully not oscillate
            rot_max, rot_min = 0.5, 0.1
            trans_max, trans_min = 0.3, 0.1  # it browns out when you start if this is too high
            # this rotateby is important - otherwise you have x and y mixed up when pointed 90 degrees
            pose = self.swerve.get_pose()
            diff_pose = pose.relativeTo(self.target_pose)
            diff_xy = diff_pose.rotateBy(pose.rotation())
            diff_rot = diff_pose
            # enforce minimum values , but try to stop oscillations
            # try to get us to overshoot but very gently
            diff_x = diff_pose.X()
            if abs(diff_x) > abs(self.last_diff_x) and self.counter > 1:
                self.x_overshot = True
            self.last_diff_x = diff_x
            diff_y = diff_pose.Y()
            if abs(diff_y) > abs(self.last_diff_y) and self.counter > 1:
                self.y_overshot = True
            self.last_diff_y = diff_y
            diff_radians = diff_rot.rotation().radians()
            if abs(diff_radians) > abs(self.last_diff_radians) and self.counter > 1:
                self.rot_overshot = True
            self.last_diff_radians = diff_radians

            if abs(x_output) < trans_min and not self.x_overshot and abs(diff_xy.X()) > ac.k_translation_tolerance_meters:
                x_output = math.copysign(trans_min, x_output)
            if abs(y_output) < trans_min and not self.y_overshot and abs(diff_xy.Y()) > ac.k_translation_tolerance_meters:
                y_output = math.copysign(trans_min, y_output)
            if abs(rot_output) < rot_min and not self.rot_overshot and abs(diff_rot.rotation().degrees()) > ac.k_rotation_tolerance.degrees():
                rot_output = math.copysign(rot_min, rot_output)
            # enforce maximum values
            x_output = x_output if math.fabs(x_output) < trans_max else math.copysign(trans_max, x_output)
            y_output = y_output if math.fabs(y_output) < trans_max else math.copysign(trans_max, y_output)
            rot_output = rot_output if math.fabs(rot_output) < rot_max else math.copysign(rot_max, rot_output)

            # smooth out the initial jumps with slew_limiters
            x_output = self.x_limiter.calculate(x_output)
            y_output = self.y_limiter.calculate(y_output)
            rot_output = self.rot_limiter.calculate(rot_output)
            self.swerve.drive(x_output, y_output, rot_output, fieldRelative=True, rate_limited=False, keep_angle=True)

            # keep track of how long we've been good - allow to recover if we overshoot
            rotation_achieved = abs(diff_pose.rotation().degrees()) < ac.k_rotation_tolerance.degrees()   # really push it - less than a degree
            translation_achieved = diff_pose.translation().norm() < ac.k_translation_tolerance_meters   # get to within an inch total
            if rotation_achieved and translation_achieved:
                self.tolerance_counter += 1
            else:
                self.tolerance_counter = 0

            if self.counter % 10 == 0 and (wpilib.RobotBase.isSimulation() or self.print_debug):
                msg = f'{self.counter:3d}  {diff_xy.X():.2f} {self.x_overshot} {x_output:.2f} | {diff_xy.Y():.2f}  {self.y_overshot} {y_output:.2f} | {diff_rot.rotation().degrees():.1f}° {self.rot_overshot} {rot_output:.2f} | {self.tolerance_counter}'
                print(msg)
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
        return self.tolerance_counter > 10

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
        msg = f"{self.indent * '    '}** {end_message} {self.getName()} at {self.swerve.get_pose()} at time {end_time:.1f} s after {end_time - self.start_time:.1f} s **"
        if print_end_message:
            print(msg)
            SmartDashboard.putString(f"alert", msg)

