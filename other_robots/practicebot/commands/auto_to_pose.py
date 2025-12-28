from math import radians
import math
import commands2
from pathplannerlib.trajectory import PathPlannerTrajectoryState
from pathplannerlib.util import DriveFeedforwards
import wpilib
from wpimath.controller import PIDController, ProfiledPIDController
from wpimath.geometry import Pose2d, Translation2d, Rotation2d
from wpimath.trajectory import TrapezoidProfile
from wpimath.filter import SlewRateLimiter

from subsystems.swerve_constants import AutoConstants as ac
from subsystems.swerve import Swerve
from subsystems.led import Led
from helpers.log_command import log_command
from helpers.utilities import get_nearest_tag


@log_command(console=True, nt=False, print_init=True, print_end=True)
class AutoToPose(commands2.Command):  #

    def __init__(self, container, swerve: Swerve, target_pose: Pose2d, nearest=False, from_robot_state=False, control_type='not_pathplanner', trapezoid=False, indent=0) -> None:
        """
        if nearest, it overrides target_pose
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
        self.last_diff_x = 999
        self.last_diff_y = 999
        self.last_diff_radians = 1

        self.addRequirements(self.swerve)
        self.reset_controllers()

    def reset_controllers(self):

        # if we want to run this on the fly, we need to pass it a pose
        if self.nearest:
            current_pose = self.container.swerve.get_pose()
            nearest_tag = get_nearest_tag(current_pose=current_pose, destination='reef')
            self.container.robot_state.set_reef_goal_by_tag(nearest_tag)
            self.target_pose = self.container.robot_state.get_reef_goal_pose()
        elif self.from_robot_state:
            self.target_pose = self.container.robot_state.get_reef_goal_pose()
        else:
            pass  # already set in __init__

        if wpilib.DriverStation.getAlliance()  == wpilib.DriverStation.Alliance.kRed:
            self.target_pose = self.target_pose.rotateAround(point=Translation2d(17.548 / 2, 8.062 / 2), rot=Rotation2d(math.pi))

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
                # after 1s at 1 error the integral contribution will be Ki - so 1s at 0.1 error will output 0.1 * Ki
                self.x_pid = PIDController(0.8, 0.1, 0.0)  # can allow for a higher Ki because of clamping below
                self.x_pid.setIntegratorRange(-0.1,0.1)  # clamp Ki * Tot Error min(negative) and max output
                self.x_pid.setIZone(0.25)  # do not allow integral unless we are within 0.25m (prevents windup)
                self.x_pid.setSetpoint(self.target_pose.X())

                self.y_pid = PIDController(0.8, 0.1, 0.0)
                self.y_pid.setIntegratorRange(-0.1,0.1)  # clamp min(negative) and max output of the integral term
                self.y_pid.setIZone(0.25)  # do not allow integral unless we are within 0.25m (prevents windup)
                self.y_pid.setSetpoint(self.target_pose.Y())

            self.rot_pid = PIDController(0.7, 0.0, 0,)  # 0.5
            self.rot_pid.enableContinuousInput(radians(-180), radians(180))
            self.rot_pid.setSetpoint(self.target_pose.rotation().radians())

            # reset overshoot
            self.x_overshot = False
            self.y_overshot = False
            self.rot_overshot = False
            self.last_diff_x = 99  # has to start out a large number
            self.last_diff_y = 99
            self.last_diff_radians = 9
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
        self.start_time = round(self.container.timer.get(), 2)
        self.reset_controllers()  # this is supposed to get us a new pose and reset all the parameters

        # let the robot know what we're up to
        self.container.led.set_indicator(Led.Indicator.kPOLKA)

        self.extra_log_info = f'target {self.target_pose}'


    def execute(self) -> None:
        # moved here because of the auto-logger
        if self.counter == 0 and (wpilib.RobotBase.isSimulation() or self.print_debug):
            msg = f'CNT  DX     XT?   Xo   |   DY    YT?   Yo   |   DR     RT?   Ro   | TC'
            print(msg)

        # we could also do this with wpilib pidcontrollers
        robot_pose = self.swerve.get_pose()

        if self.control_type == 'pathplanner':
            target_chassis_speeds = ac.k_pathplanner_holonomic_controller.calculateRobotRelativeSpeeds(
                current_pose=robot_pose, target_state=self.target_state_flipped)
            self.swerve.drive_robot_relative(target_chassis_speeds, "we don't use feedforwards")

        else:
            x_output = self.x_pid.calculate(robot_pose.X())  # setpoint is the target pose X
            y_output = self.y_pid.calculate(robot_pose.Y())  # setpoint is the target pose Y
            rot_output = self.rot_pid.calculate(robot_pose.rotation().radians())  # setpoint is the target pose radians

            # TODO optimize the last mile and have it gracefully not oscillate
            rot_max, rot_min = 0.5, 0.1
            trans_max, trans_min = 0.3, 0.1  # it browns out when you start if this is too high
            diff_pose = robot_pose.relativeTo(self.target_pose)  # this is pretty much useless for x and y, only rotation
            # this rotateby is important - otherwise you have x and y mixed up when pointed 90 degrees and using robot centric
            diff_xy = diff_pose.rotateBy(-self.target_pose.rotation())  # now dX and dY should be correct if you use .X and .Y
            diff_rot = diff_pose
            # enforce minimum values , but try to stop oscillations  i.e. try to get us to overshoot but very gently
            diff_x = self.target_pose.X() - robot_pose.X()  # x error
            if abs(diff_x) > abs(self.last_diff_x) and self.counter > 0:
                self.x_overshot = True
            self.last_diff_x = diff_x
            diff_y = self.target_pose.Y() - robot_pose.Y()  # y error
            if abs(diff_y) > abs(self.last_diff_y) and self.counter > 0:
                self.y_overshot = True
            self.last_diff_y = diff_y
            diff_radians = diff_rot.rotation().radians()  # rot error - could also use self.target_pose.rotation().radians() - robot_pose.rotation().radians()
            if abs(diff_radians) > abs(self.last_diff_radians) and self.counter > 0:
                self.rot_overshot = True
            self.last_diff_radians = diff_radians

            if abs(x_output) < trans_min and not self.x_overshot and abs(diff_x) > ac.k_translation_tolerance_meters:
                x_output = math.copysign(trans_min, x_output)
            if abs(y_output) < trans_min and not self.y_overshot and abs(diff_y) > ac.k_translation_tolerance_meters:
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
            # I finally tracked down the initial error to the keep_angle - if it is True it has a rotation kink the first time
            self.swerve.drive(x_output, y_output, rot_output, fieldRelative=True, rate_limited=False, keep_angle=False)

            # keep track of how long we've been good - allow to recover if we overshoot
            rotation_achieved = abs(diff_pose.rotation().degrees()) < ac.k_rotation_tolerance.degrees()   # really push it - less than a degree
            translation_achieved = diff_pose.translation().norm() < ac.k_translation_tolerance_meters   # get to within an inch total
            if rotation_achieved and translation_achieved:
                self.tolerance_counter += 1
            else:
                self.tolerance_counter = 0

            if self.counter % 10 == 0 and (wpilib.RobotBase.isSimulation() or self.print_debug):
                msg = f'{self.counter:3d}  {diff_x:+.2f} {str(self.x_overshot):>5} {x_output:+.2f} | {diff_y:+.2f}  {str(self.y_overshot):>5} {y_output:+.2f} '
                msg += f'| {diff_rot.rotation().degrees():>+6.1f}° {str(self.rot_overshot):>5} {rot_output:+.2f} | {self.tolerance_counter} '  # {diff_pose} {diff_xy}'
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
        if interrupted:
            commands2.CommandScheduler.getInstance().schedule(
                self.container.led.set_indicator_with_timeout(Led.Indicator.kFAILUREFLASH, 2))
        else:
            commands2.CommandScheduler.getInstance().schedule(
                self.container.led.set_indicator_with_timeout(Led.Indicator.kSUCCESSFLASH, 2))


