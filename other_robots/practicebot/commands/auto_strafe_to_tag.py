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
from subsystems.vision import Vision
from commands.log_command import log_command


@log_command(console=True, nt=False, print_init=True, print_end=True)
class AutoStrafeToTag(commands2.Command):  #

    def __init__(self, container, swerve: Swerve, location='center', hug_reef=True, trapezoid=False, indent=0) -> None:
        """
        Set a setpoint as a fraction of the screen width for your setpoint for the apriltag
        e.g. 0.5 will try to center the tag in the camera
        if you are using for reefscape, determine the pixel location of the tag when you are aligned left and right
        and use them
        """
        super().__init__()
        self.setName('AutoStrafeToTag')  # using the pathplanner controller instead
        self.indent = indent
        self.container = container
        self.swerve = swerve
        self.vision:Vision = self.container.vision
        self.counter = 0
        self.lost_tag_counter = 0
        self.location = location
        self.camera_setpoint = 0.5 # default value
        self.tolerance = 0.01  # fraction of the cameraFoV
        self.tolerance_counter = 0
        self.hug_reef = hug_reef  # if we want to keep pushing against reef the whole time

        # CJH added a slew rate limiter 20250323 - it jolts and browns out the robot if it servos to full speed
        max_units_per_second = 2  # can't be too low or you get lag and we allow a max of < 50% below
        self.x_limiter = SlewRateLimiter(max_units_per_second)
        self.y_limiter = SlewRateLimiter(max_units_per_second)

        self.trapezoid = trapezoid
        self.x_overshot = False
        self.y_overshot = False
        self.rot_overshot = False
        self.last_diff_x = 999  # needs to start bigly

        self.addRequirements(self.swerve)
        self.reset_controllers()

    def reset_controllers(self):

        # reset the smoothing objects
        self.last_diff_x = 999  # needs to start bigly
        self.x_limiter.reset(0)

        # get the camera fraction setpoint from a list
        if self.location == 'center':
            self.camera_setpoint = 0.6  # TODO - figure out where the actual center is
        elif self.location == 'left':  # we want to be on the left of the tag, so it is past center in image
            self.camera_setpoint = 0.788  # 0.788 against reef, lower 2" back
        elif self.location == 'right':
            self.camera_setpoint = 0.448  # 0.448 against reef, 0.03 lower 2" back
        else:
            raise ValueError(f"Location must be in [center, left, right] - not {self.location}.")

        # grab a rotation target that we need to maintain
        nearest_tag = self.container.swerve.get_nearest_tag(destination='reef')
        self.container.robot_state.set_reef_goal_by_tag(nearest_tag)
        self.target_pose = self.container.robot_state.get_reef_goal_pose()
        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
            self.target_pose = self.target_pose.rotateAround(point=Translation2d(17.548 / 2, 8.062 / 2),
                                                             rot=Rotation2d(math.pi))

        # this is a hybid - X goes to the tag center but y and rotation are based on pose
        # trying to get it to slow down but still make it to final position
        self.x_pid = PIDController(0.5, 0.00, 0.0)
        self.x_pid.setSetpoint(self.camera_setpoint)

        self.y_pid = PIDController(0.3, 0.00, 0.0)
        self.y_pid.setSetpoint(self.target_pose.Y())

        self.rot_pid = PIDController(0.5, 0, 0,)  # 0.5
        self.rot_pid.enableContinuousInput(radians(-180), radians(180))
        self.rot_pid.setSetpoint(self.target_pose.rotation().radians())


        self.x_pid.reset()
        # self.y_pid.reset()
        self.rot_pid.reset()

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.timer.get(), 2)
        msg = f"{self.indent * '    '}** Started {self.getName()} to {self.location} at {self.start_time} s **"
        print(msg, flush=True)
        SmartDashboard.putString("alert", msg)

        self.reset_controllers()  # this is supposed to get us a new pose

        # let the robot know what we're up to
        self.container.led.set_indicator(Led.Indicator.kPOLKA)
        self.counter = 0
        self.lost_tag_counter = 0
        self.tolerance_counter = 0

        self.x_overshot = False
        self.y_overshot = False
        self.rot_overshot = False

        if wpilib.RobotBase.isSimulation():
            msg = f'CNT  CSP STRAFE DIFF OUT '
            print(msg)


    def execute(self) -> None:

        self.counter += 1  # better to do this at the top

        # use the pose as secondary - know our rotation and our position
        robot_pose = self.swerve.get_pose()

        # get_tag_strafe returns zero if no tag, else fraction of camera where tag is located
        current_strafe = self.vision.get_tag_strafe(target='genius_low_tags')
        if current_strafe <= 0.001:
            self.lost_tag_counter += 1
            self.swerve.drive(0, 0, 0, fieldRelative=False, rate_limited=False, keep_angle=False)
            return  # just do nothing if we have no tag except increment the lost tag counter

        if True:  #
            diff_x = self.camera_setpoint - current_strafe  # this is the error, as a fraction of the x FoV
            abs_error = abs(diff_x)
            raw_output = self.x_pid.calculate(current_strafe)
            x_output = raw_output  # we’ll clamp this

            # keep track of how long we've been good - allow to recover if we overshoot
            if abs_error < self.tolerance:
                self.tolerance_counter += 1
            else:
                self.tolerance_counter = 0

            # TODO optimize the last mile and have it gracefully not oscillate
            #rot_max, rot_min = 0.8, 0.2
            trans_max = 0.25  # max strafe power
            trans_min = 0.05  # hard lower bound to overcome static friction

            # the simple way doesn't work if we overshoot too quickly, error increasing means we passed the target
            # self.x_overshot = True if math.fabs(diff_x) < self.tolerance else self.x_overshot
            if abs(diff_x) > abs(self.last_diff_x) and self.counter > 2:
                self.x_overshot = True
            self.last_diff_x = diff_x

            # Before overshoot
            if not self.x_overshot:
                # Enforce minimum based on distance from target
                scaled_min = max(trans_min, 0.1 * abs(diff_x))
                if abs(x_output) < scaled_min:
                    x_output = math.copysign(scaled_min, x_output)
            # After overshoot
            else:
                # If still far and output is too small, nudge with trans_min
                if abs(diff_x) > self.tolerance and abs(x_output) < trans_min:
                    x_output = math.copysign(trans_min, x_output)
                # If within tolerance, let it decay
                # No modification needed

            # enforce maximum values
            # x_output = x_output if math.fabs(x_output) < trans_max else math.copysign(trans_max, x_output)
            # same thing, maybe cleaner
            x_output = max(-trans_max, min(trans_max, x_output))
            # smooth out the initial jumps with slew_limiters
            x_output = self.x_limiter.calculate(x_output)

            # add a y_output to hug the reef, else maintain rotation and y position
            if self.hug_reef:
                y_output = -0.04
                rot_output = 0
            else:
                y_output = self.y_pid.calculate(robot_pose.Y())
                rot_output = self.rot_pid.calculate(robot_pose.rotation().radians())


            # robot battery is front and on the left when scoring, so + x takes you left
            self.swerve.drive(x_output, y_output, rot_output, fieldRelative=False, rate_limited=False, keep_angle=False)

            if self.counter % 5 == 0 and wpilib.RobotBase.isSimulation():
                msg = f'{self.counter}  {self.camera_setpoint:.2f} {current_strafe:.2f} {diff_x:.2f}  {x_output:.2f}  '
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

    def isFinished(self) -> bool:
        diff_x = self.camera_setpoint - self.vision.get_tag_strafe(target='genius_low_tags')  # this is the error, as a fraction of the x FoV
        translation_achieved = math.fabs(diff_x) < self.tolerance  # get to within an inch
        return self.tolerance_counter > 10 or self.lost_tag_counter > 4  # give it 0.2s to be in tolerance four missing tags and we quit

    def end(self, interrupted: bool) -> None:
        end_time = self.container.timer.get()
        end_message = 'Interrupted' if interrupted else 'Ended'
        if interrupted or self.lost_tag_counter > 4:  # TOD0) - use a different indicator for a lost tags
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

