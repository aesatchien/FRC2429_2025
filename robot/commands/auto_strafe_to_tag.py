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


class AutoStrafeToTag(commands2.Command):  #

    def __init__(self, container, swerve: Swerve, location='center', trapezoid=False, indent=0) -> None:
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

        # CJH added a slew rate limiter 20250323 - it jolts and browns out the robot if it servos to full speed
        max_units_per_second = 2  # can't be too low or you get lag and we allow a max of < 50% below
        self.x_limiter = SlewRateLimiter(max_units_per_second)
        self.y_limiter = SlewRateLimiter(max_units_per_second)

        self.trapezoid = trapezoid
        self.x_overshot = False
        self.y_overshot = False
        self.rot_overshot = False

        self.addRequirements(self.swerve)
        self.reset_controllers()

    def reset_controllers(self):

        if self.location == 'center':
            self.camera_setpoint = 0.5
        elif self.location == 'left':  # we want to be on the left of the tag, so it is past center in image
            self.camera_setpoint = 0.788  # 0.788 against reef, lower 2" back
        elif self.location == 'right' :
            self.camera_setpoint = 0.427  # 0.427 against reef, 0.03 lower 2" back
        else:
            raise ValueError(f"Location must be in [center, left, right] - not {self.location}.")

        # if we want to run this on the fly, we need to pass it a location
        if self.trapezoid:  # use a trapezoidal profile
            xy_constraints = TrapezoidProfile.Constraints(maxVelocity=2, maxAcceleration=1.0)
            self.x_pid = ProfiledPIDController(0.25, 0, 0.05, constraints=xy_constraints)
            self.x_pid.setGoal(self.camera_setpoint)

        else:
            # trying to get it to slow down but still make it to final position
            self.x_pid = PIDController(0.5, 0.00, 0.0)
            self.x_pid.setSetpoint(self.camera_setpoint)

            self.rot_pid = PIDController(0.7, 0, 0,)  # 0.5
            self.rot_pid.enableContinuousInput(radians(-180), radians(180))
            #self.rot_pid.setSetpoint(self.target_pose.rotation().radians())

            #SmartDashboard.putNumber("x commanded", 0)
            #SmartDashboard.putNumber("y commanded", 0)
            #SmartDashboard.putNumber("rot commanded", 0)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} to {self.location} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        self.reset_controllers()  # this is supposed to get us a new pose

        if self.trapezoid:
            self.x_pid.reset(self.location)
            # self.y_pid.reset(robot_pose.Y())
        else:
            self.x_pid.reset()
            # self.y_pid.reset()
            # self.rot_pid.reset()

        # let the robot know what we're up to
        self.container.led.set_indicator(Led.Indicator.kPOLKA)
        self.counter = 0
        self.lost_tag_counter = 0
        self.tolerance_counter = 0

        self.x_overshot = False
        self.y_overshot = False
        self.rot_overshot = False
        msg = f'CNT  CSP STRAFE DIFF OUT '
        print(msg)

    def execute(self) -> None:

        self.counter += 1  # better to do this at the top

        # get_tag_strafe returns zero if no tag, else fraction of camera where tag is located
        current_strafe = self.vision.get_tag_strafe(target='genius_low_tags')
        if current_strafe <= 0.001:
            self.lost_tag_counter += 1
            self.swerve.drive(0, 0, 0, fieldRelative=False, rate_limited=False, keep_angle=False)
            return  # just do nothing if we have no tag except increment the lost tag counter

        if True:  #
            x_output = self.x_pid.calculate(current_strafe)
            #y_output = self.y_pid.calculate(robot_pose.Y())
            #rot_output = self.rot_pid.calculate(robot_pose.rotation().radians())

            # TODO optimize the last mile and have it gracefully not oscillate
            #rot_max, rot_min = 0.8, 0.2
            trans_max, trans_min = 0.25, 0.05  # it browns out when you start if this is too high

            # enforce minimum values , but try to stop oscillations
            diff_x = self.camera_setpoint - current_strafe  # this is the error, as a fraction of the x FoV
            if math.fabs(diff_x) <  self.tolerance:
                self.tolerance_counter += 1
            else:
                self.tolerance_counter = 0


            self.x_overshot = True if math.fabs(diff_x) < self.tolerance else self.x_overshot
            if abs(x_output) < trans_min and not self.x_overshot and abs(diff_x) > self.tolerance:
                x_output = math.copysign(trans_min, x_output)
            # enforce maximum values
            x_output = x_output if math.fabs(x_output) < trans_max else math.copysign(trans_max, x_output)

            # smooth out the initial jumps with slew_limiters
            x_output = self.x_limiter.calculate(x_output)
            # robot battery is front and on the left when scoring, so + x takes you left
            self.swerve.drive(x_output, 0, 0, fieldRelative=False, rate_limited=False, keep_angle=False)

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
        end_time = self.container.get_enabled_time()
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

