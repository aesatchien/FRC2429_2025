
import math
import typing
import commands2
import wpilib

from subsystems.swerve import Swerve  # allows us to access the definitions
from wpilib import SmartDashboard
from commands2.button import CommandXboxController
from wpimath.geometry import Rotation2d, Transform2d, Translation2d
from wpimath.filter import Debouncer, SlewRateLimiter
from subsystems.swerve_constants import DriveConstants as dc


class DriveByJoystickSwerve(commands2.Command):
    def __init__(self, container, swerve: Swerve, controller: CommandXboxController, rate_limited=False) -> None:
        # def calculateratelimitedoutvelocity(commanded velocity, actual velocity, deltatime, max allowed accel):
            # if |(commanded - actual / deltatime)| > max allowed accel:
                # return vector with correct direction
            # else:
                # return commanded vector

        super().__init__()
        self.setName('drive_by_joystick_swerve')
        self.container = container
        self.swerve = swerve
        self.field_oriented = True
        self.rate_limited = rate_limited
        self.addRequirements(*[self.swerve])
        # can't import container and don't want to pass lambdas just yet
        self.controller: CommandXboxController = controller
        # self.slow_mode_trigger = self.controller.rightBumper()
        self.robot_oriented_trigger = self.controller.leftBumper()
        # self.robot_oriented_trigger = self.controller.povUp().or_(
        #                               self.controller.povDown().or_(
        #                               self.controller.povLeft().or_(
        #                               self.controller.povRight()
        #                               )))
        self.debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        self.robot_oriented_debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)

        # CJH added a slew rate limiter 20250311 - but there already is one in Swerve, so is this redundant?
        # make sure you put it on the joystick (not calculations), otherwise it doesn't help much on slow-mode
        stick_max_units_per_second = 3  # can't be too low or you get lag - probably should be between 3 and 5
        self.drive_limiter = SlewRateLimiter(stick_max_units_per_second)
        self.strafe_limiter = SlewRateLimiter(stick_max_units_per_second)

        self.prev_commanded_vector = Translation2d(0, 0) # we should start stationary so this should be valid

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"** Started {self.getName()} at {self.start_time} s **" + "\n", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

    def execute(self) -> None:

        # do not call the controller, call its HID - seems to cause overruns
        right_trigger_value = self.controller.getHID().getRightTriggerAxis()  # self.controller.getRightTriggerAxis()
        robot_oriented_value =  self.controller.getHID().getLeftBumperButton()  # self.robot_oriented_trigger.getAsBoolean()

        slowmode_multiplier = 0.2 + 0.8 * right_trigger_value
        angular_slowmode_multiplier = 0.5 + 0.5 * right_trigger_value


        if self.robot_oriented_debouncer.calculate(robot_oriented_value):
            self.field_oriented = False
        else:
            self.field_oriented = True

        # note that swerve's x direction is up/down on the left stick.  (normally think of this as y)
        # according to the templates, these are all multiplied by -1
        # SO IF IT DOES NOT DRIVE CORRECTLY THAT WAY, CHECK KINEMATICS, THEN INVERSION OF DRIVE/ TURNING MOTORS
        # not all swerves are the same - some require inversion of drive and or turn motors

        # can't call the controller.function - apparently that leads to overruns.  use getHID instead
        left_y = self.controller.getHID().getLeftY()
        left_x = self.controller.getHID().getLeftX()

        # CJH added a rate limiter on the joystick - my help with jitter at low end 20250311
        joystick_fwd = -(left_y - self.swerve.thrust_calibration_offset)
        joystick_strafe = -(left_x - self.swerve.strafe_calibration_offset)

        right_x = self.controller.getHID().getRightX()  # don't use  self.controller.getRightX()
        joystick_rot = - right_x # TODO: find why this had to be negated this year (2025)
        if abs(joystick_rot) < dc.k_inner_deadband: joystick_rot = 0

        desired_vector = Translation2d(joystick_fwd, joystick_strafe)  # duty cycle, not meters per second

        trace = True  # CJH watching the joystick values so we can plot them - need to get AJ better low end control
        if trace and wpilib.RobotBase.isSimulation():
            wpilib.SmartDashboard.putNumber('_js_dv1_x', math.fabs(desired_vector.X()))
            wpilib.SmartDashboard.putNumber('_js_dv1_y', math.fabs(desired_vector.Y()))

        # clipping should happen first (the below if statement is clipping)
        # we clip rotation above
        if desired_vector.norm() > dc.k_outer_deadband:
            desired_vector *= 1 / desired_vector.norm()
        elif desired_vector.norm() < dc.k_inner_deadband:
            desired_vector = Translation2d(0, 0)

        # squaring
        # desired_vector *= desired_vector.norm()
        # CJH added a 1.5 instead of 2.0 - may help with stuttering at low end 20250311
        desired_vector *= math.sqrt(desired_vector.norm())

        desired_vector *= slowmode_multiplier # must happen after clipping since we only want to clip the joystick values, not the robot speed
        desired_rot = joystick_rot * angular_slowmode_multiplier

        desired_fwd = desired_vector.X()
        desired_strafe = desired_vector.Y()

        # should come after everything else
        # see if we want to put the slewratelimiter on the trigger because changing directions isn't as bad as changing speed
        # and the trigger changes speed whereas the joystick often changes direction more than speed (??)
        desired_fwd = self.drive_limiter.calculate(desired_fwd)
        desired_strafe = self.strafe_limiter.calculate(desired_strafe)

        if trace and wpilib.RobotBase.isSimulation():
            wpilib.SmartDashboard.putNumber('_js_dv_norm_x', math.fabs(desired_fwd))
            wpilib.SmartDashboard.putNumber('_js_dv_norm_y', math.fabs(desired_strafe))
            SmartDashboard.putNumberArray('commanded values', [desired_fwd, desired_strafe, desired_rot])

        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed and self.field_oriented:
            # Since our angle is now always 0 when facing away from blue driver station, we have to appropriately reverse translation commands
            # print("inverting forward and strafe because we're in field-centric mode and on red alliance!")
            desired_fwd *= -1
            desired_strafe *= -1

        self.swerve.drive(xSpeed=desired_fwd,ySpeed=desired_strafe, rot=desired_rot,
                              fieldRelative=self.field_oriented, rate_limited=self.rate_limited, keep_angle=False)


    def end(self, interrupted: bool) -> None:
        # probably should leave the wheels where they are?
        self.swerve.drive(0, 0, 0, fieldRelative=self.field_oriented, rate_limited=True)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'

        print_end_message = False  # only need to print this for debugging, because it only ever gets interrupted
        if print_end_message:
            print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
            SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

