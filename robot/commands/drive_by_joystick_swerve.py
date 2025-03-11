#  copying 1706's default swerve drive control

import math
import typing
import commands2
from numpy import datetime_as_string
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
        self.controller = controller
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
        stick_max_units_per_second = 5  # can't be too low or you get lag - probably should be between 3 and 5
        self.drive_limiter = SlewRateLimiter(stick_max_units_per_second)
        self.strafe_limiter = SlewRateLimiter(stick_max_units_per_second)

        self.prev_commanded_vector = Translation2d(0, 0) # we should start stationary so this should be valid

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

    def execute(self) -> None:

        slowmode_multiplier = 0.2 + 0.8 * self.controller.getRightTriggerAxis()
        angular_slowmode_multiplier = 0.5 + 0.5 * self.controller.getRightTriggerAxis()

        if self.robot_oriented_debouncer.calculate(self.robot_oriented_trigger.getAsBoolean()):
            self.field_oriented = False
        else:
            self.field_oriented = True

        # note that swerve's x direction is up/down on the left stick.  (normally think of this as y)
        # according to the templates, these are all multiplied by -1
        # SO IF IT DOES NOT DRIVE CORRECTLY THAT WAY, CHECK KINEMATICS, THEN INVERSION OF DRIVE/ TURNING MOTORS
        # not all swerves are the same - some require inversion of drive and or turn motors

        # CJH added a rate limiter on the joystick - my help with jitter at low end 20250311
        joystick_fwd = -(self.controller.getLeftY() - self.swerve.thrust_calibration_offset)
        joystick_fwd = self.drive_limiter.calculate(joystick_fwd)
        joystick_strafe = -(self.controller.getLeftX() - self.swerve.strafe_calibration_offset)
        joystick_strafe = self.strafe_limiter.calculate(joystick_strafe)

        joystick_rot = self.controller.getRightX() # TODO: find why this had to be negated this year (2025)
        if abs(joystick_rot) < dc.k_inner_deadband: joystick_rot = 0

        desired_vector = Translation2d(joystick_fwd, joystick_strafe)  # duty cycle, not meters per second

        trace = True  # CJH watching the joystick values so we can plot them - need to get AJ better low end control
        if trace:
            wpilib.SmartDashboard.putNumber('_js_dv1_x', math.fabs(desired_vector.X()))
            wpilib.SmartDashboard.putNumber('_js_dv1_y', math.fabs(desired_vector.Y()))

        # clipping should happen first (the below if statement is clipping)
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

        if trace:
            wpilib.SmartDashboard.putNumber('_js_dv_norm_x', math.fabs(desired_fwd))
            wpilib.SmartDashboard.putNumber('_js_dv_norm_y', math.fabs(desired_strafe))

        # linear_mapping = True  # two ways to make sure diagonal is at "full speed"
        #                        # TODO: find why our transforms are weird at low speed- they don't act right
        #
        # # apply deadband
        #
        # if linear_mapping:
        #     # this lets you go full speed diagonal at the cost of sensitivity on the low end
        #     desired_fwd = -self.input_transform_linear(1.0 * joystick_fwd) * slowmode_multiplier
        #     desired_strafe = -self.input_transform_linear(1.0 * joystick_strafe) * slowmode_multiplier
        #     desired_rot = -self.input_transform_linear(1.0 * self.controller.getRightX()) * angular_slowmode_multiplier
        # else:
        #     # correcting for the x^2 + y^2 = 1 of the joysticks so we don't have to use a linear mapping
        #     angle = math.atan2(joystick_fwd, joystick_strafe)
        #     correction = math.fabs(math.cos(angle) * (math.sin(angle)))  # peaks at 45 degrees
        #     fwd = joystick_fwd * (1 + 0.3 * math.copysign(correction, joystick_fwd))  # they max out at .77, so add the remainder at 45 degrees to make 1
        #     strafe = joystick_strafe * (1 + 0.3 * math.copysign(correction, joystick_strafe))
        #     if wpilib.RobotBase.isSimulation():
        #         wpilib.SmartDashboard.putNumberArray('joysticks',[joystick_fwd, joystick_strafe, correction, fwd, strafe])
        #
        #     desired_fwd = -self.input_transform(1.0 * fwd) * slowmode_multiplier
        #     desired_strafe = -self.input_transform(1.0 * strafe) * slowmode_multiplier
        #     desired_rot = -self.input_transform(1.0 * self.controller.getRightX()) * angular_slowmode_multiplier

        SmartDashboard.putNumberArray('commanded values', [desired_fwd, desired_strafe, desired_rot])

        if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed and self.field_oriented:
            # Since our angle is now always 0 when facing away from blue driver station, we have to appropriately reverse translation commands
            # print("inverting forward and strafe because we're in field-centric mode and on red alliance!")
            desired_fwd *= -1
            desired_strafe *= -1

        # desired_fwd, desired_strafe = self.calculate_rate_limited_velocity(desired_fwd, desired_strafe, 0.02, 0.5)


        self.swerve.drive(xSpeed=desired_fwd,ySpeed=desired_strafe, rot=desired_rot,
                              fieldRelative=self.field_oriented, rate_limited=self.rate_limited, keep_angle=False)


    def end(self, interrupted: bool) -> None:
        # probably should leave the wheels where they are?
        self.swerve.drive(0, 0, 0, fieldRelative=self.field_oriented, rate_limited=True)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")

    # def apply_deadband(self, value, db_low=dc.k_inner_deadband, db_high=dc.k_outer_deadband):
    #     # put a deadband on the joystick input values here
    #     if abs(value) < db_low:
    #         return 0
    #     elif abs(value) > db_high:
    #         return 1 * math.copysign(1, value)
    #     else:
    #         return value
    #
    # def input_transform(self, value, a=0.9, b=0.1):
    #     # 1706 uses a nice transform
    #     db_value = self.apply_deadband(value)
    #     return a * db_value**3 + b * db_value
    #
    # def input_transform_linear(self, value, a=0.9, b=0.1):
    #     # 1706 uses a nice transform
    #     db_value = self.apply_deadband(value)
    #     return db_value
    #
    # def calculate_rate_limited_velocity(self, commanded_fwd, commanded_strafe, time_since_last_called, max_accel):
    #     """
    #     all in meters/sec^n
    #     BUG: is very very broken and will cook a robot by making it increase velocity indefinitely. DO NOT USE
    #     """
    #
    #     commanded_vector = Translation2d(commanded_fwd, commanded_strafe)
    #     print(f"prev commanded: {self.prev_commanded_vector.X()}, {self.prev_commanded_vector.Y()}")
    #     print(f"     commanded: {commanded_vector.X()}, {commanded_vector.Y()}")
    #     # commanded is (0, 0)
    #     # prev commanded is (0, 4)
    #     # too much accel!
    #     # add (1, 0) to  prev commanded
    #
    #     commanded_accel_vector: Translation2d = (commanded_vector - self.prev_commanded_vector) / time_since_last_called
    #     print("we're being commanded an acceleration of", commanded_accel_vector.norm())
    #
    #     if commanded_accel_vector.norm() > max_accel:
    #         # print("commanding too much acceleration!")
    #         # print(f"giving angle of {commanded_vector.angle()}")
    #         # print(f"initial x: {commanded_fwd}; initial y: {commanded_strafe}; final x: {commanded_vector.X()}; final y: {commanded_vector.Y()}")
    #         if commanded_vector.norm() == 0:
    #             print(f"suposed to go to 0")
    #             commanded_vector = self.prev_commanded_vector - Translation2d(distance=max_accel * time_since_last_called, angle=self.prev_commanded_vector.angle())
    #         else:
    #             print(" ")
    #             commanded_vector = self.prev_commanded_vector + Translation2d(distance=max_accel * time_since_last_called, angle=commanded_vector.angle())
    #
    #     self.prev_commanded_vector = commanded_vector
    #     return (commanded_vector.X(), commanded_vector.Y())

