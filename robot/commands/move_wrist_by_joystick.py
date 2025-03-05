import math
import commands2
from commands2.button import CommandXboxController
from rev import SparkMax
from wpilib import DriverStation, SmartDashboard, Timer
import wpilib
from constants import WristConstants
from subsystems.elevator import Elevator
from subsystems.pivot import Pivot
from subsystems.robot_state import RobotState
from subsystems.swerve import Swerve
from subsystems.wrist import Wrist


class MoveWristByJoystick(commands2.Command):

    def __init__(self, container,  timeout, side_decider: CommandXboxController | RobotState, swerve_for_field_centric: Swerve | None =None, wait_to_finish=False, indent=0) -> None:
        """
        :param wait_to_finish=False: will not make this command instantaneously execute.
        It will make this command end immediately after either the timeout has elapsed,
        or after the wrist has begun moving, whichever is first. wait_to_finish=True 
        will make the command end either after the timeout has elapsed, or after the 
        wrist has *finished* moving, whichever is first.

        To make it act kinda instantaneous, set a very small timeout. Then, it will not 
        move the wrist unless the arm is pretty much already at a safe position.

        if you pass :param swerve_for_field_centric:, it will be field centric (ie if the robot 
        is pointing at the drivers, the command will invert the driver's request because
        the driver's left is the robot's right
        """

        super().__init__()
        self.setName('Move wrist by joystick')
        self.indent = indent
        self.container = container
        self.wrist: Wrist = self.container.wrist
        self.pivot: Pivot = self.container.pivot
        self.elevator: Elevator = self.container.elevator
        self.side_decider = side_decider
        self.swerve = swerve_for_field_centric
        self.timeout = timeout
        self.wait_to_finish = wait_to_finish
        self.timer = Timer()
        self.setpoint = 0
        self.addRequirements(self.wrist)

        # initializes
        # waits for either timeout or safe wrist movement
        # if timed out: exit
        # if wrist safe: move wrist, exit

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        msg = self.indent * "    " + f"** Started {self.getName()} at {self.start_time} s **"
        print(msg, flush=True)
        SmartDashboard.putString("alert", msg)

        self.setpoint = 0
        self.moved_wrist = False
        self.timer.reset()

    # NOTE 20250225 - why is this in execute and not initialize?  seems like it will get called many times
    def execute(self) -> None:

        if type(self.side_decider) == CommandXboxController:
            if self.side_decider.getRightX() > 0.5:
                self.setpoint = math.radians(-90)
            elif self.side_decider.getRightX() < -0.5:
                self.setpoint = math.radians(90)

        elif type(self.side_decider) == RobotState:
            if self.side_decider.is_left():
                self.setpoint = math.radians(90)
            else:
                self.setpoint = math.radians(-90)

        # if self.swerve:
        #     if -90 < self.swerve.get_angle() <= 90:
        #         if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
        #             # we are facing away from the driver so don't invert
        #             pass
        #         else:
        #             # we are facing towards the driver so invert
        #             self.setpoint *= -1
        #
        #     else:
        #         if DriverStation.getAlliance() == DriverStation.Alliance.kBlue:
        #             # we are facing towards the driver so invert
        #             self.setpoint *= -1
        #         else:
        #             pass

        if self.wrist.is_safe_to_move():
            self.wrist.set_position(radians=self.setpoint, control_type=SparkMax.ControlType.kPosition)
            self.moved_wrist = True

        else:
            print("not moving too dangerous")
            # it's dangerous to move, don't
            return

    def isFinished(self) -> bool:

        if self.timer.get() > self.timeout: return True

        if self.wait_to_finish:
            return self.wrist.get_at_setpoint() and self.setpoint != 0
        else:
            return self.moved_wrist

    def end(self, interrupted: bool) -> None:
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
