import math
import commands2
from commands2.button import CommandXboxController
from rev import SparkMax
from wpilib import SmartDashboard, Timer
from constants import WristConstants
from subsystems.elevator import Elevator
from subsystems.pivot import Pivot
from subsystems.wrist import Wrist


class MoveWristByJoystick(commands2.Command):

    def __init__(self, container, controller: CommandXboxController, timeout, wait_to_finish=False, indent=0) -> None:
        """
        :param wait_to_finish=False: will not make this command instantaneously execute.
        It will make this command end immediately after either the timeout has elapsed,
        or after the wrist has begun moving, whichever is first. wait_to_finish=True 
        will make the command end either after the timeout has elapsed, or after the 
        wrist has *finished* moving, whichever is first.

        To make it act kinda instantaneous, set a very small timeout. Then, it will not 
        move the wrist unless the arm is pretty much already at a safe position.
        """
        super().__init__()
        self.setName('Move wrist by joystick')
        self.indent = indent
        self.container = container
        self.wrist: Wrist = self.container.wrist
        self.pivot: Pivot = self.container.pivot
        self.elevator: Elevator = self.container.elevator
        self.controller = controller
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

        print(f"controller right X: {self.controller.getRightX()}")
        if self.controller.getRightX() > 0.5:
            self.setpoint = math.radians(-90)
        elif self.controller.getRightX() < -0.5:
            self.setpoint = math.radians(90)

        if ((self.pivot.get_angle() > WristConstants.k_min_arm_angle_where_spinning_dangerous and self.pivot.get_angle() < WristConstants.k_max_arm_angle_where_spinning_dangerous)
            and self.elevator.get_height() < WristConstants.k_max_elevator_height_where_spinning_dangerous):
            # it's dangerous to move, don't
            return

        else:
            self.wrist.set_position(radians=self.setpoint, control_type=SparkMax.ControlType.kPosition)
            self.moved_wrist = True

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
