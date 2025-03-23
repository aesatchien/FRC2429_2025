import commands2
from rev import SparkMax
from wpilib import SmartDashboard, Timer
from constants import WristConstants
import constants
from subsystems.pivot import Pivot
from subsystems.swerve import Swerve
from subsystems.wrist import Wrist


class StowWristAfterPositionDelta(commands2.Command):

    def __init__(self, container, wait_to_finish=False, indent=0) -> None:
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
        self.setName('Stow wrist because position delta')
        self.indent = indent
        self.container = container
        self.wrist: Wrist = self.container.wrist
        self.swerve: Swerve = self.container.swerve
        self.wait_to_finish = wait_to_finish
        self.timer = Timer()
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

        self.initial_translation = self.swerve.get_pose().translation()

        self.moved_wrist = False
        self.timer.reset()

    # NOTE 20250225 - why is this in execute and not initialize?  seems like it will get called many times
    def execute(self) -> None:
        delta_translation = self.swerve.get_pose().translation() - self.initial_translation
        if delta_translation.norm() > 0.3: # we've moved that much since calling this command, so we should be safe to move the wrist
            self.wrist.set_position(constants.k_positions["stow"]["wrist_pivot"])
            self.moved_wrist = True
        

    def isFinished(self) -> bool:

        if self.wait_to_finish:
            return self.wrist.get_at_setpoint() and self.moved_wrist
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
