from typing import Union
import commands2
from wpilib import SharpIR, SmartDashboard
from subsystems.intake import Intake
from subsystems.elevator import Elevator
from subsystems.pivot import Pivot
from subsystems.wrist import Wrist


class DriveByJoystickSubsystem(commands2.Command): # for debugging; we might want to put this as an emergency override in comp

    def __init__(self, container, controller: commands2.button.CommandXboxController, subsystem: Union[Intake, Elevator, Pivot, Wrist], duty_cycle_coef, indent=0) -> None:
        super().__init__()
        self.setName('Drive by joystick subsystem')  # change this to something appropriate for this command
        self.indent = indent
        self.container = container
        self.subsystem = subsystem

        self.duty_cycle_coef = duty_cycle_coef
        self.controller = controller

        # self.addRequirements(self.container.)  # commandsv2 version of requirements

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print(f"{self.indent * '    '}** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():2.2f} s **")

        if type(self.subsystem) == Elevator or type(self.subsystem) == Pivot:
            self.subsystem.disable() # must disable our trapezoidal subsystems

    def execute(self) -> None:
        # this is definitely a janky way of doing it
        if type(self.subsystem) == Elevator or type(self.subsystem) == Pivot:
            self.subsystem.motor.set(-self.controller.getRightY() * self.duty_cycle_coef)
        elif type(self.subsystem) == Wrist or type(self.subsystem) == Intake:
            self.subsystem.sparkmax.set(-self.controller.getRightY() * self.duty_cycle_coef)

    def isFinished(self) -> bool:
        return True

    def end(self, interrupted: bool) -> None:
        if type(self.subsystem) == Elevator or type(self.subsystem) == Pivot:
            self.subsystem.enable() # must disable our trapezoidal subsystems

        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print_end_message = True
        if print_end_message:
            print(f"{self.indent * '    '}** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
            SmartDashboard.putString(f"alert",
                                     f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
