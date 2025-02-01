import commands2
import wpilib
import typing
import constants
import math

from wpilib import SmartDashboard
from subsystems.elevator import Elevator

class MoveElevator(commands2.Command):
    def __init__(self, container, elevator:Elevator, target, wait_to_finish=False) -> None:
        super().__init__()
        self.setName('Move Elevator')
        self.elevator = elevator
        self.target = target
        self.direction_sign = math.copysign(1, self.target - self.elevator.get_height())
        self.wait_to_finish = wait_to_finish
        self.container = container
        self.addRequirements(self.elevator)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")
        SmartDashboard.putNumber("Elevator Target Position", self.target)
        SmartDashboard.putString("Elevator Target Position Name", self.elevator.get_target_pos())

        self.elevator.set_height(self.target)

    def execute(self) -> None:
        SmartDashboard.putNumber("Elevator Position", self.elevator.get_height())

    def isFinished(self) -> bool:
        if self.wait_to_finish:
            # should we put this in the subsystem, like in wrist?
            return abs(self.elevator.get_height() - self.target) < constants.ElevatorConstants.k_tolerance
        else:
            return True
    
    def end(self, interrupted: bool) -> None:        
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
