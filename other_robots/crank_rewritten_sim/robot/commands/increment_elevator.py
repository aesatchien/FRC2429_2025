import commands2
import wpilib
import typing

from wpilib import SmartDashboard
from subsystems.elevator import Elevator

class IncrementElevator(commands2.Command):
    def __init__(self, container, elevator:Elevator, direction="down") -> None:
        super().__init__()
        self.setName('Increment Elevator')
        self.elevator = elevator
        self.direction = direction
        self.direction_sign = 1 if self.direction == "up" else -1
        self.container = container
        self.position_list = list(self.elevator.positions.values())
        self.addRequirements(self.elevator)
        self.tolerance = 0.01

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

        self.target_position = self.elevator.next_pos(self.direction)
        SmartDashboard.putNumber("Elevator Target Position", self.target_position)

    def execute(self) -> None:
        self.elevator.set_height(self.elevator.get_height() + (0.01 * self.direction_sign))
        SmartDashboard.putNumber("Elevator Position", self.elevator.get_height())
        # SmartDashboard.putNumber("Elevator Target Position", self.target_position)

    def isFinished(self) -> bool:
        return abs(self.elevator.get_height() - self.target_position) < self.tolerance
    
    def end(self, interrupted: bool) -> None:        
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
