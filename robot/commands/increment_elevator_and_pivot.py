import commands2
import wpilib
import typing
import constants
import math

from wpilib import SmartDashboard
from subsystems.elevator import Elevator
from subsystems.shoulder import Shoulder
from commands.move_elevator import MoveElevator
from commands.move_shoulder import MoveShoulder

class IncrementElevatorAndPivot(commands2.Command):
    def __init__(self, container, elevator:Elevator, pivot:Shoulder, direction="down") -> None:
        super().__init__()
        self.setName('Increment Elevator and Pivot')
        self.elevator = elevator
        self.double_pivot = pivot
        self.container = container
        self.position_list = list(self.elevator.coral_positions.values())
        self.direction = direction
        self.addRequirements(self.elevator)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

        self.targets = self.elevator.next_pos(self.direction)
        
        command = commands2.cmd.parallel(
            MoveElevator(self.container, self.elevator, self.targets["elevator_height"]), 
            MoveShoulder(self.container, self.double_pivot, self.targets["shoulder_pivot"]))
        
        commands2.CommandScheduler.getInstance().schedule(command)
        
    def execute(self) -> None:
        pass

    def isFinished(self) -> bool:
        return self.command.isFinished()
    
    def end(self, interrupted: bool) -> None:        
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
