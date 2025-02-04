import commands2
import wpilib
import typing
import constants
import math

from wpilib import SmartDashboard
from subsystems.shoulder import Shoulder

class MoveShoulder(commands2.Command):
    def __init__(self, container, double_pivot:Shoulder, shoulder_target_angle) -> None:
        super().__init__()
        self.setName('Move Double Pivot')
        self.double_pivot = double_pivot
        self.shoulder_target = shoulder_target_angle

        self.shoulder_direction_sign = math.copysign(1, self.double_pivot.deltaAngle(self.shoulder_target, self.double_pivot.get_shoulder_pivot()))

        self.container = container
        self.addRequirements(self.double_pivot)

        SmartDashboard.putNumber("Shoulder Target Angle", self.shoulder_target)
        SmartDashboard.putNumber("Shoulder Target Angle DIRECTION", self.shoulder_direction_sign)
        SmartDashboard.putNumber("Shoulder Target Delta Angle", self.double_pivot.deltaAngle(self.shoulder_target, self.double_pivot.get_shoulder_pivot()))

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

    def execute(self) -> None:        
        self.double_pivot.set_shoulder_pivot(self.double_pivot.get_shoulder_pivot() + (1.0 * self.shoulder_direction_sign))

        SmartDashboard.putNumber("Shoulder Position", self.double_pivot.get_shoulder_pivot())
        SmartDashboard.putNumber("Shoulder Target Delta Angle", self.double_pivot.deltaAngle(self.shoulder_target, self.double_pivot.get_shoulder_pivot()))

    def isFinished(self) -> bool:
        return abs(self.double_pivot.deltaAngle(self.shoulder_target, self.double_pivot.get_shoulder_pivot())) < constants.ElevatorConstants.k_tolerance
    
    def end(self, interrupted: bool) -> None:        
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
