import commands2
import wpilib
import typing
import constants
import math

from wpilib import SmartDashboard
from subsystems.double_pivot import DoublePivot

class MoveDoublePivot(commands2.Command):
    def __init__(self, container, double_pivot:DoublePivot, elbow_target_angle, shoulder_target_angle) -> None:
        super().__init__()
        self.setName('Move Double Pivot')
        self.double_pivot = double_pivot
        self.elbow_direction_sign = math.copysign(1, self.double_pivot.deltaAngle(elbow_target_angle, self.double_pivot.get_elbow_pivot()))
        self.shoulder_direction_sign = math.copysign(1, self.double_pivot.deltaAngle(shoulder_target_angle, self.double_pivot.get_shoulder_pivot()))

        self.elbow_target = elbow_target_angle
        self.shoulder_target = shoulder_target_angle
        self.elbow_done = False
        self.shoulder_done = False

        self.container = container
        self.addRequirements(self.double_pivot)

        SmartDashboard.putNumber("Double Pivot Shoulder Target Angle", shoulder_target_angle)
        SmartDashboard.putNumber("Double Pivot Elbow Target Angle", elbow_target_angle)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

    def execute(self) -> None:        
        if not self.elbow_done:
            self.double_pivot.set_elbow_pivot(self.double_pivot.get_elbow_pivot() + (1.0 * self.elbow_direction_sign))
            self.elbow_done = abs(self.double_pivot.get_elbow_pivot() - self.elbow_target) < constants.ElevatorConstants.k_tolerance

        if not self.shoulder_done:
            self.double_pivot.set_shoulder_pivot(self.double_pivot.get_shoulder_pivot() + (1.0 * self.shoulder_direction_sign))
            self.shoulder_done = abs(self.double_pivot.get_shoulder_pivot() - self.shoulder_target) < constants.ElevatorConstants.k_tolerance

        SmartDashboard.putNumber("Double Pivot Elbow Position", self.double_pivot.get_elbow_pivot())
        SmartDashboard.putNumber("Double Pivot Shoulder Position", self.double_pivot.get_shoulder_pivot())

    def isFinished(self) -> bool:
        return self.elbow_done and self.shoulder_done
    
    def end(self, interrupted: bool) -> None:        
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
