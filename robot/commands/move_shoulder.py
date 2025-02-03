import commands2
import wpilib
import typing
import constants
import math

from wpilib import SmartDashboard
from subsystems.shoulder import Shoulder

class MoveShoulder(commands2.Command):
    def __init__(self, container, shoulder:Shoulder, shoulder_target_angle) -> None:
        super().__init__()
        self.setName('Move Double Pivot')
        self.shoulder = shoulder
        self.shoulder_target = shoulder_target_angle

        self.shoulder_direction_sign = math.copysign(1, self.shoulder.deltaAngle(self.shoulder_target, self.shoulder.get_shoulder_pivot()))

        self.container = container
        self.addRequirements(self.shoulder)

        SmartDashboard.putNumber("Shoulder Target Angle", self.shoulder_target)
        SmartDashboard.putNumber("Shoulder Target Angle DIRECTION", self.shoulder_direction_sign)
        SmartDashboard.putNumber("Shoulder Target Delta Angle", self.shoulder.deltaAngle(self.shoulder_target, self.shoulder.get_shoulder_pivot()))

    def initialize(self) -> None:
        
        if wpilib.RobotBase.isReal():
            self.shoulder.set_shoulder_pivot(self.shoulder_target)

        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")

    def execute(self) -> None:        
        if wpilib.RobotBase.isSimulation():
            self.shoulder.set_shoulder_pivot(self.shoulder.get_shoulder_pivot() + (1.0 * self.shoulder_direction_sign))

        SmartDashboard.putNumber("Shoulder Position", self.shoulder.get_shoulder_pivot())
        SmartDashboard.putNumber("Shoulder Target Delta Angle", self.shoulder.deltaAngle(self.shoulder_target, self.shoulder.get_shoulder_pivot()))

    def isFinished(self) -> bool:
        return abs(self.shoulder.deltaAngle(self.shoulder_target, self.shoulder.get_shoulder_pivot())) < constants.ElevatorConstants.k_tolerance_degrees
    
    def end(self, interrupted: bool) -> None:        
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **", flush=True)
        SmartDashboard.putString(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s **")
