import math
from commands2 import Subsystem
from wpilib import SmartDashboard
from wpilib import Mechanism2d
from wpilib import Color8Bit, Color

import constants
from subsystems.elevator import Elevator
from subsystems.pivot import Pivot
from subsystems.climber import Climber

class AdvScope(Subsystem):
    def __init__(self, pivot:Pivot, elevator:Elevator, climber:Climber):
        super().__init__()
        self.setName('AdvScope')
        self.counter = constants.IntakeConstants.k_counter_offset
        self.pivot = pivot # type: Pivot
        self.elevator = elevator # type: Elevator
        self.climber = climber # type: Climber
        self.mech2d = Mechanism2d(0, 0)
        self.climber_mech2d = Mechanism2d(0, 0)
        self.root = self.mech2d.getRoot("Root", 0, 0)
        self.climber_root = self.climber_mech2d.getRoot("Climber_Root", 0.25, 0)
        
        # MechanismLigament2d objects represent each "section"/"stage" of the mechanism, and are based
        # off the root node or another ligament object

        self.elevator_mech = self.root.appendLigament("elevator", 0.5, 90, 10, Color8Bit(Color.kRed))
        self.pivot_mech = self.elevator_mech.appendLigament("pivot", 0.5, 90, 6, Color8Bit(Color.kYellow))
        self.climber_mech = self.climber_root.appendLigament("climber", 0.5, 90, 6, Color8Bit(Color.kBlue))                

        SmartDashboard.putData("Mech2d_ElevPivot", self.mech2d)
        SmartDashboard.putData("Mech2d_Climber", self.climber_mech2d)

    def periodic(self) -> None:
        self.counter += 1
        if self.counter % 10 == 0:
            elevator_height = self.elevator.get_height() #unit is meter
            pivot_angle = 270 + math.degrees(self.pivot.get_angle())   #unit is degree
            climber_angle = 90 - math.degrees(self.climber.get_angle())  #unit is degree

            self.elevator_mech.setLength(elevator_height)
            self.pivot_mech.setAngle(pivot_angle)
            self.climber_mech.setAngle(climber_angle)

        return super().periodic()