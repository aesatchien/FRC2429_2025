import math

import wpilib
from commands2 import Subsystem
from wpilib import SmartDashboard
import rev

import constants

class DoublePivot(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Double Pivot')

        # self.coral_positions = {key : constants.ElevatorConstants.k_positions[key] for key in ["l1", "l2", "l3", "l4"]}
        self.coral_positions = {key : {pos : constants.ElevatorConstants.k_positions[key][pos] for pos in ["elbow_pivot", "shoulder_pivot"]} for key in ["stow", "ground", "l1", "l2", "l3", "l4"]}

        #initialize motors and any other electrical component here (but skip for the purposes of simulation)
        #...

        self.elbow_pivot = constants.ElevatorConstants.k_positions["stow"]["elbow_pivot"]
        self.shoulder_pivot = constants.ElevatorConstants.k_positions["stow"]["shoulder_pivot"]
        self.wrist_color = constants.ElevatorConstants.k_positions["stow"]["wrist_color"]

    def get_elbow_pivot(self):
        return self.elbow_pivot
            
    def get_shoulder_pivot(self):
        return self.shoulder_pivot

    def get_wrist_color(self):
        return self.wrist_color
    
    def set_elbow_pivot(self, elbow_pivot):
        if wpilib.RobotBase.isReal():
            #code to move the elbow pivot in real life
            pass
        else:
            self.elbow_pivot = elbow_pivot % 360

    def set_shoulder_pivot(self, shoulder_pivot):
        if wpilib.RobotBase.isReal():
            #code to move the shoulder pivot in real life
            pass
        else:
            self.shoulder_pivot = shoulder_pivot % 360

    def deltaAngle(self, target, current): #returns the shorter angle (and sign/direction) between two angles.
        target2 = target + 360 if target < 0 else target
        current2 = current + 360 if current < 0 else current
        turnAmount1 = target - current
        turnAmount2 = target2 - current2
        return turnAmount1 if abs(turnAmount1) < abs(turnAmount2) else turnAmount2