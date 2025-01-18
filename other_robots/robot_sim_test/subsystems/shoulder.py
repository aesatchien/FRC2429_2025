import math

import wpilib
from commands2 import Subsystem
from wpilib import SmartDashboard
import rev

import constants

class Shoulder(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Double Pivot')

        # self.coral_positions = {key : constants.ElevatorConstants.k_positions[key] for key in ["l1", "l2", "l3", "l4"]}
        self.coral_positions = {key : constants.ElevatorConstants.k_positions[key]["shoulder_pivot"] for key in ["stow", "ground", "l1", "l2", "l3", "l4"]}

        #initialize motors and any other electrical component here (but skip for the purposes of simulation)
        #...

        self.shoulder_pivot = constants.ElevatorConstants.k_positions["stow"]["shoulder_pivot"]

            
    def get_shoulder_pivot(self):
        return self.shoulder_pivot

    def set_shoulder_pivot(self, shoulder_pivot):
        if wpilib.RobotBase.isReal():
            #code to move the shoulder pivot in real life
            pass
        else:
            self.shoulder_pivot = shoulder_pivot % 360

    def deltaAngle(self, target, current): #returns the shorter angle (and sign/direction) between two angles.
        target %= 360
        current %= 360

        if target < 0 or current < 0: raise ValueError("Angles must be positive")

        if current < target:
            current1 = current
            turnAmount1 = target - current1
            current2 = current + 360
            turnAmount2 = target - current2
            return turnAmount1 if abs(turnAmount1) < abs(turnAmount2) else turnAmount2

        else:
            target1 = target
            turnAmount1 = target1 - current
            target2 = target + 360
            turnAmount2 = target2 - current
        
            return turnAmount1 if abs(turnAmount1) < abs(turnAmount2) else turnAmount2