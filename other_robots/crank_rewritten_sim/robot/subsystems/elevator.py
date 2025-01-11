import math

import wpilib
from commands2 import Subsystem
from wpilib import SmartDashboard
import rev

import constants

class Elevator(Subsystem):
    def __init__(self):
        super().__init__()
        self.setName('Elevator')

        self.max_height = 1.5 #unit: meters
        self.min_height = 0.2 #these are random values; eventually make it a constant.
        self.positions = {"l1": self.min_height, "l2": 0.6, "l3": 1.2, "l4": self.max_height}

        #initialize motors and any other electrical component here (but skip for the purposes of simulation)
        #...

        self.height = self.min_height #assumes elevator starts at the bottom

    def get_height(self):
        return self.height

    def set_height(self, height):
        if wpilib.RobotBase.isReal():
            #code to move the elevator in real life
            pass
        else:
            if height >= self.max_height:
                height = self.max_height
            elif height <= self.min_height:
                height = self.min_height
            else: self.height = height

    def next_pos(self, direction):
        if direction == "up":
            for pos in self.positions.values():
                if self.height + 0.1 < pos: #in the future, numbers like "0.03" will be a value relative to some tolerance constant
                    return pos
            return self.max_height
            
        elif direction == "down":
            temp_list = list(self.positions.values())
            temp_list.reverse()
            for pos in temp_list:
                if self.height - 0.1 > pos:
                    return pos
            return self.min_height