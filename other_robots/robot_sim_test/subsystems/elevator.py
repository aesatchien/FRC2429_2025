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

        self.coral_positions = {key : constants.ElevatorConstants.k_positions[key] for key in ["stow", "ground", "l1", "l2", "l3", "l4"]}

        #initialize motors and any other electrical component here (but skip for the purposes of simulation)
        #...

        self.height = constants.ElevatorConstants.k_positions["stow"]["elevator_height"]
        
        self.target_pos = "stow"

        self.has_coral = False #put in wrist later

    def get_height(self):
        return self.height

    def get_target_pos(self):
        return self.target_pos

    def set_height(self, height):
        if wpilib.RobotBase.isReal():
            #code to move the elevator in real life
            pass
        else:
            if height >= constants.ElevatorConstants.k_elevator_max_height:
                self.height = constants.ElevatorConstants.k_elevator_max_height
            elif height <= constants.ElevatorConstants.k_elevator_min_height:
                self.height = constants.ElevatorConstants.k_elevator_min_height
            else: self.height = height

    def next_pos(self, direction="down"):
        keys = list(self.coral_positions.keys())
        current_index = keys.index(self.target_pos)

        if direction == "up":
            if current_index + 1 < len(keys):
                self.target_pos = keys[current_index + 1]
                return (self.coral_positions[self.target_pos])
            return (self.coral_positions[self.target_pos])
        else:
            if current_index - 1 >= 0:
                self.target_pos = keys[current_index - 1]
                return (self.coral_positions[self.target_pos])
            return (self.coral_positions[self.target_pos])
    
    def set_has_coral(self, has_coral):
        self.has_coral = has_coral
        
    def get_has_coral(self):
        return self.has_coral