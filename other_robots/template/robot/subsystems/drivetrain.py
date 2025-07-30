import math
import wpilib
from commands2 import Subsystem
import rev

import constants


class Drivetrain(Subsystem):
    def __init__(self, container) -> None:
        super().__init__()
        self.setName('Drivetrain')
        self.container = container
        self.counter = 0

        # add motors and drive methods

    def periodic(self):
        # here we do all the checking of the robot state - read inputs, calculate outputs
        self.counter += 1

        if self.counter % 50 == 0:
            # update the SmartDashboard
            wpilib.SmartDashboard.putNumber('counter', self.counter)
