#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

#
# See the documentation for more details on how this works
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#

import wpilib.simulation

from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units

import typing
import constants

if typing.TYPE_CHECKING:
    from robot import MyRobot

import simmech as sm
import math


class PhysicsEngine:
    """
    Simulates a motor moving something that strikes two limit switches,
    one on each end of the track. Obviously, this is not particularly
    realistic, but it's good enough to illustrate the point
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        self.physics_controller = physics_controller

        self.robot = robot
        self.update_positions()

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # move the elevator based on controller input
        self.update_positions()

    def update_positions(self):
        if self.robot is None:
            raise ValueError("Robot is not defined")
        
        self.elevator_height_sim = self.robot.container.elevator.get_height() * (constants.ElevatorConstants.k_elevator_sim_max_height / constants.ElevatorConstants.k_elevator_max_height)
        self.shoulder_pivot = self.robot.container.double_pivot.get_shoulder_pivot()
        self.elbow_pivot = self.robot.container.double_pivot.get_elbow_pivot()
        self.wrist_color = self.robot.container.double_pivot.get_wrist_color()
        
        sm.front_elevator.components["elevator_right"]["ligament"].setLength(self.elevator_height_sim)
        sm.front_elevator.components["elevator_left"]["ligament"].setLength(self.elevator_height_sim)
        
        sm.side_elevator.components["elevator_side"]["ligament"].setLength(self.elevator_height_sim)
        sm.side_elevator.components["double_pivot_shoulder"]["ligament"].setAngle(self.shoulder_pivot)
        sm.side_elevator.components["double_pivot_elbow"]["ligament"].setAngle(self.elbow_pivot)
        sm.side_elevator.components["wrist"]["ligament"].setColor(self.wrist_color)
