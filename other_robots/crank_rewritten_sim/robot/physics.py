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
        self.elevator_height_sim = self.robot.container.elevator.get_height() * (50 / 1.5) #joshsugino 1/10/25 - (50/1.5) is the conversion from meters to simulation units.  Will implement constants later for better readability
        sm.side_mech.components["elevator_right"]["ligament"].setLength(self.elevator_height_sim)
        sm.side_mech.components["elevator_left"]["ligament"].setLength(self.elevator_height_sim)

        self.count = 0

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        self.count += 0.25 * 360/50  # one quarter of a circle (360/4) per second,  running 50x per second (/50)

        # move the elevator based on controller input
        self.elevator_height_sim = self.robot.container.elevator.get_height() * (50 / 1.5) #joshsugino 1/10/25 - (50/1.5) is the conversion from meters to simulation units.  Will implement constants later for better readability
        sm.side_mech.components["elevator_right"]["ligament"].setLength(self.elevator_height_sim)
        sm.side_mech.components["elevator_left"]["ligament"].setLength(self.elevator_height_sim)

        # change some wheel angles and colors
        # TODO: link the colors to the drive motor and the angles to the turn motor
        test_color = wpilib.Color8Bit(wpilib.Color.kLimeGreen) if self.count % 360 > 180 else wpilib.Color8Bit(wpilib.Color.kDarkRed)

        swerves = swerves = ['swerve_right', 'swerve_front', 'swerve_left', 'swerve_back']
        for swerve in swerves:
            start_angle = sm.top_mech.components[swerve + 'lig_front']['angle']
            angle = (start_angle + self.count) % 360
            sm.top_mech.components[swerve + 'lig_front']["ligament"].setAngle(angle)
            sm.top_mech.components[swerve + 'lig_back']["ligament"].setAngle(angle+180)
            sm.top_mech.components[swerve + 'lig_back']["ligament"].setColor(test_color)
