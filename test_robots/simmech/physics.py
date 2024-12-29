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

        # Motors
        self.l_motor = wpilib.simulation.PWMSim(robot.l_motor.getChannel())
        self.r_motor = wpilib.simulation.PWMSim(robot.r_motor.getChannel())

        self.dio1 = wpilib.simulation.DIOSim(robot.limit1)
        self.dio2 = wpilib.simulation.DIOSim(robot.limit2)
        self.ain2 = wpilib.simulation.AnalogInputSim(robot.position)

        self.motor = wpilib.simulation.PWMSim(robot.motor.getChannel())

        # Gyro
        self.gyro = wpilib.simulation.AnalogGyroSim(robot.gyro)

        self.position = 0

        # Change these parameters to fit your robot!
        bumper_width = 3.25 * units.inch

        # fmt: off
        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                    # robot mass
            10.71,                              # drivetrain gear ratio
            2,                                  # motors per side
            22 * units.inch,                    # robot wheelbase
            23 * units.inch + bumper_width * 2, # robot width
            32 * units.inch + bumper_width * 2, # robot length
            6 * units.inch,                     # wheel diameter
        )
        # fmt: on
        self.count = 0

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        l_motor = self.l_motor.getSpeed()
        r_motor = self.r_motor.getSpeed()

        transform = self.drivetrain.calculate(l_motor, r_motor, tm_diff)
        pose = self.physics_controller.move_robot(transform)

        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        self.gyro.setAngle(-pose.rotation().degrees())

        # update position (use tm_diff so the rate is constant)
        self.position += self.motor.getSpeed() * tm_diff * 3

        # update limit switches based on position
        if self.position <= 0:
            switch1 = True
            switch2 = False

        elif self.position > 10:
            switch1 = False
            switch2 = True

        else:
            switch1 = False
            switch2 = False

        # set values here
        self.dio1.setValue(switch1)
        self.dio2.setValue(switch2)
        self.ain2.setVoltage(self.position)

        # play with the new mechs - 
        # they are located with the components dictionary, e.g. sm.side_mech.components["Indexer"]["ligament"]
        self.count += 0.25 * 360/50  # one quarter of a circle (360/4) per second,  running 50x per second (/50)
        # set the mech angles
        sm.side_mech.components["Shooter Arm"]["ligament"].setAngle(90 + self.count % 360)  # it's in degrees, zero is pointing right, so we start up (+90)
        sm.side_mech.components["Crank Arm Tower"]["ligament"].setAngle(90 + 10 * math.cos(self.count * math.pi/180))  # it's in degrees, 90 is pointing up
        # set some mech colors
        test_color = wpilib.Color8Bit(wpilib.Color.kLimeGreen) if self.count % 360 > 180 else wpilib.Color8Bit(wpilib.Color.kDarkRed)
        note_color = wpilib.Color8Bit(wpilib.Color.kOrange) if self.count % 360 > 180 else wpilib.Color8Bit(wpilib.Color.kYellow)
        sm.side_mech.components["Spacer"]["ligament"].setColor(note_color)
        sm.side_mech.components["Indexer"]["ligament"].setColor(test_color)
        sm.side_mech.components["Flywheel"]["ligament"].setColor(test_color)
        sm.top_mech.components["Intake"]["ligament"].setColor(test_color)
        sm.side_mech.components["Intake"]["ligament"].setColor(test_color)
        # change some mech lengths
        sm.top_mech.components["Intake"]["ligament"].setLength( sm.top_mech.components["Intake"]["length"]* math.fabs(math.cos(self.count * 0.5 * math.pi/180)))
        sm.side_mech.components["Intake"]["ligament"].setLength( sm.side_mech.components["Intake"]["length"]* math.fabs(math.cos(self.count * 0.5 * math.pi/180)))
        # change some wheel angles and colors
        # TODO: link the colors to the drive motor and the angles to the turn motor
        swerves = swerves = ['swerve_right', 'swerve_front', 'swerve_left', 'swerve_back']
        for swerve in swerves:
            start_angle = sm.top_mech.components[swerve + 'lig_front']['angle']
            angle = (start_angle + self.count) % 360
            sm.top_mech.components[swerve + 'lig_front']["ligament"].setAngle(angle)
            sm.top_mech.components[swerve + 'lig_back']["ligament"].setAngle(angle+180)
            sm.top_mech.components[swerve + 'lig_back']["ligament"].setColor(test_color)
