import math
import wpilib
import wpilib.simulation as simlib  # 2021 name for the simulation library
import wpimath.geometry as geo
from wpimath.kinematics._kinematics import SwerveDrive4Kinematics, SwerveModuleState, SwerveModulePosition
from pyfrc.physics.core import PhysicsInterface

from robot import MyRobot
import constants


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # Copied from 2024 code
        self.physics_controller = physics_controller  # must have for simulation
        self.robot = robot


    def update_sim(self, now, tm_diff):
        pass

