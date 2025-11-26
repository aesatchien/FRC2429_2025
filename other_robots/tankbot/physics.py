import math
import wpilib
import wpilib.simulation as simlib  # 2021 name for the simulation library
import wpimath.geometry as geo
from wpimath.kinematics._kinematics import SwerveDrive4Kinematics, SwerveModuleState, SwerveModulePosition
import wpimath.kinematics as kin
from pyfrc.physics.core import PhysicsInterface

from robot import MyRobot
import constants


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # Copied from 2024 code
        self.physics_controller = physics_controller  # must have for simulation
        self.robot = robot
        self.container = robot.container

        # Teaching constants
        self.track_width = 0.6  # meters
        self.max_speed = 4.0  # m/s at motor command = ±1

        self.kin = kin.DifferentialDriveKinematics(self.track_width)

    def update_sim(self, now, dt):

        # 1. Read SparkMax commands
        # CANSparkMax.get() returns the last commanded value in [-1, 1]
        left_cmd = self.container.drive.drive_l1.get()
        right_cmd = self.container.drive.drive_r1.get()

        # Safety clamp
        left_cmd = max(-1.0, min(1.0, left_cmd))
        right_cmd = max(-1.0, min(1.0, right_cmd))

        # 2. Convert to wheel speeds (m/s)
        left_speed = left_cmd * self.max_speed
        right_speed = right_cmd * self.max_speed

        # 3. Differential-drive kinematics
        vx = 0.5 * (left_speed + right_speed)               # forward m/s
        omega = (right_speed - left_speed) / self.track_width  # rad/s

        speeds = kin.ChassisSpeeds(vx, 0.0, omega)

        # 4. Move the robot
        self.physics_controller.drive(speeds, dt)

