import math
import wpilib
import wpilib.simulation as simlib  # 2021 name for the simulation library
import wpimath.geometry as geo
from wpimath.kinematics._kinematics import SwerveDrive4Kinematics, SwerveModuleState, SwerveModulePosition
import wpimath.kinematics as kin
import wpimath.system.plant as plant
import rev
from pyfrc.physics.core import PhysicsInterface
from wpilib.simulation import FlywheelSim, DCMotorSim, SimDeviceSim

from robot import MyRobot
import constants
from helpers.blockhead_mech import BlockheadMech


class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # Copied from 2024 code
        self.physics_controller = physics_controller  # must have for simulation
        self.robot = robot
        self.container = robot.container
        self.counter = 0

        # Teaching constants
        self.track_width = 0.6  # meters
        self.max_speed = 4.0  # m/s at motor command = ±1

        self.kin = kin.DifferentialDriveKinematics(self.track_width)

        # ------------------- SHOOTER SIMULATION -------------------
        # Flywheel: Velocity is the primary concern. FlywheelSim is optimized for this.
        # 2025 Update: Use LinearSystemId to create the plant (Motor, MOI, Gearing)
        self.flywheel_plant = plant.LinearSystemId.flywheelSystem(plant.DCMotor.NEO(1), 0.01, 1.0)
        self.flywheel_sim = FlywheelSim(self.flywheel_plant, plant.DCMotor.NEO(1))
        self.flywheel_spark = rev.SparkMaxSim(self.container.shooter.flywheel_left_leader, plant.DCMotor.NEO(1))
        
        # Access internal SimDevice to read 'Reference' for closed-loop simulation
        self.flywheel_sim_device = SimDeviceSim(f"SPARK MAX [{self.container.shooter.flywheel_left_leader.getDeviceId()}]")

        # Indexer:
        self.indexer_spark = rev.SparkMaxSim(self.container.shooter.indexer_motor, plant.DCMotor.NEO(1))
        self.indexer_sim_device = SimDeviceSim(f"SPARK MAX [{self.container.shooter.indexer_motor.getDeviceId()}]")

        # Visualization
        self.mech = BlockheadMech()

    def update_sim(self, now, dt):

        self.counter += 1
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

        self.update_shooter(dt)


    def update_shooter(self, dt):
        # ------------------- SHOOTER UPDATE -------------------
        # setReference(kVelocity, value=rpm, ...) in the robot code changes this
        cmd_rpm = self.indexer_spark.getSetpoint()  # "setpoint sent to device" and we told it RPM
        # Spark sim expects motor velocity here and updates internal state/encoder
        self.indexer_spark.iterate(cmd_rpm,12, dt)

        pos = self.indexer_spark.getPosition()
        self.mech.update_indexer(pos)

        if self.counter % 200 == 0:
            pass
            # print(f'applied output: {applied_volts}')

# -------------------   SIM COMPONENTS  --------------------
