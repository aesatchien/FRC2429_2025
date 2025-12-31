import wpilib
import wpilib.simulation as simlib  # 2021 name for the simulation library
from  wpimath.geometry import Pose2d, Transform2d
from pyfrc.physics.core import PhysicsInterface
import ntcore

from robot import MyRobot
import constants
from simulation import sim_utils
from simulation.swerve_sim import SwerveSim
from simulation.gamepiece_sim import GamepieceSim
from simulation.vision_sim import VisionSim

class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # Copied from 2024 code
        self.physics_controller = physics_controller  # must have for simulation
        self.robot = robot


        self._init_networktables()
        
        # Create a Field2d for visualization
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)  # this should just keep the default one
        self.target_object = self.field.getObject("Target")

        # Initialize Simulations
        self.swerve_sim = SwerveSim(physics_controller, robot)
        self.gamepiece_sim = GamepieceSim(self.field)
        self.vision_sim = VisionSim(self.field)

        # Ghost Robot linger state
        self.last_ghost_update_time = 0
        self.ghost_linger_duration = 2.0 # seconds

        # Initial Pose
        self.physics_controller.move_robot(Transform2d(constants.k_start_x, constants.k_start_y, 0))

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        sim_prefix = constants.sim_prefix
        auto_sim_prefix = constants.auto_prefix

        # ground truth Publisher for Simulating Sensors
        self.ground_truth_pub = self.inst.getStructTopic(f"{sim_prefix}/ground_truth", Pose2d).publish()

        # Ghost Robot Subscribers - used for tracking goals in auto
        self.auto_active_sub = self.inst.getBooleanTopic(f"{auto_sim_prefix}/robot_in_auto").subscribe(False)
        self.goal_pose_sub = self.inst.getStructTopic(f"{auto_sim_prefix}/goal_pose", Pose2d).subscribe(Pose2d())

    def update_sim(self, now, tm_diff):

        # simlib.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue2)
        amps = []

        # Update Physics Models
        self.swerve_sim.update(tm_diff)

        simlib.RoboRioSim.setVInVoltage(
                simlib.BatterySim.calculate(amps)
        )
        
        # Update Game State
        robot_pose = self.physics_controller.get_pose()
        self.gamepiece_sim.update(robot_pose)
        
        # Update Vision
        active_gamepieces = self.gamepiece_sim.get_active_gamepieces()
        self.vision_sim.update(robot_pose, active_gamepieces)
        
        # Update Field2d
        self.field.setRobotPose(robot_pose)
        self.ground_truth_pub.set(robot_pose)

        # Update Ghost Robot
        if self.auto_active_sub.get():
            self.target_object.setPose(self.goal_pose_sub.get())
            self.last_ghost_update_time = now
        else:
            # make it disappear after the ghost timeout
            if now - self.last_ghost_update_time > self.ghost_linger_duration:
                self.target_object.setPoses([])