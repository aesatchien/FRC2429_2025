import math
from rev import SparkFlex, SparkFlexSim, SparkMaxSim
import wpilib
import wpilib.simulation as simlib  # 2021 name for the simulation library
from  wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d
from wpimath.kinematics._kinematics import SwerveDrive4Kinematics, SwerveModuleState, SwerveModulePosition
from pyfrc.physics.core import PhysicsInterface
import ntcore

from robot import MyRobot
import constants
from simulation import sim_utils
from simulation.vision_sim import VisionSim
from subsystems.swerve_constants import DriveConstants as dc

class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # Copied from 2024 code
        self.physics_controller = physics_controller  # must have for simulation
        self.robot = robot

        # if we want to add an armsim see test_robots/sparksim_test/

        # gamepiece locations, using the notes from 2024 as an example.  may want to move to an init field function
        self.gamepiece_locations = [(2.89, 7.0), (2.89, 5.57), (2.89, 4.1), (8.28, 7.46), (8.28, 5.76), (8.28, 4.1),
                                    (8.28, 2.42), (8.28, 0.76), (13.68, 7.0), (13.68, 5.57), (13.68, 4.1)]
        self.gamepieces = [{'pos': Translation2d(gl), 'active': True} for gl in self.gamepiece_locations]

        self.initialize_swerve()

        self._init_networktables()
        
        # Create a Field2d for visualization
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)  # this should just keep the default one
        self.target_object = self.field.getObject("Target")

        # Initialize Vision Simulation
        self.vision_sim = VisionSim(self.field)

        # Ghost Robot linger state
        self.last_ghost_update_time = 0
        self.ghost_linger_duration = 2.0 # seconds

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        sim_prefix = constants.sim_prefix

        # ground truth Publisher for Simulating Sensors
        self.ground_truth_pub = self.inst.getStructTopic(f"{sim_prefix}/ground_truth", Pose2d).publish()

        # Swerve Debugging
        self.target_angles_pub = self.inst.getDoubleArrayTopic(f"{sim_prefix}/target_angles").publish()
        
        # Swerve Target Subscribers
        dash_values = ['lf_target_vel_angle', 'rf_target_vel_angle', 'lb_target_vel_angle', 'rb_target_vel_angle']
        self.swerve_target_subs = [self.inst.getDoubleArrayTopic(f"/SmartDashboard/{v}").subscribe([0, 0]) for v in dash_values]
        
        # Ghost Robot Subscribers - used for tracking goals in auto
        self.auto_active_sub = self.inst.getBooleanTopic(f"{sim_prefix}/robot_in_auto").subscribe(False)
        self.goal_pose_sub = self.inst.getStructTopic(f"{sim_prefix}/goal_pose", Pose2d).subscribe(Pose2d())

    def update_sim(self, now, tm_diff):

        # simlib.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue2)
        amps = []

        self.update_swerve(tm_diff)
        self.consume_gamepieces()

        simlib.RoboRioSim.setVInVoltage(
                simlib.BatterySim.calculate(amps)
        )
        self.vision_sim.update(self.physics_controller.get_pose(), self.gamepieces)
        
        # Update Field2d
        self.field.setRobotPose(self.physics_controller.get_pose())
        active_poses = [Pose2d(gp['pos'], Rotation2d()) for gp in self.gamepieces if gp['active']]
        self.field.getObject("Gamepieces").setPoses(active_poses)

        # Update Ghost Robot
        if self.auto_active_sub.get():
            self.target_object.setPose(self.goal_pose_sub.get())
            self.last_ghost_update_time = now
        else:
            # make it disappear after the ghost timeout
            if now - self.last_ghost_update_time > self.ghost_linger_duration:
                self.target_object.setPoses([])

    def reset_gamepieces(self):
        for gp in self.gamepieces:
            gp['active'] = True
        print(f"Simulation reset all game pieces")

    def consume_gamepieces(self):
        # Check if we are on any gamepiece
        # Optimization: only check active ones
        for gp in self.gamepieces:
            if gp['active']:
                # is_on_gamepiece uses distance < threshold (0.5m default)
                if sim_utils.is_on_gamepiece(self.physics_controller.get_pose(), gp['pos']):
                    gp['active'] = False
                    print(f"Simulation consumed gamepiece at {gp['pos']}")
        if len([gp for gp in self.gamepieces if gp['active']]) == 0:
            self.reset_gamepieces()

    def update_swerve(self, tm_diff):

        target_angles = [sub.get()[1] for sub in self.swerve_target_subs]
        for spark_turn, target_angle in zip(self.spark_turns, target_angles):
            self.spark_dict[spark_turn]['position'].set(target_angle)  # this works to update the simulated spark
        if constants.k_swerve_debugging_messages:
            self.target_angles_pub.set(target_angles)

        # send the speeds and positions from the spark sim devices to the fourmotorswervedrivetrain
        module_states = []
        for drive, turn in zip(self.spark_drives, self.spark_turns):
            module_states.append(SwerveModuleState(
                self.spark_dict[drive]['velocity'].value, Rotation2d(self.spark_dict[turn]['position'].value))
            )

        # using our own kinematics to update the chassis speeds
        module_states = self.robot.container.swerve.get_desired_swerve_module_states()
        speeds = self.kinematics.toChassisSpeeds(tuple(module_states))

        # update the sim's robot
        self.physics_controller.drive(speeds, tm_diff)

        # Publish ground truth for Questnav Sim - TODO: use this as the only sim pose
        self.ground_truth_pub.set(self.physics_controller.get_pose())

        self.robot.container.swerve.pose_estimator.resetPosition(gyroAngle=self.physics_controller.get_pose().rotation(), wheelPositions=[SwerveModulePosition()] * 4, pose=self.physics_controller.get_pose())

        self.navx_yaw.set(self.navx_yaw.get() - math.degrees(speeds.omega * tm_diff))

    # ---------------------------- SUBSYSTEM INITIALIZATIONS -----------------------

    def initialize_swerve(self):
        self.kinematics: SwerveDrive4Kinematics = dc.kDriveKinematics  # our swerve drive kinematics

        # set up LEDs - apparently not necessary - glass gui grabs the default one and you can show it
        # self.ledsim = simlib.AddressableLEDSim()

        # NavX (SPI interface) - no idea why the "4" is there, seems to be the default name generated by the navx code
        self.navx = simlib.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")  # for some reason it seems we have to set Yaw and not Angle
        self.navx_angle = self.navx.getDouble("Angle")

        self.analogs = [simlib.AnalogInputSim(i) for i in range(4)]
        self.analog_offsets = []

        # create a dictionary so we can refer to the sparks by name and get their relevant parameters
        self.spark_dict = {}
        # kinematics chassis speeds wants them in same order as in original definition - unfortunate ordering
        # TODO - get these from swerve_constants
        self.spark_drives = ['lf_drive', 'rf_drive', 'lb_drive', 'rb_drive']
        self.spark_drive_ids = [21, 25, 23, 27]  # keep in this order - based on our kinematics definition
        self.spark_turns = ['lf_turn', 'rf_turn', 'lb_turn', 'rb_turn']
        self.spark_turn_ids = [20, 24, 22, 26]  # keep in this order

        # Got rid of last year's elements: 'br_crank', 'bl_crank', 'tr_crank', 'tl_crank', 't_shooter', 'b_shooter'
        self.spark_peripherals = ['intake', 'indexer']
        self.spark_peripheral_ids = [5, 12] # Kept  'indexer' id as 12 because it came last before removing the elements

        # allow ourselves to access the simdevice's Position, Velocity, Applied Output, etc
        self.spark_names = self.spark_drives + self.spark_turns + self.spark_peripherals
        self.spark_ids = self.spark_drive_ids + self.spark_turn_ids + self.spark_peripheral_ids
        for idx, (spark_name, can_id) in enumerate(zip(self.spark_names, self.spark_ids)):
            spark = simlib.SimDeviceSim(f'SPARK MAX [{can_id}]')
            position = spark.getDouble('Position')
            velocity = spark.getDouble('Velocity')
            output = spark.getDouble('Applied Output')
            self.spark_dict.update({spark_name: {'controller': spark, 'position': position,
                                                 'velocity': velocity, 'output': output}})
        for key, value in self.spark_dict.items():  # see if these make sense
            # print(f'{key}: {value}')
            pass

        self.distances = [0, 0, 0, 0]

        # set up the initial location of the robot on the field
        self.x, self.y = constants.k_start_x, constants.k_start_y
        self.theta = 0
        initial_pose = Pose2d(0, 0, Rotation2d())
        self.physics_controller.move_robot(Transform2d(self.x, self.y, 0))