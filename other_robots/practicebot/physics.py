import math
from rev import SparkFlex, SparkFlexSim, SparkMaxSim
import wpilib
import wpilib.simulation as simlib  # 2021 name for the simulation library
from  wpimath.geometry import Pose2d, Rotation2d, Transform2d
from wpimath.kinematics._kinematics import SwerveDrive4Kinematics, SwerveModuleState, SwerveModulePosition
from pyfrc.physics.core import PhysicsInterface
import ntcore

from robot import MyRobot
import constants
from subsystems.swerve_constants import DriveConstants as dc

class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # Copied from 2024 code
        self.physics_controller = physics_controller  # must have for simulation
        self.robot = robot

        # if we want to add an armsim see test_robots/sparksim_test/


        self.initialize_swerve()


        # vision stuff - using 2024 stuff for now (CJH)
        key = 'orange'
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.camera_dict = {}
        self.cam_list = ['ArducamHigh', 'ArducamBack', 'GeniusLow', 'LogitechReef',]
        for ix, cam in enumerate(self.cam_list):
            table = self.inst.getTable(f'/Cameras/{cam}')   # test cam for sim
            targets_entry = table.getEntry(f"{key}/targets")
            distance_entry = table.getEntry(f"{key}/distance")
            strafe_entry = table.getEntry(f"{key}/strafe")
            rotation_entry = table.getEntry(f"{key}/rotation")
            timestamp_entry = table.getEntry(f"_timestamp")
            self.camera_dict[cam] = {'table': table, 'offset': ix, 'targets_entry': targets_entry, 'distance_entry': distance_entry,
                                     'strafe_entry': strafe_entry, 'rotation_entry': rotation_entry, 'timestamp_entry': timestamp_entry}

    def update_sim(self, now, tm_diff):

        # simlib.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue2)
        amps = []

        self.update_swerve(tm_diff)

        simlib.RoboRioSim.setVInVoltage(
                simlib.BatterySim.calculate(amps)
        )
        self.update_vision()

    # ------------------ SUBSYSTEM UPDATES --------------------


    def update_vision(self):
        # update the vision - using 2024 stuff for now (CJH)
        ring_dist, ring_rot = 2.22, 3.22 #self.distance_to_ring()
        wpilib.SmartDashboard.putNumber('/sim/hub_dist', round(ring_dist, 2))
        wpilib.SmartDashboard.putNumber('/sim/hub_rot', round(ring_rot, 2))
        for cam in self.cam_list:
            offset = self.camera_dict[cam]['offset']
            self.camera_dict[cam]['targets_entry'].setDouble(1 + offset)
            self.camera_dict[cam]['distance_entry'].setDouble(ring_dist + offset)
            self.camera_dict[cam]['strafe_entry'].setDouble(0)
            self.camera_dict[cam]['rotation_entry'].setDouble(self.theta - ring_rot)
            ts = wpilib.Timer.getFPGATimestamp() if wpilib.Timer.getFPGATimestamp() % 30 < (30/5)*(1+offset) else wpilib.Timer.getFPGATimestamp() -1  # simulate cameras dropping out
            self.camera_dict[cam]['timestamp_entry'].setDouble(ts)  # pretend the camera is live


    def update_swerve(self, tm_diff):

        dash_values = ['lf_target_vel_angle', 'rf_target_vel_angle', 'lb_target_vel_angle', 'rb_target_vel_angle']
        target_angles = [wpilib.SmartDashboard.getNumberArray(dash_value, [0, 0])[1] for dash_value in dash_values]
        for spark_turn, target_angle in zip(self.spark_turns, target_angles):
            self.spark_dict[spark_turn]['position'].set(target_angle)  # this works to update the simulated spark
        if constants.k_swerve_debugging_messages:
            wpilib.SmartDashboard.putNumberArray('target_angles', target_angles)

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
