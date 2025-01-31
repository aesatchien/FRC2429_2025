import math
from rev import SparkMaxSim, SparkSim
import wpilib
import hal
import wpilib.simulation as simlib  # 2021 name for the simulation library
import wpimath
import wpimath.geometry as geo
from wpimath.system.plant import DCMotor
from wpimath.kinematics._kinematics import SwerveDrive4Kinematics, SwerveModuleState, SwerveModulePosition
from pyfrc.physics.core import PhysicsInterface
from wpimath.units import feetToMeters

from robot import MyRobot
import constants
import simmech as sm
from subsystems.swerve_constants import DriveConstants as dc

class PhysicsEngine:

    def __init__(self, physics_controller: PhysicsInterface, robot: MyRobot):
        # Copied from 2024 code
        self.physics_controller = physics_controller  # must have for simulation
        self.robot = robot

        # if we want to add an armsim see test_robots/sparksim_test/
        self.initialize_swerve()
        self.initialize_wrist()
        self.update_elevator_positions()

    def update_sim(self, now, tm_diff):
        # simlib.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue2)

        amps = [0] # list of current draws
        self.update_swerve(tm_diff)
        self.update_intake(tm_diff)
        amps.append(self.update_wrist(tm_diff))
        self.update_elevator_positions()

        simlib.RoboRioSim.setVInVoltage(
                simlib.BatterySim.calculate(amps)
        )


    def update_wrist(self, tm_diff):
        self.wrist_sim.setInput(0, self.wrist_spark_sim.getAppliedOutput() * simlib.RoboRioSim.getVInVoltage())
        self.wrist_sim.update(tm_diff)
        self.wrist_spark_sim.iterate(velocity=self.wrist_sim.getVelocity(), vbus=simlib.RoboRioSim.getVInVoltage(),
                                     dt=tm_diff)
        return self.wrist_sim.getCurrentDraw()

    def update_intake(self, tm_diff):
        self.robot.container.intake.sparkmax_sim.iterate(self.robot.container.intake.sparkmax_sim.getVelocity(), 12, tm_diff)

    def update_elevator_positions(self):
        if self.robot is None:
            raise ValueError("Robot is not defined")
        
        self.elevator_height_sim = self.robot.container.elevator.get_height() * (constants.ElevatorConstants.k_elevator_sim_max_height / constants.ElevatorConstants.k_elevator_max_height)
        self.shoulder_pivot = self.robot.container.double_pivot.get_shoulder_pivot()
        self.elbow_pivot = self.robot.container.double_pivot.get_elbow_pivot()
        
        sm.front_elevator.components["elevator_right"]["ligament"].setLength(self.elevator_height_sim)
        sm.front_elevator.components["elevator_left"]["ligament"].setLength(self.elevator_height_sim)
        
        sm.side_elevator.components["elevator_side"]["ligament"].setLength(self.elevator_height_sim)
        sm.side_elevator.components["double_pivot_shoulder"]["ligament"].setAngle(self.shoulder_pivot)
        sm.side_elevator.components["double_pivot_elbow"]["ligament"].setAngle(self.elbow_pivot)

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
                self.spark_dict[drive]['velocity'].value, geo.Rotation2d(self.spark_dict[turn]['position'].value))
            )

        # using our own kinematics to update the chassis speeds
        module_states = self.robot.container.swerve.get_desired_swerve_module_states()
        speeds = self.kinematics.toChassisSpeeds(tuple(module_states))

        # update the sim's robot
        self.physics_controller.drive(speeds, tm_diff)

        self.robot.container.swerve.pose_estimator.resetPosition(gyroAngle=self.physics_controller.get_pose().rotation(), wheelPositions=[SwerveModulePosition()] * 4, pose=self.physics_controller.get_pose())

        self.navx_yaw.set(self.navx_yaw.get() - math.degrees(speeds.omega * tm_diff))

    # ---------------------------- INITIALIZATIONS -----------------------

    def initialize_wrist(self):

        self.wrist_sim = simlib.SingleJointedArmSim(gearbox=constants.WristConstants.k_plant,
                                                    gearing=constants.WristConstants.k_gear_ratio,
                                                    moi=constants.WristConstants.k_moi,
                                                    armLength=constants.WristConstants.k_center_of_mass_to_axis_of_rotation_dist_meters, # not sure how to make this accurate
                                                    minAngle=constants.WristConstants.k_min_angle,
                                                    maxAngle=constants.WristConstants.k_max_angle,
                                                    simulateGravity=False,
                                                    startingAngle=constants.WristConstants.k_sim_starting_angle)

        self.wrist_spark_sim = SparkMaxSim(self.robot.container.wrist.sparkmax, motor=constants.WristConstants.k_plant)


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
            print(f'{key}: {value}')

        self.distances = [0, 0, 0, 0]

        # set up the initial location of the robot on the field
        self.x, self.y = constants.k_start_x, constants.k_start_y
        self.theta = 0
        initial_pose = geo.Pose2d(0, 0, geo.Rotation2d())
        self.physics_controller.move_robot(geo.Transform2d(self.x, self.y, 0))

