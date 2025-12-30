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
        
        # Simulation flags
        self.do_blink_test = False  # Set to True to test dashboard connection handling
        
        # Create a Field2d for visualization
        self.field = wpilib.Field2d()
        wpilib.SmartDashboard.putData("Field", self.field)  # this should just keep the default one

    def _init_networktables(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        sim_prefix = constants.sim_prefix
        camera_prefix = constants.camera_prefix
        
        # Vision Sim Publishers
        self.sim_hub_dist_pub = self.inst.getDoubleTopic(f"{sim_prefix}/hub_dist").publish()
        self.sim_hub_rot_pub = self.inst.getDoubleTopic(f"{sim_prefix}/hub_rot").publish()

        self.camera_dict = {}
        self.cam_list = list(constants.k_cameras.keys())
        self.physical_cameras = sorted(list(set(c['topic_name'] for c in constants.k_cameras.values())))

        # vision stuff - using 2024 stuff for now (CJH).  This could easily be extended to make fake tags as well
        # then you could use more of the tag stuff in vision, and the tag faking could be here instead of there
        # this mimics what the pis would send if they were live  TODO - allow live pis for testing
        for ix, (key, config) in enumerate(constants.k_cameras.items()):
            cam_topic = config['topic_name']
            cam_type = config['type']
            base = f'/Cameras/{cam_topic}/{cam_type}'
            
            self.camera_dict[key] = {
                'offset': ix,
                'timestamp_pub': self.inst.getDoubleTopic(f"/Cameras/{cam_topic}/_timestamp").publish(),
                'targets_pub': self.inst.getDoubleTopic(f"{base}/targets").publish(),
                'distance_pub': self.inst.getDoubleTopic(f"{base}/distance").publish(),
                'strafe_pub': self.inst.getDoubleTopic(f"{base}/strafe").publish(),
                'rotation_pub': self.inst.getDoubleTopic(f"{base}/rotation").publish()
            }

        if constants.SimConstants.k_print_config:
            print('\n*** PHYSICS.PY CAMERA DICT ***')
            for key, item in self.camera_dict.items():
                print(f'{key}: {item}')
            print()

        # ground truth Publisher for Simulating Sensors
        self.ground_truth_pub = self.inst.getStructTopic(f"{sim_prefix}/ground_truth", Pose2d).publish()

        # Swerve Debugging
        self.target_angles_pub = self.inst.getDoubleArrayTopic(f"{sim_prefix}/target_angles").publish()
        
        # Swerve Target Subscribers
        dash_values = ['lf_target_vel_angle', 'rf_target_vel_angle', 'lb_target_vel_angle', 'rb_target_vel_angle']
        self.swerve_target_subs = [self.inst.getDoubleArrayTopic(f"/SmartDashboard/{v}").subscribe([0, 0]) for v in dash_values]

    def update_sim(self, now, tm_diff):

        # simlib.DriverStationSim.setAllianceStationId(hal.AllianceStationID.kBlue2)
        amps = []

        self.update_swerve(tm_diff)
        self.consume_gamepieces()

        simlib.RoboRioSim.setVInVoltage(
                simlib.BatterySim.calculate(amps)
        )
        self.update_vision()
        
        # Update Field2d
        self.field.setRobotPose(self.physics_controller.get_pose())
        active_poses = [Pose2d(gp['pos'], Rotation2d()) for gp in self.gamepieces if gp['active']]
        self.field.getObject("Gamepieces").setPoses(active_poses)

    # ------------------ SUBSYSTEM UPDATES --------------------

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
                if self.is_on_gamepiece(gp['pos']):
                    gp['active'] = False
                    print(f"Simulation consumed gamepiece at {gp['pos']}")
        if len([gp for gp in self.gamepieces if gp['active']]) == 0:
            self.reset_gamepieces()


    def update_vision(self):
        # update the vision - using 2024 stuff for now (CJH)
        # Calculate closest gamepiece to the robot for general debugging
        dist, rot, strafe, target_pose = self.get_closest_gamepiece()
        
        self.sim_hub_dist_pub.set(round(dist, 2))
        self.sim_hub_rot_pub.set(round(rot, 2))
        
        self.update_simulated_cameras()

    def update_simulated_cameras(self):
        now = wpilib.Timer.getFPGATimestamp()
        robot_pose = self.physics_controller.get_pose()
        
        # Determine blink test state
        topic_off = None
        key_target_on = None
        if self.do_blink_test:
            # Rotate which physical camera is disconnected (15s per camera)
            topic_off = self.physical_cameras[int(now / 15.0) % len(self.physical_cameras)]
            # Rotate which logical camera sees targets (2s per camera) - ONE ON AT A TIME
            key_target_on = self.cam_list[int(now / 2.0) % len(self.cam_list)]

        for key, cam_data in self.camera_dict.items():
            config = constants.k_cameras[key]
            topic = config['topic_name']
            cam_rot = config.get('rotation', 0)
            cam_fov = config.get('fov', 90)

            # 1. Calculate Real Visibility & Data
            # Find closest gamepiece visible to THIS camera
            targets = 0
            dist = 0
            rot = 0
            strafe = 0

            # Check all gamepieces
            visible_gps = []
            for gp_data in self.gamepieces:
                if not gp_data['active']: continue
                gp = gp_data['pos']
                # Vector from robot center to gamepiece
                vec_to_gp = gp - robot_pose.translation()
                
                # Angle to gamepiece relative to robot heading
                angle_robot_relative = (vec_to_gp.angle() - robot_pose.rotation()).degrees()
                # Normalize to -180 to 180
                angle_robot_relative = (angle_robot_relative + 180) % 360 - 180

                # Angle relative to camera mount
                angle_cam_relative = angle_robot_relative - cam_rot
                # Normalize again
                angle_cam_relative = (angle_cam_relative + 180) % 360 - 180

                # Check FOV
                if abs(angle_cam_relative) < (cam_fov / 2.0):
                    d = vec_to_gp.norm()
                    # Camera-relative strafe (lateral offset in camera view)
                    s = d * math.sin(math.radians(angle_cam_relative))
                    visible_gps.append({'dist': d, 'rot': angle_cam_relative, 'strafe': s})

            if visible_gps:
                # Pick the closest one
                closest = min(visible_gps, key=lambda x: x['dist'])
                targets = len(visible_gps)  # TODO - reduce this to targets < x m, say ~5
                dist = closest['dist']
                rot = closest['rot']
                strafe = closest['strafe']

            # 2. Apply Blink Test Overrides (if enabled)
            is_connected = True
            if self.do_blink_test:
                is_connected = (topic != topic_off)
                # If connected, only show targets if it's this camera's turn
                if is_connected and key != key_target_on:
                    targets = 0
            
            # Publish timestamp (stale if disconnected) and targets (0 if disconnected or not the active target)
            cam_data['timestamp_pub'].set(now if is_connected else now - 5)
            cam_data['targets_pub'].set(targets if is_connected else 0)
            
            cam_data['distance_pub'].set(dist)
            cam_data['strafe_pub'].set(strafe)
            cam_data['rotation_pub'].set(rot)


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


    # ------------------ HELPER FUNCTIONS FOR STUDENTS --------------------

    def get_distance(self, pos1: Translation2d, pos2: Translation2d) -> float:
        """
        Calculate the Euclidean distance between two positions.
        Useful for determining how far the robot is from a target.

        :param pos1: The first Translation2d
        :param pos2: The second Translation2d
        :return: Distance in meters
        """
        return pos1.distance(pos2)

    def distance_to_gamepiece(self, gamepiece_position: Translation2d) -> tuple[float, float]:
        """
        Calculate the distance and rotation from the robot's current simulated position to a gamepiece.

        :param gamepiece_position: The Translation2d of the gamepiece (e.g. a Note or Coral)
        :return: Tuple (distance_in_meters, rotation_needed_in_degrees)
        """
        robot_pose = self.physics_controller.get_pose()
        distance = self.get_distance(robot_pose.translation(), gamepiece_position)

        to_target = gamepiece_position - robot_pose.translation()
        rotation_needed = to_target.angle() - robot_pose.rotation()

        return distance, rotation_needed.degrees()

    def is_on_gamepiece(self, gamepiece_position: Translation2d, threshold: float = 0.5) -> bool:
        """
        Check if the robot is "on" (close enough to interact with) a gamepiece.

        :param gamepiece_position: The Translation2d of the gamepiece
        :param threshold: The distance threshold in meters (default 0.5m)
        :return: True if within threshold, False otherwise
        """
        dist, _ = self.distance_to_gamepiece(gamepiece_position)
        return dist < threshold

    def get_closest_gamepiece(self, gamepieces: list[Translation2d] = None) -> tuple[float, float, float, Pose2d]:
        """
        Find the closest gamepiece from a list of gamepieces.

        This is useful for autonomous modes where the robot needs to decide
        which gamepiece to go for next.

        :param gamepieces: A list of Translation2d objects representing gamepieces on the field. 
                           If None, uses the default field gamepieces.
        :return: A tuple containing (distance, rotation, strafe, target_pose).
                 distance: Euclidean distance in meters
                 rotation: Angle in degrees relative to robot front (CCW positive)
                 strafe:   Lateral offset in meters relative to robot (Left positive)
                 target_pose: Pose2d of the gamepiece, rotated to face the robot
                 Returns (inf, 0, 0, Pose2d()) if list is empty.
        """
        if gamepieces is None:
            gamepieces = [gp['pos'] for gp in self.gamepieces if gp['active']]

        if not gamepieces:
            return float('inf'), 0, 0, Pose2d()

        robot_pose = self.physics_controller.get_pose()
        robot_translation = robot_pose.translation()
        
        # Find the gamepiece with the minimum distance to the robot
        closest = min(gamepieces, key=lambda gp: self.get_distance(robot_translation, gp))
        
        # Calculate properties
        dist, rot = self.distance_to_gamepiece(closest) # rot is degrees relative to robot
        
        # Calculate strafe (Y component in robot frame)
        # Vector to target, rotated by -robot_heading, gives vector in robot frame
        vec_robot_frame = (closest - robot_translation).rotateBy(-robot_pose.rotation())
        strafe = vec_robot_frame.Y()
        
        # Target Pose: Location of gamepiece, facing the robot (angle from robot to GP)
        # If we drive to this pose, we will be at the gamepiece facing the direction we came from?
        # User requested: "rotation of that Pose2D will be the rotation what will have the front of our robot facing the gamepiece when we get to it."
        # This implies the rotation of the vector FROM robot TO gamepiece.
        target_rotation = (closest - robot_translation).angle()
        target_pose = Pose2d(closest, target_rotation)

        return dist, rot, strafe, target_pose