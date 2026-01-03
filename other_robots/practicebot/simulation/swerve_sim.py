import math
import wpilib.simulation as simlib
from wpimath.geometry import Pose2d, Pose3d, Rotation3d, Transform2d, Translation3d
from wpimath.kinematics import SwerveModulePosition
import ntcore
import constants
from subsystems.swerve_constants import DriveConstants as dc

class SwerveSim:
    def __init__(self, physics_controller, robot):
        self.physics_controller = physics_controller
        self.robot = robot
        self.kinematics = dc.kDriveKinematics
        
        self.inst = ntcore.NetworkTableInstance.getDefault()
        
        # Swerve Debugging
        self.target_angles_pub = self.inst.getDoubleArrayTopic(f"{constants.sim_prefix}/swerve_target_angles").publish()
        
        # Swerve Target Subscribers
        dash_values = ['lf_target_vel_angle', 'rf_target_vel_angle', 'lb_target_vel_angle', 'rb_target_vel_angle']
        self.swerve_target_subs = [self.inst.getDoubleArrayTopic(f"/SmartDashboard/{v}").subscribe([0, 0]) for v in dash_values]

        # Live Tag Subscribers (for snapping sim to reality)
        self.camera_names = [config['topic_name'] for config in constants.k_cameras.values() if config['type'] == 'tags']
        self.pose_subscribers = [self.inst.getDoubleArrayTopic(f"/Cameras/{cam}/poses/tag1").subscribe([0] * 7) for cam in self.camera_names]
        self.count_subscribers = [self.inst.getDoubleTopic(f"/Cameras/{cam}/tags/targets").subscribe(0) for cam in self.camera_names]

        self._initialize_sim_devices()

    def _initialize_sim_devices(self):
        # NavX
        self.navx = simlib.SimDeviceSim("navX-Sensor[4]")
        self.navx_yaw = self.navx.getDouble("Yaw")
        
        # Sparks
        self.spark_dict = {}
        self.spark_drives = ['lf_drive', 'rf_drive', 'lb_drive', 'rb_drive']
        self.spark_drive_ids = [21, 25, 23, 27]
        self.spark_turns = ['lf_turn', 'rf_turn', 'lb_turn', 'rb_turn']
        self.spark_turn_ids = [20, 24, 22, 26]
        
        self.spark_names = self.spark_drives + self.spark_turns
        self.spark_ids = self.spark_drive_ids + self.spark_turn_ids
        
        for spark_name, can_id in zip(self.spark_names, self.spark_ids):
            spark = simlib.SimDeviceSim(f'SPARK MAX [{can_id}]')
            position = spark.getDouble('Position')
            velocity = spark.getDouble('Velocity')
            output = spark.getDouble('Applied Output')
            self.spark_dict.update({spark_name: {'controller': spark, 'position': position,
                                                 'velocity': velocity, 'output': output}})

    def update(self, tm_diff):
        # Update simulated spark positions from target angles (perfect response)
        target_angles = [sub.get()[1] for sub in self.swerve_target_subs]
        for spark_turn, target_angle in zip(self.spark_turns, target_angles):
            self.spark_dict[spark_turn]['position'].set(target_angle)
            
        if constants.k_swerve_debugging_messages:
            self.target_angles_pub.set(target_angles)

        # Get desired states from robot code (for kinematics)
        module_states = self.robot.container.swerve.get_desired_swerve_module_states()
        speeds = self.kinematics.toChassisSpeeds(tuple(module_states))

        # Update physics controller
        self.physics_controller.drive(speeds, tm_diff)
        
        # Update Robot Odometry (Perfect Odometry for Sim)
        pose = self.physics_controller.get_pose()
        self.robot.container.swerve.pose_estimator.resetPosition(gyroAngle=pose.rotation(), wheelPositions=[SwerveModulePosition()] * 4, pose=pose)

        # Update NavX
        self.navx_yaw.set(self.navx_yaw.get() - math.degrees(speeds.omega * tm_diff))

        # --- Live Tag Snapping ---
        if constants.SimConstants.k_use_live_tags_in_sim:
            self._snap_to_live_tags()

    def _snap_to_live_tags(self):
        """
        If a live camera sees a tag, move the simulated robot to that location.
        This allows testing vision-based localization in the simulator using real hardware.
        """
        for count_sub, pose_sub in zip(self.count_subscribers, self.pose_subscribers):
            if count_sub.get() > 0:
                atomic_data = pose_sub.getAtomic()
                tag_data = atomic_data.value
                timestamp_us = atomic_data.time

                # Check for freshness (e.g., < 0.1s old to avoid jitter from stale data)
                # NOTE: This also prevents the simulation from snapping to its own simulated tags.
                # vision_sim.py updates 'targets' but NOT the 'poses' topic.
                # Therefore, simulated tags will have stale timestamps here and be ignored.
                if ntcore._now() - timestamp_us < 100000: 
                    tx, ty, tz = tag_data[1], tag_data[2], tag_data[3]
                    rx, ry, rz = tag_data[4], tag_data[5], tag_data[6]
                    
                    # Calculate the robot's field pose from the tag data
                    vision_pose = Pose3d(Translation3d(tx, ty, tz), Rotation3d(rx, ry, rz)).toPose2d()
                    
                    # Move the physics controller to this pose
                    current_sim_pose = self.physics_controller.get_pose()
                    transform = Transform2d(current_sim_pose, vision_pose)
                    self.physics_controller.move_robot(transform)
                    
                    # Only use the first valid tag we find to avoid fighting
                    return